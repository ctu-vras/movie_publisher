// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Publisher of movie files to ROS image topics.
 * \author Martin Pecka
 */

#include <getopt.h>
#include <memory>
#include <string>
#include <thread>

#include <fmt/format.h>

#include CXX_FILESYSTEM_INCLUDE
namespace fs = CXX_FILESYSTEM_NAMESPACE;

#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/functional.hpp>
#include <cras_cpp_common/node_utils.hpp>
#include <cras_cpp_common/node_utils/node_with_optional_master.h>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/param_utils.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/tqdm.hpp>
#include <gps_common/GPSFix.h>
#include <image_transport_codecs/image_transport_codecs.h>
#include <movie_publisher/movie.h>
#include <movie_publisher/movie_to_bag.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/common.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_msgs/TFMessage.h>

namespace movie_publisher
{
/**
 * \brief Convert movie files and their metadata to ROS bag file.
 *
 * \par Stored topics
 *
 * - `${~topic}` (`sensor_msgs/Image`): The published movie (if raw transport is used).
 * - `${~topic}/${~transport}` (*): The published movie compressed stream (if raw transport is not used).
 * - `${~topic}/camera_info` (`sensor_msgs/CameraInfo`): Camera info.
 * - `${~topic}/azimuth` (`compass_msgs/Azimuth`): Georeferenced heading of the camera.
 * - `${~topic}/fix` (`sensor_msgs/NavSatFix`): GNSS position of the camera.
 * - `${~topic}/fix_detail` (`gps_common/GPSFix`): GNSS position of the camera.
 * - `${~topic}/imu` (`sensor_msgs/Imu`): Orientation and acceleration of the camera.
 *
 * To extract the additional topics except `movie`, the node uses instances of MetadataExtractor.
 *
 * To change the prefix of all topics, set `~topic` parameter. To change the name of a single topic, remap it.
 *
 * \par Parameters
 *
 * Parameters `~start`, `~end` and `~duration` can be expressed in seconds `(15.35)`, in `(min, sec)`,
 * in `(hour, min, sec)`, or as a string: `'01:03:05.35'`.
 *
 * - `~bag` (string, required): Path where the result should be stored.
 * - `~overwrite_bag` (bool, default false): If true and the bag file exists, it will be overwritten. Otherwise, it will
 *                                           be appended (and created if needed).
 * - `~movie` (string, required): Path to the movie to play. Any format that ffmpeg can decode.
 * - `~transport` (string, default `raw`, suggested `compressed`): The image_transport used to store the movie
 *                                                                 in the bag.
 * - `~start` (float|tuple|string, optional): If set, the movie will be read from the specified time.
 *                                            Cannot be set together with `~end` and `~duration`.
 * - `~end` (float|tuple|string, optional): If set, the movie will be read up to the specified time (not affected by
 *                                          start). Cannot be set together with `~start` and `~duration`.
 * - `~duration` (float|tuple|string, optional): If set, playback will have this duration. If end is also set, the
 *                                               duration is counted from the end of the clip, otherwise, it is the
 *                                               duration from the start of the clip. Cannot be set together with
 *                                               `~start` and `~end`.
 * - `~timestamp_offset` (int|float|string, default 0.0): Adjustment of timestamps determined by `~timestamp_source`.
 *                                                        If given as string, it can be a simple mathematical expression
 *                                                        that can also resolve several variables:
 *                                                        `ros_time` (current ROS time),
 *                                                        `wall_time` (current wall time),
 *                                                        `metadata_start` (start time from metadata),
 *                                                        `bag_start` (start time of the bag file),
 *                                                        `bag_end` (end time of the metadata),
 *                                                        `bag_duration` (duration of the bag file in s).
 * - `~timestamp_source` (str, default `metadata`): How to determine timestamps of the movie frames. Options are:
 *   - `metadata`: Extract absolute time when the movie was recorded and use that time as timestamps.
 *   - `all_zeros`: Use zero timestamps. Please note that time 0.0 cannot be stored in bag files. Use
 *                  `~timestamp_offset` to make the time valid.
 *   - `absolute_timecode`: Use the absolute timecode as timestamps (i.e. time since start of movie file).
 *   - `relative_timecode`: Use the relative timecode as timestamps (i.e. time since `~start`).
 *   - `ros_time`: Timestamp the frames with current ROS time. Note that this mode is not very useful for movie_to_bag.
 * - `~frame_id` (string, default ""): The frame_id used in the geometrical messages' headers.
 * - `~optical_frame_id` (string, default `${frame_id}_optical_frame`): The frame_id used in the image messages'
 *                                                                      headers.
 * - `~verbose` (bool, default False): If True, logs info about every frame played.
 * - `~allow_yuv_fallback` (bool, default False): Set whether `YUV***` formats should be decoded to YUV422, or whether
 *                                                the default encoding should be used.
 * - `~default_encoding` (string, optional): Set the default encoding which should be used for output frames if there is
 *                                           no direct match between the libav pixel format and ROS image encodings.
 * - `~encoding` (string, optional): Set the encoding which should be used for output frames regardless of their source
 *                                   encoding (one of sensor_msgs::image_encodings constants).
 */
class MoviesToBags : public cras::NodeWithOptionalMaster
{
public:
  explicit MoviesToBags(const cras::LogHelperPtr& log) : cras::NodeWithOptionalMaster(log)
  {
  }

  void run(const ros::V_string& args, const cras::BoundParamHelperPtr& params)
  {
    std::list<std::string> movies;
    std::list<std::string> bags;
    std::string bagDir;

    // TODO getopt
    for (const auto& arg : args)
    {
      if (fs::is_directory(arg))
      {
        size_t numFiles {0_sz};
        for (const auto& entry : fs::recursive_directory_iterator(arg))
        {
          if (!fs::exists(entry) || !fs::is_regular_file(entry))
            continue;
          if (entry.path().extension() == ".bag")
            bags.push_back(entry.path());
          else
            movies.push_back(entry.path());
          numFiles++;
        }

        if (numFiles == 0)
          bagDir = arg;
      }
      else if (fs::is_regular_file(arg))
      {
        fs::path entry(arg);
        if (entry.extension() == ".bag")
          bags.push_back(entry);
        else
          movies.push_back(entry);
      }
    }

    this->loadParams(params);

    CRAS_INFO("Found %zu movies and %zu bag files.", movies.size(), bags.size());

    if (movies.empty())
    {
      CRAS_ERROR("No movies specified. Nothing to do. Exiting.");
      return;
    }

    if (this->movieReader == nullptr)
      this->movieReader = std::move(this->createReader(this->params));

    if (bags.empty())
    {
      if (this->clusterByUniqueCameraName)
        this->runMoviesOnlyClustered(movies);
      else
        this->runMoviesOnlyNonClustered(movies);
    }
    else
    {
      this->runMoviesAndBags(movies, bags);
    }
  }

protected:
  struct MovieToBagAssignment
  {
    std::string movie;
    std::string bag;
    std::string topic;
    std::string frameId;
    cras::optional<std::pair<StreamTime, StreamTime>> subclip;
  };

  void loadParams(const cras::BoundParamHelperPtr& params)
  {
    this->params = params;
    this->clusterByUniqueCameraName = params->getParam("cluster_by_unique_camera_name", false);
    this->transport = params->getParam("transport", "compressed");
    this->frameIdTemplate = params->getParam("frame_id_template", this->frameIdTemplate);
    this->topicTemplate = params->getParam("topic_template", this->topicTemplate);
    this->bagTemplate = params->getParam("bag_template", this->bagTemplate);
    this->appendToBags = params->getParam("append_to_bags", false);
  }

  std::unique_ptr<MovieReaderRos> createReader(const cras::BoundParamHelperPtr& params)
  {
    return std::make_unique<MovieReaderRos>(this->log, params);
  }

  std::shared_ptr<MovieToBagMetadataProcessor> createMetadataProcessor(const std::string& bagFilename,
  const std::string& transport, const cras::BoundParamHelperPtr& params, const std::string& topic)
  {
    return std::make_shared<MovieToBagMetadataProcessor>(this->log, bagFilename, transport,
      [this](const std::string& name) {return this->resolveName(name);}, params, topic);
  }

  cras::expected<void, std::string> run(const std::list<MovieToBagAssignment>& moviesAndBags)
  {
    for (const auto& [movie, bag, topic, frameId, maybeSubclip] : moviesAndBags)
    {
      if (!this->ok())
        break;

      auto metadataProcessor = this->createMetadataProcessor(bag, this->transport, this->params, topic);
      metadataProcessor->addTimestampOffsetVars(*this->movieReader);
      auto maybeConfig = this->movieReader->createDefaultConfig();
      if (!maybeConfig.has_value())
      {
        CRAS_WARN("Could not create config for reading %s. Skipping.", movie.c_str());
        continue;
      }

      auto config = *maybeConfig;
      config.metadataProcessors().push_back(metadataProcessor);

      config.setFrameId(frameId);
      config.setOpticalFrameId(config.frameId() + "_optical_frame");

      if (maybeSubclip.has_value())
      {
        config.setSubClip(maybeSubclip->first, maybeSubclip->second, {});
        CRAS_INFO("Limiting movie to %s - %s",
          cras::to_string(maybeSubclip->first).c_str(), cras::to_string(maybeSubclip->second).c_str());
      }

      const auto maybeMovie = this->movieReader->open(movie, config);
      if (!maybeMovie.has_value())
      {
        CRAS_ERROR("Failed to open movie file '%s' due to the following error: %s",
          movie.c_str(), maybeMovie.error().c_str());
        continue;
      }
      auto openMovie = *maybeMovie;

      CRAS_INFO("Writing movie %s to bag %s", movie.c_str(), bag.c_str());

      if (maybeSubclip.has_value())
        openMovie->setSubClip(maybeSubclip->first, maybeSubclip->second, {});

      tq::progress_bar tqdm;
      const auto numFrames = openMovie->info()->subclipNumFrames();
      size_t frameNum = 0;
      tqdm.update(0.0);

      while (this->ok())
      {
        const auto maybePtsAndImg = openMovie->nextFrame();
        if (!maybePtsAndImg.has_value() || std::get<1>(*maybePtsAndImg) == nullptr)
        {
          if (!maybePtsAndImg.has_value())
          {
            CRAS_ERROR("Reading movie %s has failed with the following error: %s Stopped conversion.",
              movie.c_str(), maybePtsAndImg.error().c_str());
          }
          break;
        }

        frameNum++;
        tqdm.update(static_cast<double>(frameNum) / static_cast<double>(numFrames));

        const auto playbackState = std::get<0>(*maybePtsAndImg);
        const auto& subclipEnd = openMovie->info()->subclipEnd();
        if (!subclipEnd.isZero() && playbackState.streamTime() > subclipEnd)
          break;
      }
      tqdm.restart();

      CRAS_INFO("Reached end of movie %s.", movie.c_str());

      metadataProcessor->close();
      openMovie.reset();
      metadataProcessor.reset();

      CRAS_INFO("Saved %s to bag file %s on topic %s.", movie.c_str(), bag.c_str(), topic.c_str());
    }

    return {};
  }

  cras::expected<void, std::string> runMoviesOnlyNonClustered(const std::list<std::string>& movies)
  {
    std::list<MovieToBagAssignment> moviesAndBags;
    for (const auto& movie : movies)
    {
      const std::string bag = movie + ".bag";
      const auto topic = fmt::format(this->topicTemplate, fmt::arg("cam_num", 0));
      const auto frameId = fmt::format(this->frameIdTemplate, fmt::arg("cam_num", 0));
      moviesAndBags.push_back(MovieToBagAssignment{movie, bag, topic, frameId, {}});
    }
    return this->run(moviesAndBags);
  }

  cras::expected<void, std::string> runMoviesOnlyClustered(const std::list<std::string>& movies)
  {
    auto maybeConfig = this->movieReader->createDefaultConfig();
    if (!maybeConfig.has_value())
      return cras::make_unexpected("Could not create config for movie reader.");

    std::list<MovieToBagAssignment> moviesAndBags;
    for (const auto& movie : movies)
    {
      const auto maybeMovie = this->movieReader->open(movie, *maybeConfig);
      if (!maybeMovie.has_value())
      {
        CRAS_WARN("Failed to open movie file '%s' due to the following error: %s",
          movie.c_str(), maybeMovie.error().c_str());
        continue;
      }

      const auto& openedMovie = *maybeMovie;
      const auto metadata = openedMovie->staticMetadata();
      auto maybeCamName = metadata->getCameraUniqueName();
      if (!maybeCamName.has_value())
        maybeCamName = metadata->getCameraGeneralName();
      const auto camName = cras::toValidRosName(maybeCamName.value_or(movie), true, movie);

      const auto bag = fmt::format(this->bagTemplate, fmt::arg("cam_num", 0), fmt::arg("cam_name", camName));
      const auto topic = fmt::format(this->topicTemplate, fmt::arg("cam_num", 0), fmt::arg("cam_name", camName));
      const auto frameId = fmt::format(this->frameIdTemplate, fmt::arg("cam_num", 0), fmt::arg("cam_name", camName));
      moviesAndBags.push_back(MovieToBagAssignment{movie, bag, topic, frameId, {}});
    }

    return this->run(moviesAndBags);
  }

  cras::expected<void, std::string> runMoviesAndBags(
    const std::list<std::string>& movies, const std::list<std::string>& bags)
  {
    auto maybeConfig = this->movieReader->createDefaultConfig();
    if (!maybeConfig.has_value())
      return cras::make_unexpected("Could not create config for movie reader.");

    std::unordered_map<std::string, std::tuple<std::string, size_t, ros::Time, ros::Time>> movieInfo;
    std::unordered_map<std::string, size_t> camNums;

    for (const auto& movie : movies)
    {
      const auto maybeMovie = this->movieReader->open(movie, *maybeConfig);
      if (!maybeMovie.has_value())
      {
        CRAS_WARN("Failed to open movie file '%s' due to the following error: %s",
          movie.c_str(), maybeMovie.error().c_str());
        continue;
      }

      const auto& openedMovie = *maybeMovie;
      const auto& info = openedMovie->info();
      const auto metadata = openedMovie->staticMetadata();
      const auto startTime = openedMovie->convertTime(info->streamStart());
      const auto endTime = openedMovie->convertTime(info->streamEnd());

      auto maybeCamName = metadata->getCameraUniqueName();
      if (!maybeCamName.has_value())
        maybeCamName = metadata->getCameraGeneralName();
      const auto camName = cras::toValidRosName(maybeCamName.value_or(movie), true, movie);

      if (camNums.find(camName) == camNums.end())
        camNums[camName] = camNums.size();

      movieInfo[movie] = std::make_tuple(camName, camNums[camName], startTime, endTime);
    }

    std::list<MovieToBagAssignment> moviesAndBags;

    for (const auto& bag : bags)
    {
      if (!fs::exists(bag))
      {
        CRAS_WARN("Bag file %s does not exist.", bag.c_str());
        continue;
      }
      CRAS_INFO("Reading index of bag %s", bag.c_str());
      const rosbag::Bag openBag(bag, rosbag::BagMode::Read);
      rosbag::View bagView(openBag);
      const auto bagStart = bagView.getBeginTime();
      const auto bagEnd = bagView.getEndTime();

      for (const auto& [movie, info] : movieInfo)
      {
        const auto& [camName, camNum, movieStart, movieEnd] = info;
        if (movieStart >= bagEnd || movieEnd <= bagStart)
          continue;
        const auto bagName = this->appendToBags ? bag : fmt::format(this->bagTemplate,
          fmt::arg("cam_num", camNum), fmt::arg("cam_name", camName), fmt::arg("bag", bag),
          fmt::arg("bag_no_ext", bag.substr(0, bag.length() - 4)));
        const auto topic = fmt::format(this->topicTemplate, fmt::arg("cam_num", camNum), fmt::arg("cam_name", camName));
        const auto frameId = fmt::format(this->frameIdTemplate,
          fmt::arg("cam_num", camNum), fmt::arg("cam_name", camName));

        const auto subclipStartRos = std::max(movieStart, bagStart);
        const auto subclipEndRos = std::min(movieEnd, bagEnd);
        const StreamTime subclipStart(StreamDuration(subclipStartRos - movieStart));
        const StreamTime subclipEnd(StreamDuration(subclipEndRos - movieStart));

        moviesAndBags.push_back(
          MovieToBagAssignment{movie, bagName, topic, frameId, std::pair{subclipStart, subclipEnd}});
      }
    }

    return this->run(moviesAndBags);
  }

  cras::BoundParamHelperPtr params;
  bool clusterByUniqueCameraName {false};
  std::string transport;
  std::string frameIdTemplate {"external_cam_{cam_num}"};
  std::string topicTemplate {"external_cams/cam_{cam_num}"};
  std::string bagTemplate {"{bag_no_ext}.external_cams.bag"};
  bool appendToBags {false};

  std::unique_ptr<MovieReaderRos> movieReader;  //!< The movie reader.
  std::unordered_map<std::string, MoviePtr> movies;  //!< The opened movies.
};

}

int main(int argc, char* argv[])
{
  const auto log = std::make_shared<cras::NodeLogHelper>();
  movie_publisher::MoviesToBags node(log);

  // We're using NodeWithOptionalMaster, so this is instead of ros::init().
  auto options = ros::init_options::AnonymousName | ros::init_options::NoRosout;
#if ROS_VERSION_MINIMUM(1, 17, 0)
  options |= ros::init_options::NoSimTime;
#endif
  node.init(argc, argv, "movies_to_bags", options);

  const auto params = node.getPrivateParams();

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  if (args.size() > 1)
    args.erase(args.begin());

  node.run(args, params);

  return 0;
}

