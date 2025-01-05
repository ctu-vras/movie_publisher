// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Publisher of movie files to ROS image topics.
 * \author Martin Pecka
 */

#include <memory>
#include <string>
#include <thread>

#include CXX_FILESYSTEM_INCLUDE
namespace fs = CXX_FILESYSTEM_NAMESPACE;

#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/functional.hpp>
#include <cras_cpp_common/node_utils.hpp>
#include <cras_cpp_common/node_utils/node_with_optional_master.h>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/param_utils.hpp>
#include <gps_common/GPSFix.h>
#include <image_transport_codecs/image_transport_codecs.h>
#include <movie_publisher/movie_reader_ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
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
class MovieToBag : public cras::NodeWithOptionalMaster
{
public:
  explicit MovieToBag(const cras::LogHelperPtr& log) : cras::NodeWithOptionalMaster(log)
  {
  }

  void open(const std::string& bagFilename, const std::string& transport, const std::string& movieFilename,
    const cras::BoundParamHelperPtr& params)
  {
    this->reader = std::make_unique<MovieReaderRos>(this->log, params);
    this->transport = transport;

    const auto bagDir = fs::path(bagFilename).parent_path();
    try
    {
      fs::create_directories(bagDir);
    }
    catch (const fs::filesystem_error& e)
    {
      CRAS_WARN("Error creating directory [%s]: %s", bagDir.c_str(), e.what());
    }

    const auto overwriteBag = params->getParam("overwrite_bag", false);
    const auto bagExists = fs::exists(bagFilename);
    const auto bagMode = bagExists && !overwriteBag ? rosbag::BagMode::Append : rosbag::BagMode::Write;
    if (!bagExists)
      CRAS_INFO("Creating bag file %s", bagFilename.c_str());
    else if (bagMode == rosbag::BagMode::Append)
      CRAS_INFO("Appending bag file %s", fs::canonical(bagFilename).c_str());
    else
      CRAS_INFO("Overwriting bag file %s", fs::canonical(bagFilename).c_str());

    this->bag = std::make_unique<rosbag::Bag>(bagFilename, bagMode | rosbag::BagMode::Read);

    const auto bagView = std::make_shared<rosbag::View>(*this->bag);
    if (bagView->size() > 0)
    {
      this->reader->addTimestampOffsetVar("bag_start", bagView->getBeginTime().toSec());
      this->reader->addTimestampOffsetVar("bag_end", bagView->getEndTime().toSec());
      this->reader->addTimestampOffsetVar("bag_duration", (bagView->getEndTime() - bagView->getBeginTime()).toSec());
    }

    this->topic = params->getParam("topic", "movie");
    this->verbose = params->getParam("verbose", false);

    const auto openResult = this->reader->open(movieFilename);
    if (!openResult.has_value())
    {
      CRAS_FATAL("Failed to open movie file '%s' due to the following error: %s",
        movieFilename.c_str(), openResult.error().c_str());
      exit(1);
    }

    this->imageCodecs = std::make_unique<image_transport_codecs::ImageTransportCodecs>(this->log);

    const auto& opticalTfMsg = this->reader->getOpticalFrameTF();
    if (opticalTfMsg.has_value() && !this->reader->getFrameId().empty() && !this->reader->getOpticalFrameId().empty())
    {
      this->lastOpticalTf = opticalTfMsg->transform;
      tf2_msgs::TFMessage msg;
      msg.transforms.push_back(*opticalTfMsg);
      try
      {
        this->bag->write(this->resolveName("/tf_static"), opticalTfMsg->header.stamp, msg);
      }
      catch (const rosbag::BagIOException& e)
      {
        CRAS_ERROR_THROTTLE(1.0, cras::format("Failed to save static TF into bagfile: %s", e.what()));
      }
    }
  }

  void run()
  {
    size_t frameNum {0L};
    while (this->ok())
    {
      const auto maybePtsAndImg = this->reader->nextFrame();
      if (!maybePtsAndImg.has_value() || std::get<1>(*maybePtsAndImg) == nullptr)
      {
        if (!maybePtsAndImg.has_value())
        {
          CRAS_ERROR("Reading the movie has failed with the following error: %s Stopped conversion.",
            maybePtsAndImg.error().c_str());
          exit(4);
        }
        break;
      }

      const auto pts = std::get<0>(*maybePtsAndImg);
      if (!this->reader->getEnd().isZero() && pts > this->reader->getEnd())
        break;

      const auto img = std::get<1>(*maybePtsAndImg);

      if (this->verbose)
      {
        const auto timecode = pts - ros::Duration().fromNSec(this->reader->getStart().toNSec());
        const auto numFrames = this->reader->getNumFrames();

        CRAS_INFO("Frame %zu/%zu, time %s, timecode %s, stamp %s",
          frameNum, numFrames, cras::to_string(pts).c_str(), cras::to_string(timecode).c_str(),
          cras::to_string(img->header.stamp).c_str());
      }

      frameNum++;

      const auto encodedImage = this->imageCodecs->encode(*img, this->transport);
      if (encodedImage.has_value())
      {
        const auto topic = this->transport == "raw" ? this->topic : (this->topic + "/" + this->transport);
        try
        {
          this->bag->write(this->resolveName(topic), img->header.stamp, *encodedImage);
        }
        catch (const rosbag::BagIOException& e)
        {
          CRAS_ERROR_THROTTLE(1.0, cras::format("Failed to save image into bagfile: %s", e.what()));
        }
      }
      else
      {
        CRAS_ERROR_THROTTLE(1.0, cras::format("Failed to encode the image: %s", encodedImage.error().c_str()));
        continue;
      }

      const auto& cameraInfoMsg = this->reader->getCameraInfoMsg();
      if (cameraInfoMsg.has_value())
      {
        try
        {
          this->bag->write(
            this->resolveName(this->topic + "/camera_info"), cameraInfoMsg->header.stamp, *cameraInfoMsg);
        }
        catch (const rosbag::BagIOException& e)
        {
          CRAS_ERROR_THROTTLE(1.0, cras::format("Failed to save camera info into bagfile: %s", e.what()));
        }
      }

      const auto& azimuthMsg = this->reader->getAzimuthMsg();
      if (azimuthMsg.has_value())
      {
        try
        {
          this->bag->write(this->resolveName(this->topic + "/azimuth"), azimuthMsg->header.stamp, *azimuthMsg);
        }
        catch (const rosbag::BagIOException& e)
        {
          CRAS_ERROR_THROTTLE(1.0, cras::format("Failed to save azimuth info into bagfile: %s", e.what()));
        }
      }

      const auto& navSatFixMsg = this->reader->getNavSatFixMsg();
      if (navSatFixMsg.has_value())
      {
        try
        {
          this->bag->write(this->resolveName(this->topic + "/fix"), navSatFixMsg->header.stamp, *navSatFixMsg);
        }
        catch (const rosbag::BagIOException& e)
        {
          CRAS_ERROR_THROTTLE(1.0, cras::format("Failed to save fix into bagfile: %s", e.what()));
        }
      }

      const auto& gpsMsg = this->reader->getGpsMsg();
      if (gpsMsg.has_value())
      {
        try
        {
          this->bag->write(this->resolveName(this->topic + "/fix_detail"), gpsMsg->header.stamp, *gpsMsg);
        }
        catch (const rosbag::BagIOException& e)
        {
          CRAS_ERROR_THROTTLE(1.0, cras::format("Failed to save fix details into bagfile: %s", e.what()));
        }
      }

      const auto& imuMsg = this->reader->getImuMsg();
      if (imuMsg.has_value())
      {
        try
        {
          this->bag->write(this->resolveName(this->topic + "/imu"), imuMsg->header.stamp, *imuMsg);
        }
        catch (const rosbag::BagIOException& e)
        {
          CRAS_ERROR_THROTTLE(1.0, cras::format("Failed to save IMU into bagfile: %s", e.what()));
        }
      }

      const auto& zeroRollPitchTfMsg = this->reader->getZeroRollPitchTF();
      if (zeroRollPitchTfMsg.has_value())
      {
        tf2_msgs::TFMessage msg;
        msg.transforms.push_back(*zeroRollPitchTfMsg);
        try
        {
          this->bag->write(this->resolveName("/tf"), zeroRollPitchTfMsg->header.stamp, msg);
        }
        catch (const rosbag::BagIOException& e)
        {
          CRAS_ERROR_THROTTLE(1.0, cras::format("Failed to save TF into bagfile: %s", e.what()));
        }
      }

      const auto& opticalTfMsg = this->reader->getOpticalFrameTF();
      if (opticalTfMsg.has_value() &&
        !this->reader->getFrameId().empty() && !this->reader->getOpticalFrameId().empty() &&
        opticalTfMsg->transform != this->lastOpticalTf)
      {
        this->lastOpticalTf = opticalTfMsg->transform;
        tf2_msgs::TFMessage msg;
        msg.transforms.push_back(*opticalTfMsg);
        try
        {
          this->bag->write(this->resolveName("/tf_static"), opticalTfMsg->header.stamp, msg);
        }
        catch (const rosbag::BagIOException& e)
        {
          CRAS_ERROR_THROTTLE(1.0, cras::format("Failed to save static TF into bagfile: %s", e.what()));
        }
      }
    }

    CRAS_INFO("Reached end of movie.");
  }

private:
  std::unique_ptr<MovieReaderRos> reader;  //!< Movie reader instance.
  std::unique_ptr<image_transport_codecs::ImageTransportCodecs> imageCodecs;  //!< Image transport codec instance.
  std::unique_ptr<rosbag::Bag> bag;
  // ros::NodeHandle nh;

  //! Last determined optical TF (transform from geometrical to optical frame).
  geometry_msgs::Transform lastOpticalTf;

  bool verbose {false};  //!< Whether to print detailed info about playback.
  std::string topic;  //!< The base topic name.
  std::string transport;  //!< The transport to use.
};

}

int main(int argc, char* argv[])
{
  const auto log = std::make_shared<cras::NodeLogHelper>();
  movie_publisher::MovieToBag node(log);

  // We're using NodeWithOptionalMaster, so this is instead of ros::init().
  const auto options = ros::init_options::AnonymousName;
  node.init(argc, argv, "movie_to_bag", options);

  const auto params = node.getPrivateParams();
  const auto bagFilename = params->getParam<std::string>("bag", cras::nullopt);
  const auto movieFilename = params->getParam<std::string>("movie", cras::nullopt);
  const auto transport = params->getParam<std::string>("transport", "raw");

  node.open(bagFilename, transport, movieFilename, params);
  node.run();

  return 0;
}

