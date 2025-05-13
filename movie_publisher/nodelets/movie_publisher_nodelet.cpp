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

#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/functional.hpp>
#include <cras_cpp_common/nodelet_utils.hpp>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/param_utils.hpp>
#include <gps_common/GPSFix.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <movie_publisher/movie_metadata_processor.h>
#include <movie_publisher/parsing_utils.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <vision_msgs/Detection2DArray.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "movie_publisher/movie_reader_ros.h"

namespace movie_publisher
{

class NodeletMetadataProcessor : public MovieMetadataProcessor, protected cras::HasLogger
{
public:
  explicit NodeletMetadataProcessor(const cras::LogHelperPtr& log, const cras::BoundParamHelperPtr& params,
    const ros::NodeHandle& nh) : HasLogger(log), params(params)
  {
    this->topicsNh = ros::NodeHandle(nh, "movie");
    this->imageTransport = std::make_unique<image_transport::ImageTransport>(nh);
  }

  void shutdown()
  {
    this->imagePub.shutdown();
    this->cameraPub.shutdown();
    this->imageTransport.reset();
    this->azimuthPub.shutdown();
    this->navMsgPub.shutdown();
    this->gpsPub.shutdown();
    this->imuPub.shutdown();
    this->magPub.shutdown();
    this->facesPub.shutdown();
  }

  cras::expected<void, std::string> onMetadataReady(
    const std::shared_ptr<TimedMetadataExtractor>& metadataExtractor) override
  {
    const auto result = MovieMetadataProcessor::onMetadataReady(metadataExtractor);
    if (!result.has_value())
      return cras::make_unexpected(result.error());

    const auto immediateMode = this->params->getParam("immediate", false);
    const auto pubQueueSize = this->params->getParam("publisher_queue_size", immediateMode ? 1000 : 10, "messages");
    const auto waitAfterPublisherCreated =
      this->params->getParam("wait_after_publisher_created", ros::WallDuration(1));

    const auto& timedMeta = metadataExtractor->supportedTimedMetadata({});
    const auto hasTimedMeta = [&timedMeta](const MetadataType type)
    {
      return timedMeta.find(type) != timedMeta.end();
    };

    if (metadataExtractor->getCameraInfo().has_value() || hasTimedMeta(MetadataType::CAMERA_INFO))
      this->cameraPub = this->imageTransport->advertiseCamera("movie", pubQueueSize);
    else
      this->imagePub = this->imageTransport->advertise("movie", pubQueueSize);

    if (metadataExtractor->getAzimuth().has_value() || hasTimedMeta(MetadataType::AZIMUTH))
      this->azimuthPub = this->topicsNh.advertise<compass_msgs::Azimuth>("azimuth", pubQueueSize);
    if (metadataExtractor->getMagneticField().has_value() || hasTimedMeta(MetadataType::MAGNETIC_FIELD))
      this->magPub = this->topicsNh.advertise<sensor_msgs::MagneticField>("imu/mag", pubQueueSize);
    if (metadataExtractor->getGNSSPosition().first.has_value() || hasTimedMeta(MetadataType::GNSS_POSITION))
      this->navMsgPub = this->topicsNh.advertise<sensor_msgs::NavSatFix>("fix", pubQueueSize);
    if (metadataExtractor->getGNSSPosition().second.has_value() || hasTimedMeta(MetadataType::GNSS_POSITION))
      this->gpsPub = this->topicsNh.advertise<gps_common::GPSFix>("fix_detail", pubQueueSize);
    if (metadataExtractor->getImu().has_value() || hasTimedMeta(MetadataType::IMU))
      this->imuPub = this->topicsNh.advertise<sensor_msgs::Imu>("imu/data", pubQueueSize);
    if (metadataExtractor->getFaces().has_value() || hasTimedMeta(MetadataType::FACES))
      this->facesPub = this->topicsNh.advertise<vision_msgs::Detection2DArray>("faces", pubQueueSize);

    waitAfterPublisherCreated.sleep();

    return {};
  }

  cras::expected<void, std::string> repeatLastFrame()
  {
    if (this->lastImageMsg != nullptr)
      return this->processImage(this->lastImageMsg, this->lastCameraInfoMsg);
    return {};
  }

  cras::expected<void, std::string> processCameraInfo(const sensor_msgs::CameraInfo& cameraInfoMsg) override
  {
    this->lastCameraInfoMsg = cameraInfoMsg;
    return {};
  }

  cras::expected<void, std::string> processImage(const sensor_msgs::ImageConstPtr& image,
                                                 const cras::optional<sensor_msgs::CameraInfo>& cameraInfoMsg) override
  {
    this->lastImageMsg = image;

    const auto& last = this->lastCameraInfoMsg;
    if (cameraInfoMsg.has_value() || (last.has_value() && image->header.stamp == last->header.stamp))
    {
      const auto cameraInfo = boost::make_shared<sensor_msgs::CameraInfo>(
        cameraInfoMsg.has_value() ? *cameraInfoMsg : *last);
      this->cameraPub.publish(image, cameraInfo);
    }
    else
    {
      this->imagePub.publish(image);
    }

    return {};
  }

  cras::expected<void, std::string> processAzimuth(const compass_msgs::Azimuth& azimuthMsg) override
  {
    this->azimuthPub.publish(azimuthMsg);
    return {};
  }

  cras::expected<void, std::string> processMagneticField(const sensor_msgs::MagneticField& magneticFieldMsg) override
  {
    this->magPub.publish(magneticFieldMsg);
    return {};
  }

  cras::expected<void, std::string> processNavSatFix(const sensor_msgs::NavSatFix& navSatFixMsg) override
  {
    this->navMsgPub.publish(navSatFixMsg);
    return {};
  }

  cras::expected<void, std::string> processGps(const gps_common::GPSFix& gpsMsg) override
  {
    this->gpsPub.publish(gpsMsg);
    return {};
  }

  cras::expected<void, std::string> processImu(const sensor_msgs::Imu& imuMsg) override
  {
    this->imuPub.publish(imuMsg);
    return {};
  }

  cras::expected<void, std::string> processZeroRollPitchTf(
    const geometry_msgs::TransformStamped& zeroRollPitchTfMsg) override
  {
    this->tfBroadcaster.sendTransform(zeroRollPitchTfMsg);
    return {};
  }

  cras::expected<void, std::string> processOpticalTf(const geometry_msgs::TransformStamped& opticalTfMsg) override
  {
    this->staticTfBroadcaster.sendTransform(opticalTfMsg);
    return {};
  }

  cras::expected<void, std::string> processFaces(const vision_msgs::Detection2DArray& facesMsg) override
  {
    this->facesPub.publish(facesMsg);
    return {};
  }

  cras::BoundParamHelperPtr params;
  ros::NodeHandle topicsNh;
  cras::optional<sensor_msgs::CameraInfo> lastCameraInfoMsg;
  sensor_msgs::ImageConstPtr lastImageMsg;

  std::unique_ptr<image_transport::ImageTransport> imageTransport;  //!< Image transport instance.
  image_transport::Publisher imagePub;  //!< Image publisher (no camera info).
  image_transport::CameraPublisher cameraPub;  //!< Camera publisher (with camera info).
  ros::Publisher azimuthPub;  //!< Azimuth publisher.
  ros::Publisher navMsgPub;  //!< Fix publisher.
  ros::Publisher gpsPub;  //!< Detailed fix publisher.
  ros::Publisher imuPub;  //!< IMU publisher.
  ros::Publisher magPub;  //!< Magnetic field publisher.
  tf2_ros::TransformBroadcaster tfBroadcaster;  //!< Dynamic TF broadcaster.
  tf2_ros::StaticTransformBroadcaster staticTfBroadcaster;  //!< Static TF broadcaster.
  ros::Publisher facesPub;  //!< Publisher of detected faces.
};

/**
 * \brief Publisher of movie files to ROS image topics.
 *
 * \par Published topics
 *
 * - `movie` (`sensor_msgs/Image`): The published movie. Subtopics from image\_transport are also provided.
 * - `movie/camera_info` (`sensor_msgs/CameraInfo`): Camera info.
 * - `movie/azimuth` (`compass_msgs/Azimuth`): Georeferenced heading of the camera.
 * - `movie/faces` (`vision_msgs/Detection2DArray`): Faces detected in the image.
 * - `movie/fix` (`sensor_msgs/NavSatFix`): GNSS position of the camera.
 * - `movie/fix_detail` (`gps_common/GPSFix`): GNSS position of the camera.
 * - `movie/imu/data` (`sensor_msgs/Imu`): Orientation, angular velocity and acceleration of the camera.
 * - `movie/imu/mag` (`sensor_msgs/MagneticField`): Magnetic field strength.
 *
 * To extract the additional topics except `movie`, the node uses instances of MetadataExtractor.
 *
 * \par Parameters
 *
 * Parameters `~start`, `~end` and `~duration` can be expressed in seconds `(15.35)`, in `(min, sec)`,
 * in `(hour, min, sec)`, or as a string: `'01:03:05.35'`.
 * *
 * - `~movie_file` (string, required): Path to the movie to play. Any format that ffmpeg can decode.
 * - `~fps` (float, optional): If set, the playback will be at the specified FPS (speeding up/slowing down the movie).
 * - `~start` (float|tuple|string, optional): If set, the movie will be read from the specified time.
 *                                            Cannot be set together with `~end` and `~duration`.
 * - `~end` (float|tuple|string, optional): If set, the movie will be read up to the specified time (not affected by
 *                                          start). Cannot be set together with `~start` and `~duration`.
 * - `~duration` (float|tuple|string, optional): If set, playback will have this duration. If end is also set, the
 *                                               duration is counted from the end of the clip, otherwise, it is the
 *                                               duration from the start of the clip. Cannot be set together with
 *                                               `~start` and `~end`.
 * - `~loop` (bool, default False): Whether to loop the movie until the node is shut down. Excludes `~immediate`.
 * - `~immediate` (bool, default False): If True, the movie will be processed and published as quickly as possible not
 *                                       waiting for the real time. The timestamps in the resulting messages act
 *                                       "real-world-like" (i.e. 15 FPS means the frames' timestamps will be 1/15 sec
 *                                       apart). You can set `~fake_time_start` if you want these timestamps to begin
 *                                       from a non-zero time. Excludes `~loop`.
 * - `~playback_rate` (float, optional): If set to a number, immediate mode will not play as fast as possible, but at
 *                                       this rate (set the rate to a number where you do not lose any messages, e.g. in
 *                                       image_transport/republish).
 * - `~fake_time_start` (float, default 0.0): Used with `~immediate` to specify the timestamp of the first message.
 * - `~timestamp_offset` (int|float|string, default `~fake_time_start`): Adjustment of timestamps determined by
 *                                                        `~timestamp_source`.
 *                                                        If given as string, it can be a simple mathematical expression
 *                                                        that can also resolve several variables:
 *                                                        `ros_time` (current ROS time),
 *                                                        `wall_time` (current wall time),
 *                                                        `metadata_start` (start time from metadata).
 * - `~timestamp_source` (str, default `metadata`): How to determine timestamps of the movie frames. Options are:
 *   - `metadata`: Extract absolute time when the movie was recorded and use that time as timestamps.
 *   - `all_zeros`: Use zero timestamps.
 *   - `absolute_timecode`: Use the absolute timecode as timestamps (i.e. time since start of movie file).
 *   - `relative_timecode`: Use the relative timecode as timestamps (i.e. time since `~start`).
 *   - `ros_time`: Timestamp the frames with current ROS time.
 * - `~frame_id` (string, default ""): The frame_id used in the geometrical messages' headers.
 * - `~optical_frame_id` (string, default `${frame_id}_optical_frame`): The frame_id used in the image messages' headers.
 * - `~spin_after_end` (bool, default False): If True, a rospy.spin() is called after the movie has been published.
 * - `~verbose` (bool, default False): If True, logs info about every frame played.
 * - `~allow_yuv_fallback` (bool, default False): Set whether `YUV***` formats should be decoded to YUV422, or whether
 *                                                the default encoding should be used.
 * - `~default_encoding` (string, optional): Set the default encoding which should be used for output frames if there is
 *                                           no direct match between the libav pixel format and ROS image encodings.
 * - `~encoding` (string, optional): Set the encoding which should be used for output frames regardless of their source
 *                                   encoding (one of sensor_msgs::image_encodings constants).
 * - `~wait_after_publisher_created` (float, default 1.0): A workaround for the case where you need to give your
 *                                                         subscribers some time after the publisher was created. Tweak
 *                                                         this number until you get no missing start messages.
 * - `~publisher_queue_size` (int, default 1000 in immediate mode, 10 otherwise): `queue_size` of the movie publisher.
 */
class MoviePublisherNodelet : public cras::Nodelet
{
public:
  ~MoviePublisherNodelet() override
  {
    if (this->playThread.joinable())
      this->playThread.join();
  }

  void onInit() override
  {
    cras::Nodelet::onInit();

    this->spinAfterEnd = this->privateParams()->getParam("spin_after_end", false);
    this->verbose = this->privateParams()->getParam("verbose", false);
    this->loop = this->privateParams()->getParam("loop", false);

    // Legacy parameter
    const auto fakeStart = this->privateParams()->getParam("fake_time_start", ros::Duration{});
    if (!fakeStart.isZero() && this->privateParams()->hasParam("timestamp_offset"))
      ros::param::set("timestamp_offset", fakeStart.toSec());

    const auto filename = this->privateParams()->getParam<std::string>("movie_file", cras::nullopt);

    this->movieReader = std::make_unique<MovieReaderRos>(this->log, this->privateParams());
    this->movieMetadataProcessor = std::make_shared<NodeletMetadataProcessor>(
      this->log, this->privateParams(), this->getNodeHandle());
    auto maybeConfig = this->movieReader->createDefaultConfig();
    if (!maybeConfig.has_value())
    {
      CRAS_FATAL("Failed to create movie configuration object: %s Movie publisher will do nothing.",
        maybeConfig.error().c_str());
      this->requestStop();
      return;
    }

    auto config = *maybeConfig;
    config.metadataProcessors().push_back(this->movieMetadataProcessor);

    const auto maybeMovie = this->movieReader->open(filename, config);
    if (!maybeMovie.has_value())
    {
      CRAS_FATAL("Failed to open movie file '%s' due to the following error: %s Movie publisher will do nothing.",
        filename.c_str(), maybeMovie.error().c_str());
      this->requestStop();
      return;
    }
    this->movie = *maybeMovie;

    const auto immediateMode = this->privateParams()->getParam("immediate", false);
    if (immediateMode)
    {
      if (this->privateParams()->hasParam("playback_rate"))
        this->playbackRate = this->privateParams()->getParam<ros::Rate>("playback_rate", cras::nullopt, "FPS");
    }
    else
    {
      this->playbackRate = this->privateParams()->getParam("fps", ros::Rate(this->movie->info()->frameRate()), "FPS");
    }

    if (immediateMode && this->loop)
    {
      CRAS_FATAL("Cannot set both ~immediate and ~loop");
      this->requestStop();
      return;
    }

    this->playThread = std::thread([this] { this->play(); });
  }

  void play()
  {
    cras::setThreadName("play_thread");

    do
    {
      if (!this->movie->info()->isStillImage())
      {
        if (this->verbose)
          CRAS_INFO("Seeking to %s", cras::to_string(this->movie->info()->subclipStart()).c_str());
        const auto seekResult = this->movie->seekInSubclip({0, 0});
        if (!seekResult.has_value())
        {
          CRAS_ERROR("Error seeking to position %s. Stopping publishing.",
            cras::to_string(this->movie->info()->subclipStart()).c_str());
          this->movieMetadataProcessor->shutdown();
          if (!this->spinAfterEnd)
            this->requestStop();
          return;
        }
      }

      while (this->ok())
      {
        const auto maybePtsAndImg = this->movie->nextFrame();
        if (!maybePtsAndImg.has_value() || maybePtsAndImg->second == nullptr)
        {
          if (!maybePtsAndImg.has_value())
            CRAS_ERROR("Reading the movie has failed with the following error: %s Stopping publishing.",
              maybePtsAndImg.error().c_str());
          else if (this->loop)
          {
            break;
          }
          else
            CRAS_INFO("Movie has ended, stopping publishing.");

          this->movieMetadataProcessor->shutdown();
          if (!this->spinAfterEnd)
            this->requestStop();
          return;
        }

        const auto playbackState = maybePtsAndImg->first;
        const auto& subclipEnd = this->movie->info()->subclipEnd();
        if (!subclipEnd.isZero() && playbackState.streamTime() > subclipEnd)
          break;

        do  // Efficiently loop over still images without reloading them
        {
          if (this->playbackRate.has_value())
            this->playbackRate->sleep();
          if (this->loop && this->movie->info()->isStillImage())
            this->movieMetadataProcessor->repeatLastFrame();
        } while (this->loop && this->movie->info()->isStillImage() && ros::ok() && this->ok());
      }
    } while (this->loop && ros::ok() && this->ok());

    CRAS_INFO("Movie has ended, stopping publishing.");

    this->movieMetadataProcessor->shutdown();
    if (!this->spinAfterEnd)
      this->requestStop();
  }

private:
  std::unique_ptr<MovieReaderRos> movieReader;
  std::shared_ptr<NodeletMetadataProcessor> movieMetadataProcessor;
  MoviePtr movie;

  bool spinAfterEnd {false};  //!< Whether to keep spinning ROS after the movie has ended.
  bool loop {false};  //!< Whether to loop playback when reaching the end of movie.
  cras::optional<ros::Rate> playbackRate;  //!< Rate of playback.
  bool verbose {false};  //!< Verbose console printing.

  std::thread playThread;  //!< Thread that is responsible for timing and publishing the images.
};

}

PLUGINLIB_EXPORT_CLASS(movie_publisher::MoviePublisherNodelet, nodelet::Nodelet)
