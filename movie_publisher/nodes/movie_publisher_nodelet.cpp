// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief
 * \author Martin Pecka
 */

#include <memory>
#include <regex>
#include <string>
#include <thread>

#include <movie_publisher/movie_reader.h>

#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/functional.hpp>
#include <cras_cpp_common/nodelet_utils.hpp>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/param_utils.hpp>
#include <gps_common/GPSFix.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace movie_publisher
{

std::regex timeRegex("(\\d+):(\\d+):(\\d+([,|.]\\d+)?)");

template<typename T>
T parseTimeParam(const XmlRpc::XmlRpcValue& param)
{
  if (param.getType() == XmlRpc::XmlRpcValue::TypeString)
  {
    auto time = static_cast<std::string>(param);
    if (!cras::contains(time, ',') && !cras::contains(time, '.'))
      time = time + ".0";
    std::smatch matches;
    if (!std::regex_match(time, matches, timeRegex))
      throw std::runtime_error(cras::format("Could not parse value '%s' as a time string.", time.c_str()));

    const int64_t hr = std::atoll(matches[1].str().c_str());
    const int32_t mn = std::atol(matches[2].str().c_str());
    const auto secondsStr = cras::replace(matches[3].str(), ",", ".");
    const double sec = cras::parseDouble(secondsStr);
    return T().fromSec(3600.0 * hr + 60.0 * mn + sec);
  }
  else if (param.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    if (param.size() != 2 && param.size() != 3)
      throw std::runtime_error("Time parameter has to have 2 or 3 values when passed as tuple.");
    std::vector<double> values;
    std::list<std::string> errors;
    if (!cras::convert(param, values, false, &errors))
      throw std::runtime_error(cras::format("Wrong values for time parameter: %s", cras::to_string(errors).c_str()));
    double hr = 0, mn = 0, sec = 0;
    if (values.size() == 3)
    {
      hr = values[0];
      mn = values[1];
      sec = values[2];
    }
    else
    {
      mn = values[0];
      sec = values[1];
    }
    return T().fromSec(3600.0 * hr + 60.0 * mn + sec);
  }
  else
  {
    double value;
    std::list<std::string> errors;
    if (!cras::convert(param, value, false, &errors))
      throw std::runtime_error(cras::format("Wrong value for time parameter: %s", cras::to_string(errors).c_str()));
    return T().fromSec(value);
  }
}

MovieReader::TimestampSource parseTimestampSource(const std::string& param)
{
  const auto& p = cras::toLower(param);
  if (p == "all_zeros")
    return MovieReader::TimestampSource::AllZeros;
  else if (p == "absolute_timecode")
    return MovieReader::TimestampSource::AbsoluteVideoTimecode;
  else if (p == "relative_timecode")
    return MovieReader::TimestampSource::RelativeVideoTimecode;
  else if (p == "ros_time")
    return MovieReader::TimestampSource::RosTime;
  else if (p == "metadata")
    return MovieReader::TimestampSource::FromMetadata;
  throw std::runtime_error(cras::format("Value %s is not a valid timestamp_source value.", param.c_str()));
}

std::string timestampSourceToStr(const MovieReader::TimestampSource& source)
{
  switch (source)
  {
    case MovieReader::TimestampSource::AllZeros:
      return "all_zeros";
    case MovieReader::TimestampSource::AbsoluteVideoTimecode:
      return "absolute_timecode";
    case MovieReader::TimestampSource::RelativeVideoTimecode:
      return "relative_timecode";
    case MovieReader::TimestampSource::RosTime:
      return "ros_time";
    case MovieReader::TimestampSource::FromMetadata:
      return "metadata";
    default:
      throw std::runtime_error("Wrong value");
  }
}

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
    this->reader = std::make_unique<MovieReader>(this->getLogger(), this->privateParams());

    this->reader->forceEncoding(sensor_msgs::image_encodings::BGR8);

    const auto filename = this->privateParams()->getParam<std::string>("movie_file", cras::nullopt);
    const auto numThreads = this->privateParams()->getParam("num_decoder_threads", 1_sz);
    this->reader->setNumThreads(numThreads);

    const auto openResult = this->reader->open(filename, MovieReader::TimestampSource::FromMetadata);
    if (!openResult.has_value())
    {
      CRAS_FATAL("Failed to open movie file '%s' due to the following error: %s Movie publisher will do nothing.",
        filename.c_str(), openResult.error().c_str());
      this->requestStop();
      return;
    }

    this->isStillImage = this->reader->isStillImage();

    cras::optional<ros::Time> start;
    if (this->privateParams()->hasParam("start"))
      start = this->privateParams()->getParamVerbose<ros::Time>("start", cras::nullopt, "s", {.toResult = &parseTimeParam<ros::Time>});

    cras::optional<ros::Time> end;
    if (this->privateParams()->hasParam("end"))
      end = this->privateParams()->getParamVerbose<ros::Time>("end", cras::nullopt, "s", {.toResult = &parseTimeParam<ros::Time>});

    cras::optional<ros::Duration> duration;
    if (this->privateParams()->hasParam("duration"))
      duration = this->privateParams()->getParamVerbose<ros::Duration>("duration", cras::nullopt, "s", {.toResult = &parseTimeParam<ros::Duration>});

    if (start.has_value() && end.has_value() && duration.has_value())
    {
      CRAS_FATAL("At least one of ~start, ~end and ~duration parameters must remain unset.");
      this->requestStop();
      return;
    }

    if (
      (end && start && *end <= *start) ||
      (duration && *duration <= ros::Duration{}) ||
      (duration && *duration > this->reader->getDuration()) ||
      (start && duration && (*start + *duration) > (ros::Time{} + this->reader->getDuration())) ||
      (end && duration && (*end < (ros::Time{} + *duration))))
    {
      CRAS_FATAL("The provided combination of ~start, ~end and ~duration is not consistent with the movie file.");
      this->requestStop();
      return;
    }

    this->start = {0, 0};
    this->end = {0, 0};

    if (start)
      this->start = *start;
    if (end)
      this->end = *end;

    if (duration)
    {
      if (start)
        this->end = *start + *duration;
      else if (end)
        this->start = *end - *duration;
      else
        this->end = this->start + *duration;
    }

    const auto immediateMode = this->privateParams()->getParam("immediate", false);
    if (immediateMode)
    {
      if (this->privateParams()->hasParam("playback_rate"))
        this->playbackRate = this->privateParams()->getParam<ros::Rate>("playback_rate", cras::nullopt, "FPS");
    }
    else
    {
      this->playbackRate = this->privateParams()->getParam("fps", ros::Rate(this->reader->getFrameRate()), "FPS");
    }

    this->frameId = this->privateParams()->getParam("frame_id", "");
    this->opticalFrameId = this->privateParams()->getParam(
      "optical_frame_id", this->frameId.empty() ? "" : this->frameId + "_optical_frame");
    this->reader->setFrameId(this->frameId, this->opticalFrameId);
    this->spinAfterEnd = this->privateParams()->getParam("spin_after_end", false);
    this->verbose = this->privateParams()->getParam("verbose", false);
    this->loop = this->privateParams()->getParam("loop", false);

    this->reader->setTimestampOffset(this->privateParams()->getParam("fake_time_start", ros::Duration{}));
    const auto& timestampSource = this->privateParams()->getParam<MovieReader::TimestampSource, std::string>(
      "timestamp_source", MovieReader::TimestampSource::FromMetadata, "",
      {.resultToStr = &timestampSourceToStr, .toResult = &parseTimestampSource});
    this->reader->setTimestampSource(timestampSource);


    const auto pubQueueSize =
      this->privateParams()->getParam("publisher_queue_size", immediateMode ? 1000 : 10, "messages");
    const auto waitAfterPublisherCreated =
      this->privateParams()->getParam("wait_after_publisher_created", ros::WallDuration(1));

    if (immediateMode && this->loop)
    {
      CRAS_FATAL("Cannot set both ~immediate and ~loop");
      this->requestStop();
      return;
    }

    this->imageTransport = std::make_unique<image_transport::ImageTransport>(this->getNodeHandle());
    if (this->reader->getCameraInfoMsg().has_value())
      this->cameraPub = this->imageTransport->advertiseCamera("movie", pubQueueSize);
    else
      this->imagePub = this->imageTransport->advertise("movie", pubQueueSize);

    ros::NodeHandle topicsNh(this->getNodeHandle(), "movie");
    if (this->reader->getAzimuthMsg().has_value())
      this->azimuthPub = topicsNh.advertise<compass_msgs::Azimuth>("azimuth", pubQueueSize);
    if (this->reader->getNavSatFixMsg().has_value())
      this->navMsgPub = topicsNh.advertise<sensor_msgs::NavSatFix>("fix", pubQueueSize);
    if (this->reader->getGpsMsg().has_value())
      this->gpsPub = topicsNh.advertise<gps_common::GPSFix>("fix_detail", pubQueueSize);
    if (this->reader->getImuMsg().has_value())
      this->imuPub = topicsNh.advertise<sensor_msgs::Imu>("imu", pubQueueSize);

    waitAfterPublisherCreated.sleep();

    this->playThread = std::thread([this] { this->play(); });
  }

  void play()
  {
    do
    {
      if (!this->isStillImage)
      {
        if (this->verbose)
          CRAS_INFO("Seeking to %s", cras::to_string(this->start).c_str());
        const auto seekResult = this->reader->seek(this->start);
        if (!seekResult.has_value())
        {
          CRAS_ERROR("Error seeking to position %s. Stopping publishing.", cras::to_string(this->start).c_str());
          this->imagePub.shutdown();
          if (!this->spinAfterEnd)
            this->requestStop();
          return;
        }
      }

      size_t frameNum {0L};
      while (this->ok())
      {
        const auto maybePtsAndImg = this->reader->nextFrame();
        if (!maybePtsAndImg.has_value() || std::get<1>(*maybePtsAndImg) == nullptr)
        {
          if (!maybePtsAndImg.has_value())
            CRAS_ERROR("Reading the movie has failed with the following error: %s Stopping publishing.", maybePtsAndImg.error().c_str());
          else if (this->loop)
          {
            break;
          }
          else
            CRAS_INFO("Movie has ended, stopping publishing.");

          this->imagePub.shutdown();
          if (!this->spinAfterEnd)
            this->requestStop();
          return;
        }

        const auto pts = std::get<0>(*maybePtsAndImg);
        if (!this->end.isZero() && pts > this->end)
          break;

        const auto img = std::get<1>(*maybePtsAndImg);

        do
        {
          if (this->verbose)
          {
            const auto timecode = pts - ros::Duration().fromNSec(this->start.toNSec());
            const auto numFrames = this->reader->getNumFrames();

            CRAS_INFO("Frame %zu/%zu, time %s, timecode %s, stamp %s",
              frameNum, numFrames, cras::to_string(pts).c_str(), cras::to_string(timecode).c_str(),
              cras::to_string(img->header.stamp).c_str());
          }

          frameNum++;

          const auto& cameraInfoMsg = this->reader->getCameraInfoMsg();
          if (cameraInfoMsg.has_value())
            this->cameraPub.publish(img, boost::make_shared<sensor_msgs::CameraInfo>(*cameraInfoMsg));
          else
            this->imagePub.publish(img);

          const auto& azimuthMsg = this->reader->getAzimuthMsg();
          if (azimuthMsg.has_value())
            this->azimuthPub.publish(*azimuthMsg);

          const auto& navSatFixMsg = this->reader->getNavSatFixMsg();
          if (navSatFixMsg.has_value())
            this->navMsgPub.publish(*navSatFixMsg);

          const auto& gpsMsg = this->reader->getGpsMsg();
          if (gpsMsg.has_value())
            this->gpsPub.publish(*gpsMsg);

          const auto& imuMsg = this->reader->getImuMsg();
          if (imuMsg.has_value())
            this->imuPub.publish(*imuMsg);

          const auto& opticalTfMsg = this->reader->getOpticalFrameTF();
          if (opticalTfMsg.has_value())
          {
            if (opticalTfMsg->transform != this->lastOpticalTf)
              this->staticTfBroadcaster.sendTransform(*opticalTfMsg);
            this->lastOpticalTf = opticalTfMsg->transform;
          }

          const auto& zeroRollPitchTfMsg = this->reader->getZeroRollPitchTF();
          if (zeroRollPitchTfMsg.has_value())
            this->tfBroadcaster.sendTransform(*zeroRollPitchTfMsg);

          if (this->playbackRate.has_value())
            this->playbackRate->sleep();
        } while (this->loop && this->isStillImage && ros::ok() && this->ok());
      }
    } while (this->loop && ros::ok() && this->ok());

    CRAS_INFO("Movie has ended, stopping publishing.");

    this->imagePub.shutdown();
    if (!this->spinAfterEnd)
      this->requestStop();
  }

private:
  std::unique_ptr<MovieReader> reader;
  std::unique_ptr<image_transport::ImageTransport> imageTransport;
  image_transport::Publisher imagePub;
  image_transport::CameraPublisher cameraPub;
  ros::Publisher azimuthPub;
  ros::Publisher navMsgPub;
  ros::Publisher gpsPub;
  ros::Publisher imuPub;
  tf2_ros::TransformBroadcaster tfBroadcaster;
  tf2_ros::StaticTransformBroadcaster staticTfBroadcaster;

  geometry_msgs::Transform lastOpticalTf;

  ros::Time start;
  ros::Time end;

  bool spinAfterEnd {false};
  bool verbose {false};
  bool loop {false};
  std::string frameId;
  std::string opticalFrameId;
  cras::optional<ros::Rate> playbackRate;

  bool isStillImage {false};

  std::thread playThread;
};

}

PLUGINLIB_EXPORT_CLASS(movie_publisher::MoviePublisherNodelet, nodelet::Nodelet)
