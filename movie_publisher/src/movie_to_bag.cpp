// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Convert movie files and their metadata to ROS bag file.
 * \author Martin Pecka
 */

#include <memory>
#include <string>

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
#include <movie_publisher/movie_to_bag.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_msgs/TFMessage.h>

namespace movie_publisher
{

MovieToBag::MovieToBag(const cras::LogHelperPtr& log) : cras::NodeWithOptionalMaster(log), MovieProcessorBase(log)
{
}

std::unique_ptr<MovieReaderRos> MovieToBag::createReader(const cras::BoundParamHelperPtr& params)
{
  auto reader = MovieProcessorBase::createReader(params);

  const auto bagView = std::make_shared<rosbag::View>(*this->bag);
  if (bagView->size() > 0)
  {
    reader->addTimestampOffsetVar("bag_start", bagView->getBeginTime().toSec());
    reader->addTimestampOffsetVar("bag_end", bagView->getEndTime().toSec());
    reader->addTimestampOffsetVar("bag_duration", (bagView->getEndTime() - bagView->getBeginTime()).toSec());
  }

  return reader;
}

cras::expected<void, std::string> MovieToBag::open(
  const std::string& bagFilename, const std::string& transport, const std::string& movieFilename,
  const cras::BoundParamHelperPtr& params)
{
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

  this->topic = params->getParam("topic", "movie");

  const auto openResult = MovieProcessorBase::open(movieFilename, params);
  if (!openResult.has_value())
  {
    return cras::make_unexpected(cras::format("Failed to open movie file '%s' due to the following error: %s",
      movieFilename.c_str(), openResult.error().c_str()));
  }

  this->imageCodecs = std::make_unique<image_transport_codecs::ImageTransportCodecs>(MovieProcessorBase::log);

  return {};
}

cras::expected<void, std::string> MovieToBag::run()
{
  const auto& opticalTfMsg = this->reader->getOpticalFrameTF();
  if (opticalTfMsg.has_value() && !this->reader->getFrameId().empty() && !this->reader->getOpticalFrameId().empty())
  {
    this->lastOpticalTf = opticalTfMsg->transform;
    this->processOpticalTf(*opticalTfMsg);
  }

  while (this->ok())
  {
    const auto maybePtsAndImg = this->reader->nextFrame();
    if (!maybePtsAndImg.has_value() || std::get<1>(*maybePtsAndImg) == nullptr)
    {
      if (!maybePtsAndImg.has_value())
      {
        return cras::make_unexpected(cras::format(
          "Reading the movie has failed with the following error: %s Stopped conversion.",
          maybePtsAndImg.error().c_str()));
      }
      break;
    }

    const auto pts = std::get<0>(*maybePtsAndImg);
    if (!this->reader->getEnd().isZero() && pts > this->reader->getEnd())
      break;

    const auto img = std::get<1>(*maybePtsAndImg);

    const auto processResult = this->processFrame(img, pts);
    if (!processResult.has_value())
    {
      CRAS_DEBUG("Failed processing frame %zu/%zu: %s",
        frameNum, this->reader->getNumFrames(), processResult.error().c_str());
      CRAS_WARN_THROTTLE(1.0, "Failed processing frame %zu/%zu: %s",
        frameNum, this->reader->getNumFrames(), processResult.error().c_str());
    }
  }

  CRAS_INFO("Reached end of movie.");

  return {};
}

cras::LogHelperConstPtr MovieToBag::getCrasLogger() const
{
  return MovieProcessorBase::getCrasLogger();
}

std::string MovieToBag::getImageTopic() const
{
  return this->resolveName(this->transport == "raw" ? this->topic : (this->topic + "/" + this->transport));
}

std::string MovieToBag::getCameraInfoTopic() const
{
  return this->getPrefixedTopic("camera_info");
}

std::string MovieToBag::getAzimuthTopic() const
{
  return this->getPrefixedTopic("azimuth");
}

std::string MovieToBag::getNavSatFixTopic() const
{
  return this->getPrefixedTopic("fix");
}

std::string MovieToBag::getGpsTopic() const
{
  return this->getPrefixedTopic("fix_detail");
}

std::string MovieToBag::getImuTopic() const
{
  return this->getPrefixedTopic("imu");
}

std::string MovieToBag::getTfTopic() const
{
  return this->resolveName("/tf");
}

std::string MovieToBag::getStaticTfTopic() const
{
  return this->resolveName("/tf_static");
}

std::string MovieToBag::getPrefixedTopic(const std::string& topicName) const
{
  return this->resolveName(ros::names::append(this->topic, topicName));
}

cras::expected<void, std::string> MovieToBag::processImage(
  const sensor_msgs::ImageConstPtr& image, const cras::optional<sensor_msgs::CameraInfo>& cameraInfoMsg)
{
  if (image == nullptr)
    return cras::make_unexpected("Image is null.");

  const auto encodedImage = this->imageCodecs->encode(*image, this->transport);
  if (!encodedImage.has_value())
    return cras::make_unexpected(encodedImage.error());

  try
  {
    this->bag->write(this->getImageTopic(), image->header.stamp, *encodedImage);
  }
  catch (const rosbag::BagIOException& e)
  {
    CRAS_ERROR_THROTTLE(1.0, cras::format("Failed to save image into bagfile: %s", e.what()));
  }

  if (cameraInfoMsg.has_value())
  {
    try
    {
      this->bag->write(this->getCameraInfoTopic(), cameraInfoMsg->header.stamp, *cameraInfoMsg);
    }
    catch (const rosbag::BagIOException& e)
    {
      CRAS_ERROR_THROTTLE(1.0, cras::format("Failed to save camera info into bagfile: %s", e.what()));
    }
  }

  return {};
}

void MovieToBag::processAzimuth(const compass_msgs::Azimuth& azimuthMsg)
{
  try
  {
    this->bag->write(this->getAzimuthTopic(), azimuthMsg.header.stamp, azimuthMsg);
  }
  catch (const rosbag::BagIOException& e)
  {
    CRAS_ERROR_THROTTLE(1.0, cras::format("Failed to save azimuth info into bagfile: %s", e.what()));
  }
}

void MovieToBag::processNavSatFix(const sensor_msgs::NavSatFix& navSatFixMsg)
{
  try
  {
    this->bag->write(this->getNavSatFixTopic(), navSatFixMsg.header.stamp, navSatFixMsg);
  }
  catch (const rosbag::BagIOException& e)
  {
    CRAS_ERROR_THROTTLE(1.0, cras::format("Failed to save fix into bagfile: %s", e.what()));
  }
}

void MovieToBag::processGps(const gps_common::GPSFix& gpsMsg)
{
  try
  {
    this->bag->write(this->getGpsTopic(), gpsMsg.header.stamp, gpsMsg);
  }
  catch (const rosbag::BagIOException& e)
  {
    CRAS_ERROR_THROTTLE(1.0, cras::format("Failed to save fix details into bagfile: %s", e.what()));
  }
}

void MovieToBag::processImu(const sensor_msgs::Imu& imuMsg)
{
  try
  {
    this->bag->write(this->getImuTopic(), imuMsg.header.stamp, imuMsg);
  }
  catch (const rosbag::BagIOException& e)
  {
    CRAS_ERROR_THROTTLE(1.0, cras::format("Failed to save IMU into bagfile: %s", e.what()));
  }
}

void MovieToBag::processZeroRollPitchTf(const geometry_msgs::TransformStamped& zeroRollPitchTfMsg)
{
  tf2_msgs::TFMessage msg;
  msg.transforms.push_back(zeroRollPitchTfMsg);
  try
  {
    this->bag->write(this->getTfTopic(), zeroRollPitchTfMsg.header.stamp, msg);
  }
  catch (const rosbag::BagIOException& e)
  {
    CRAS_ERROR_THROTTLE(1.0, cras::format("Failed to save TF into bagfile: %s", e.what()));
  }
}

void MovieToBag::processOpticalTf(const geometry_msgs::TransformStamped& opticalTfMsg)
{
  tf2_msgs::TFMessage msg;
  msg.transforms.push_back(opticalTfMsg);
  try
  {
    this->bag->write(this->getStaticTfTopic(), opticalTfMsg.header.stamp, msg);
  }
  catch (const rosbag::BagIOException& e)
  {
    CRAS_ERROR_THROTTLE(1.0, cras::format("Failed to save static TF into bagfile: %s", e.what()));
  }
}

}
