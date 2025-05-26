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
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_msgs/TFMessage.h>
#include <vision_msgs/Detection2DArray.h>

namespace movie_publisher
{

MovieToBag::MovieToBag(const cras::LogHelperPtr& log) : cras::NodeWithOptionalMaster(log)
{
}

std::unique_ptr<MovieReaderRos> MovieToBag::createReader(const cras::BoundParamHelperPtr& params)
{
  return std::make_unique<MovieReaderRos>(this->log, params);
}

cras::expected<void, std::string> MovieToBag::open(
  const std::string& bagFilename, const std::string& transport, const std::string& movieFilename,
  const cras::BoundParamHelperPtr& params)
{
  if (this->movieReader == nullptr)
    this->movieReader = std::move(this->createReader(params));

  this->metadataProcessor = this->createMetadataProcessor(bagFilename, transport, params);
  this->metadataProcessor->addTimestampOffsetVars(*this->movieReader);
  auto maybeConfig = this->movieReader->createDefaultConfig();
  if (!maybeConfig.has_value())
    return cras::make_unexpected(maybeConfig.error());

  auto config = *maybeConfig;
  config.metadataProcessors().push_back(this->metadataProcessor);

  const auto maybeMovie = this->movieReader->open(movieFilename, config);
  if (!maybeMovie.has_value())
  {
    return cras::make_unexpected(cras::format("Failed to open movie file '%s' due to the following error: %s",
      movieFilename.c_str(), maybeMovie.error().c_str()));
  }
  this->movie = *maybeMovie;

  return {};
}

cras::expected<void, std::string> MovieToBag::run()
{
  while (this->ok())
  {
    const auto maybePtsAndImg = this->movie->nextFrame();
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

    const auto playbackState = std::get<0>(*maybePtsAndImg);
    const auto& subclipEnd = this->movie->info()->subclipEnd();
    if (!subclipEnd.isZero() && playbackState.streamTime() > subclipEnd)
      break;
  }

  CRAS_INFO("Reached end of movie.");
  this->metadataProcessor->close();
  this->movie.reset();
  this->metadataProcessor.reset();

  return {};
}

std::string MovieToBagMetadataProcessor::getImageTopic() const
{
  return this->resolveName(this->transport == "raw" ? this->topic : (this->topic + "/" + this->transport));
}

std::string MovieToBagMetadataProcessor::getCameraInfoTopic() const
{
  return this->getPrefixedTopic("camera_info");
}

std::string MovieToBagMetadataProcessor::getAzimuthTopic() const
{
  return this->getPrefixedTopic("azimuth");
}

std::string MovieToBagMetadataProcessor::getMagneticFieldTopic() const
{
  return this->getPrefixedTopic("imu/mag");
}

std::string MovieToBagMetadataProcessor::getNavSatFixTopic() const
{
  return this->getPrefixedTopic("fix");
}

std::string MovieToBagMetadataProcessor::getGpsTopic() const
{
  return this->getPrefixedTopic("fix_detail");
}

std::string MovieToBagMetadataProcessor::getImuTopic() const
{
  return this->getPrefixedTopic("imu");
}

std::string MovieToBagMetadataProcessor::getTfTopic() const
{
  return this->resolveName("/tf");
}

std::string MovieToBagMetadataProcessor::getStaticTfTopic() const
{
  return this->resolveName("/tf_static");
}

std::string MovieToBagMetadataProcessor::getFacesTopic() const
{
  return this->getPrefixedTopic("faces");
}

std::string MovieToBagMetadataProcessor::getPrefixedTopic(const std::string& topicName) const
{
  return this->resolveName(ros::names::append(this->topic, topicName));
}

std::shared_ptr<MovieToBagMetadataProcessor> MovieToBag::createMetadataProcessor(const std::string& bagFilename,
  const std::string& transport, const cras::BoundParamHelperPtr& params)
{
  return std::make_shared<MovieToBagMetadataProcessor>(this->log, bagFilename, transport,
    [this](const std::string& name) {return this->resolveName(name);}, params);
}

MovieToBagMetadataProcessor::MovieToBagMetadataProcessor(
  const cras::LogHelperPtr& log, const std::string& bagFilename, const std::string& transport,
  const std::function<std::string(const std::string&)>& resolveName, const cras::BoundParamHelperPtr& params)
  : cras::HasLogger(log), transport(transport), resolveName(resolveName)
{
  this->imageCodecs = std::make_unique<image_transport_codecs::ImageTransportCodecs>(log);
  this->topic = params->getParam("topic", "movie");

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
}

MovieToBagMetadataProcessor::~MovieToBagMetadataProcessor()
{
  this->close();
}

void MovieToBagMetadataProcessor::close()
{
  if (this->bag != nullptr)
  {
    CRAS_INFO("Closing bag file.");
    this->bag->close();
    this->bag.reset();
    ros::WallDuration(2.0).sleep();
  }
}

void MovieToBagMetadataProcessor::addTimestampOffsetVars(MovieReaderRos& reader) const
{
  const auto bagView = std::make_shared<rosbag::View>(*this->bag);
  if (bagView->size() > 0)
  {
    reader.addTimestampOffsetVar("bag_start", bagView->getBeginTime().toSec());
    reader.addTimestampOffsetVar("bag_end", bagView->getEndTime().toSec());
    reader.addTimestampOffsetVar("bag_duration", (bagView->getEndTime() - bagView->getBeginTime()).toSec());
  }
}

cras::expected<void, std::string> MovieToBagMetadataProcessor::processImage(
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

cras::expected<void, std::string> MovieToBagMetadataProcessor::processAzimuth(const compass_msgs::Azimuth& azimuthMsg)
{
  try
  {
    this->bag->write(this->getAzimuthTopic(), azimuthMsg.header.stamp, azimuthMsg);
  }
  catch (const rosbag::BagIOException& e)
  {
    return cras::make_unexpected(cras::format("Failed to save azimuth info into bagfile: %s", e.what()));
  }
  return {};
}

cras::expected<void, std::string> MovieToBagMetadataProcessor::processMagneticField(
  const sensor_msgs::MagneticField& magneticFieldMsg)
{
  try
  {
    this->bag->write(this->getMagneticFieldTopic(), magneticFieldMsg.header.stamp, magneticFieldMsg);
  }
  catch (const rosbag::BagIOException& e)
  {
    return cras::make_unexpected(cras::format("Failed to save magnetic field info into bagfile: %s", e.what()));
  }
  return {};
}

cras::expected<void, std::string> MovieToBagMetadataProcessor::processNavSatFix(
  const sensor_msgs::NavSatFix& navSatFixMsg)
{
  try
  {
    this->bag->write(this->getNavSatFixTopic(), navSatFixMsg.header.stamp, navSatFixMsg);
  }
  catch (const rosbag::BagIOException& e)
  {
    return cras::make_unexpected(cras::format("Failed to save fix into bagfile: %s", e.what()));
  }
  return {};
}

cras::expected<void, std::string> MovieToBagMetadataProcessor::processGps(const gps_common::GPSFix& gpsMsg)
{
  try
  {
    this->bag->write(this->getGpsTopic(), gpsMsg.header.stamp, gpsMsg);
  }
  catch (const rosbag::BagIOException& e)
  {
    return cras::make_unexpected(cras::format("Failed to save fix details into bagfile: %s", e.what()));
  }
  return {};
}

cras::expected<void, std::string> MovieToBagMetadataProcessor::processImu(const sensor_msgs::Imu& imuMsg)
{
  try
  {
    this->bag->write(this->getImuTopic(), imuMsg.header.stamp, imuMsg);
  }
  catch (const rosbag::BagIOException& e)
  {
    return cras::make_unexpected(cras::format("Failed to save IMU into bagfile: %s", e.what()));
  }
  return {};
}

cras::expected<void, std::string> MovieToBagMetadataProcessor::processZeroRollPitchTf(
  const geometry_msgs::TransformStamped& zeroRollPitchTfMsg)
{
  tf2_msgs::TFMessage msg;
  msg.transforms.push_back(zeroRollPitchTfMsg);
  try
  {
    this->bag->write(this->getTfTopic(), zeroRollPitchTfMsg.header.stamp, msg);
  }
  catch (const rosbag::BagIOException& e)
  {
    return cras::make_unexpected(cras::format("Failed to save TF into bagfile: %s", e.what()));
  }
  return {};
}

cras::expected<void, std::string> MovieToBagMetadataProcessor::processOpticalTf(
  const geometry_msgs::TransformStamped& opticalTfMsg)
{
  auto zeroStampCopy = opticalTfMsg;
  zeroStampCopy.header.stamp = {};

  if (this->lastOpticalTF.has_value() && *this->lastOpticalTF == zeroStampCopy)
    return {};
  this->lastOpticalTF = zeroStampCopy;

  tf2_msgs::TFMessage msg;
  msg.transforms.push_back(opticalTfMsg);
  try
  {
    auto connectionHeader = boost::make_shared<ros::M_string>();
    (*connectionHeader)["latching"] = "1";
    (*connectionHeader)["type"] = ros::message_traits::datatype<decltype(msg)>();
    (*connectionHeader)["md5sum"] = ros::message_traits::md5sum<decltype(msg)>();
    (*connectionHeader)["message_definition"] = ros::message_traits::definition<decltype(msg)>();
    this->bag->write(this->getStaticTfTopic(), opticalTfMsg.header.stamp, msg, connectionHeader);
  }
  catch (const rosbag::BagIOException& e)
  {
    return cras::make_unexpected(cras::format("Failed to save static TF into bagfile: %s", e.what()));
  }
  return {};
}

cras::expected<void, std::string> MovieToBagMetadataProcessor::processFaces(
  const vision_msgs::Detection2DArray& facesMsg)
{
  try
  {
    this->bag->write(this->getFacesTopic(), facesMsg.header.stamp, facesMsg);
  }
  catch (const rosbag::BagIOException& e)
  {
    return cras::make_unexpected(cras::format("Failed to save detected faces info into bagfile: %s", e.what()));
  }
  return {};
}
}
