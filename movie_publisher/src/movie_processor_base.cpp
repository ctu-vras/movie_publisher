// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Base for nodes that process movie files.
 * \author Martin Pecka
 */

#include <memory>
#include <string>

#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/param_utils.hpp>
#include <gps_common/GPSFix.h>
#include <movie_publisher/movie_processor_base.h>
#include <movie_publisher/movie_reader_ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_msgs/TFMessage.h>

namespace movie_publisher
{

MovieProcessorBase::MovieProcessorBase(const cras::LogHelperPtr& log) : cras::HasLogger(log)
{
}

MovieProcessorBase::~MovieProcessorBase() = default;

std::unique_ptr<MovieReaderRos> MovieProcessorBase::createReader(const cras::BoundParamHelperPtr& params)
{
  return std::make_unique<MovieReaderRos>(this->log, params);
}

cras::expected<void, std::string> MovieProcessorBase::open(
  const std::string& movieFilename, const cras::BoundParamHelperPtr& params)
{
  this->verbose = params->getParam("verbose", false);

  this->reader = this->createReader(params);

  const auto openResult = this->reader->open(movieFilename);
  if (!openResult.has_value())
  {
    return cras::make_unexpected(cras::format("Failed to open movie file '%s' due to the following error: %s",
      movieFilename.c_str(), openResult.error().c_str()));
  }

  return {};
}

cras::expected<void, std::string> MovieProcessorBase::processImage(
  const sensor_msgs::ImageConstPtr& image, const cras::optional<sensor_msgs::CameraInfo>& cameraInfoMsg)
{
  return {};
}

void MovieProcessorBase::processAzimuth(const compass_msgs::Azimuth& azimuthMsg)
{
}

void MovieProcessorBase::processNavSatFix(const sensor_msgs::NavSatFix& navSatFixMsg)
{
}

void MovieProcessorBase::processGps(const gps_common::GPSFix& gpsMsg)
{
}

void MovieProcessorBase::processImu(const sensor_msgs::Imu& imuMsg)
{
}

void MovieProcessorBase::processZeroRollPitchTf(const geometry_msgs::TransformStamped& zeroRollPitchTfMsg)
{
}

void MovieProcessorBase::processOpticalTf(const geometry_msgs::TransformStamped& opticalTfMsg)
{
}

cras::expected<void, std::string> MovieProcessorBase::processFrame(
  const sensor_msgs::ImageConstPtr& image, const ros::Time& pts)
{
  if (this->verbose)
  {
    const auto timecode = pts - ros::Duration().fromNSec(this->reader->getStart().toNSec());
    const auto numFrames = this->reader->getNumFrames();

    CRAS_INFO("Frame %zu/%zu, time %s, timecode %s, stamp %s",
      this->frameNum, numFrames, cras::to_string(pts).c_str(), cras::to_string(timecode).c_str(),
      cras::to_string(image->header.stamp).c_str());
  }

  this->frameNum++;

  const auto& cameraInfoMsg = this->reader->getCameraInfoMsg();
  const auto processImageResult = this->processImage(image, cameraInfoMsg);
  if (!processImageResult.has_value())
    return cras::make_unexpected(processImageResult.error());

  const auto& azimuthMsg = this->reader->getAzimuthMsg();
  if (azimuthMsg.has_value())
    this->processAzimuth(*azimuthMsg);

  const auto& navSatFixMsg = this->reader->getNavSatFixMsg();
  if (navSatFixMsg.has_value())
    this->processNavSatFix(*navSatFixMsg);

  const auto& gpsMsg = this->reader->getGpsMsg();
  if (gpsMsg.has_value())
    this->processGps(*gpsMsg);

  const auto& imuMsg = this->reader->getImuMsg();
  if (imuMsg.has_value())
    this->processImu(*imuMsg);

  const auto& zeroRollPitchTfMsg = this->reader->getZeroRollPitchTF();
  if (zeroRollPitchTfMsg.has_value())
    this->processZeroRollPitchTf(*zeroRollPitchTfMsg);

  const auto& opticalTfMsg = this->reader->getOpticalFrameTF();
  if (opticalTfMsg.has_value() && !this->reader->getFrameId().empty() && !this->reader->getOpticalFrameId().empty() &&
    opticalTfMsg->transform != this->lastOpticalTf)
  {
    this->lastOpticalTf = opticalTfMsg->transform;
    this->processOpticalTf(*opticalTfMsg);
  }

  return {};
}

}
