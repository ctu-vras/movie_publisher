// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief
 * \author Martin Pecka
 */

#pragma once

#include "metadata_extractor.h"

#include <deque>
#include <memory>
#include <string>
#include <utility>

#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/optional.hpp>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <pluginlib/class_loader.hpp>
#include <ros/time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>


namespace movie_publisher
{

class StackGuard;

class MetadataManager : public MetadataExtractor
{
public:
  explicit MetadataManager(const cras::LogHelperPtr& log, size_t width, size_t height);
  ~MetadataManager() override;

  void addExtractor(const std::shared_ptr<MetadataExtractor>& extractor);
  void loadExtractorPlugins(const MetadataExtractorParams& params);

  std::string getName() const override;
  int getPriority() const override;
  cras::optional<std::string> getCameraGeneralName() override;
  cras::optional<std::string> getCameraUniqueName() override;
  cras::optional<std::string> getCameraSerialNumber() override;
  cras::optional<std::string> getCameraMake() override;
  cras::optional<std::string> getCameraModel() override;
  cras::optional<std::string> getLensMake() override;
  cras::optional<std::string> getLensModel() override;
  cras::optional<int> getRotation() override;
  cras::optional<ros::Time> getCreationTime() override;
  cras::optional<double> getCropFactor() override;
  cras::optional<std::pair<double, double>> getSensorSizeMM() override;
  cras::optional<double> getFocalLength35MM() override;
  cras::optional<double> getFocalLengthPx() override;
  cras::optional<double> getFocalLengthMM() override;
  cras::optional<CI::_K_type> getIntrinsicMatrix() override;
  cras::optional<std::pair<CI::_distortion_model_type, CI::_D_type>> getDistortion() override;
  std::pair<cras::optional<sensor_msgs::NavSatFix>, cras::optional<gps_common::GPSFix>> getGNSSPosition() override;
  cras::optional<compass_msgs::Azimuth> getAzimuth() override;
  cras::optional<std::pair<double, double>> getRollPitch() override;
  cras::optional<geometry_msgs::Vector3> getAcceleration() override;

  virtual cras::optional<sensor_msgs::CameraInfo> getCameraInfo();
  virtual cras::optional<sensor_msgs::Imu> getImu();
  virtual cras::optional<geometry_msgs::Quaternion> getRollPitchOrientation();
  virtual cras::optional<geometry_msgs::Transform> getOpticalFrameTF();

protected:
  bool stopRecursion(const std::string& fn, const MetadataExtractor* extractor) const;

  pluginlib::ClassLoader<MetadataExtractorPlugin> loader;
  std::list<std::shared_ptr<MetadataExtractor>> extractors;
  std::deque<std::pair<std::string, const MetadataExtractor*>> callStack;
  size_t width {0u};
  size_t height {0u};

  cras::optional<cras::optional<std::string>> getCameraGeneralNameResult;
  cras::optional<cras::optional<std::string>> getCameraUniqueNameResult;
  cras::optional<cras::optional<std::string>> getCameraSerialNumberResult;
  cras::optional<cras::optional<std::string>> getCameraMakeResult;
  cras::optional<cras::optional<std::string>> getCameraModelResult;
  cras::optional<cras::optional<std::string>> getLensMakeResult;
  cras::optional<cras::optional<std::string>> getLensModelResult;
  cras::optional<cras::optional<int>> getRotationResult;
  cras::optional<cras::optional<ros::Time>> getCreationTimeResult;
  cras::optional<cras::optional<double>> getCropFactorResult;
  cras::optional<cras::optional<std::pair<double, double>>> getSensorSizeMMResult;
  cras::optional<cras::optional<double>> getFocalLength35MMResult;
  cras::optional<cras::optional<double>> getFocalLengthPxResult;
  cras::optional<cras::optional<double>> getFocalLengthMMResult;
  cras::optional<cras::optional<CI::_K_type>> getIntrinsicMatrixResult;
  cras::optional<cras::optional<std::pair<CI::_distortion_model_type, CI::_D_type>>> getDistortionResult;
  cras::optional<std::pair<cras::optional<sensor_msgs::NavSatFix>, cras::optional<gps_common::GPSFix>>>
    getGNSSPositionResult;
  cras::optional<cras::optional<compass_msgs::Azimuth>> getAzimuthResult;
  cras::optional<cras::optional<std::pair<double, double>>> getRollPitchResult;
  cras::optional<cras::optional<geometry_msgs::Vector3>> getAccelerationResult;
  cras::optional<cras::optional<sensor_msgs::CameraInfo>> getCameraInfoResult;
  cras::optional<cras::optional<sensor_msgs::Imu>> getImuResult;
  cras::optional<cras::optional<geometry_msgs::Quaternion>> getRollPitchOrientationResult;
  cras::optional<cras::optional<geometry_msgs::Transform>> getOpticalFrameTFResult;

  friend StackGuard;
};

class StackGuard
{
public:
  StackGuard(decltype(MetadataManager::callStack)& stack, const std::string& fn, const MetadataExtractor* extractor);
  ~StackGuard();
private:
  std::string getStackDescription() const;
  decltype(MetadataManager::callStack)& stack;
  std::string fn;
  const MetadataExtractor* extractor;
};
}
