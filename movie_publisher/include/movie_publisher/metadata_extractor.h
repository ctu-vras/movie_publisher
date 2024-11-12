// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief
 * \author Martin Pecka
 */

#pragma once

#include <memory>
#include <string>
#include <utility>

#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <geometry_msgs/Vector3.h>
#include <gps_common/GPSFix.h>
#include <ros/time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/NavSatFix.h>


class AVFormatContext;

namespace movie_publisher
{

using CI = sensor_msgs::CameraInfo;

class MetadataManager;

struct MetadataExtractorParams
{
  cras::LogHelperPtr log;
  std::weak_ptr<MetadataManager> manager;
  cras::BoundParamHelperPtr params;
  std::string filename;
  size_t width;
  size_t height;
  const AVFormatContext* avFormatContext;
  int streamIndex;
};

class MetadataExtractor : public cras::HasLogger
{
public:
  explicit MetadataExtractor(const cras::LogHelperPtr& log) : HasLogger(log) {}
  virtual ~MetadataExtractor() = default;

  virtual std::string getName() const = 0;
  virtual int getPriority() const = 0;

  virtual cras::optional<std::string> getCameraGeneralName() { return cras::nullopt; }
  virtual cras::optional<std::string> getCameraUniqueName() { return cras::nullopt; }
  virtual cras::optional<std::string> getCameraSerialNumber() { return cras::nullopt; }
  virtual cras::optional<std::string> getCameraMake() { return cras::nullopt; }
  virtual cras::optional<std::string> getCameraModel() { return cras::nullopt; }
  virtual cras::optional<std::string> getLensMake() { return cras::nullopt; }
  virtual cras::optional<std::string> getLensModel() { return cras::nullopt; }
  virtual cras::optional<int> getRotation() { return cras::nullopt; }
  virtual cras::optional<ros::Time> getCreationTime() { return cras::nullopt; }
  virtual cras::optional<double> getCropFactor() { return cras::nullopt; }
  virtual cras::optional<std::pair<double, double>> getSensorSizeMM() { return cras::nullopt; }
  virtual cras::optional<double> getFocalLength35MM() { return cras::nullopt; }
  virtual cras::optional<double> getFocalLengthMM() { return cras::nullopt; }
  virtual cras::optional<double> getFocalLengthPx() { return cras::nullopt; }
  virtual cras::optional<CI::_K_type> getIntrinsicMatrix() { return cras::nullopt; }
  virtual cras::optional<std::pair<CI::_distortion_model_type, CI::_D_type>> getDistortion() { return cras::nullopt; }
  virtual std::pair<cras::optional<sensor_msgs::NavSatFix>, cras::optional<gps_common::GPSFix>> getGNSSPosition()
  {
    return std::make_pair(cras::nullopt, cras::nullopt);
  }
  virtual cras::optional<compass_msgs::Azimuth> getAzimuth() { return cras::nullopt; }
  virtual cras::optional<std::pair<double, double>> getRollPitch() { return cras::nullopt; }
  virtual cras::optional<geometry_msgs::Vector3> getAcceleration() { return cras::nullopt; }

  typedef std::shared_ptr<MetadataExtractor> Ptr;
  typedef std::shared_ptr<const MetadataExtractor> ConstPtr;
};

struct MetadataExtractorPlugin
{
  virtual ~MetadataExtractorPlugin() = default;
  virtual MetadataExtractor::Ptr getExtractor(const MetadataExtractorParams& params) = 0;
};

}
