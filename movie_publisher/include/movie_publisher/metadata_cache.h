// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Metadata cache.
 * \author Martin Pecka
 */

#pragma once

#include <memory>
#include <string>

#include <movie_publisher/metadata_extractor.h>

namespace movie_publisher
{

/**
 * \brief Latest metadata of each type.
 */
struct LatestMetadataCache final
{
  LatestMetadataCache();
  ~LatestMetadataCache();

  void clear();

  cras::optional<cras::optional<std::string>>& getCameraGeneralName();
  const cras::optional<cras::optional<std::string>>& getCameraGeneralName() const;

  cras::optional<cras::optional<std::string>>& getCameraUniqueName();
  const cras::optional<cras::optional<std::string>>& getCameraUniqueName() const;

  cras::optional<cras::optional<std::string>>& getCameraSerialNumber();
  const cras::optional<cras::optional<std::string>>& getCameraSerialNumber() const;

  cras::optional<cras::optional<std::string>>& getCameraMake();
  const cras::optional<cras::optional<std::string>>& getCameraMake() const;

  cras::optional<cras::optional<std::string>>& getCameraModel();
  const cras::optional<cras::optional<std::string>>& getCameraModel() const;

  cras::optional<cras::optional<std::string>>& getLensMake();
  const cras::optional<cras::optional<std::string>>& getLensMake() const;

  cras::optional<cras::optional<std::string>>& getLensModel();
  const cras::optional<cras::optional<std::string>>& getLensModel() const;

  cras::optional<cras::optional<int>>& getRotation();
  const cras::optional<cras::optional<int>>& getRotation() const;

  cras::optional<cras::optional<ros::Time>>& getCreationTime();
  const cras::optional<cras::optional<ros::Time>>& getCreationTime() const;

  cras::optional<cras::optional<double>>& getCropFactor();
  const cras::optional<cras::optional<double>>& getCropFactor() const;

  cras::optional<cras::optional<SensorSize>>& getSensorSizeMM();
  const cras::optional<cras::optional<SensorSize>>& getSensorSizeMM() const;

  cras::optional<cras::optional<double>>& getFocalLength35MM();
  const cras::optional<cras::optional<double>>& getFocalLength35MM() const;

  cras::optional<cras::optional<double>>& getFocalLengthPx();
  const cras::optional<cras::optional<double>>& getFocalLengthPx() const;

  cras::optional<cras::optional<double>>& getFocalLengthMM();
  const cras::optional<cras::optional<double>>& getFocalLengthMM() const;

  cras::optional<cras::optional<IntrinsicMatrix>>& getIntrinsicMatrix();
  const cras::optional<cras::optional<IntrinsicMatrix>>& getIntrinsicMatrix() const;

  cras::optional<cras::optional<std::pair<DistortionType, Distortion>>>& getDistortion();
  const cras::optional<cras::optional<std::pair<DistortionType, Distortion>>>& getDistortion() const;

  cras::optional<GNSSFixAndDetail>& getGNSSPosition();
  const cras::optional<GNSSFixAndDetail>& getGNSSPosition() const;

  cras::optional<cras::optional<compass_msgs::Azimuth>>& getAzimuth();
  const cras::optional<cras::optional<compass_msgs::Azimuth>>& getAzimuth() const;

  cras::optional<cras::optional<sensor_msgs::MagneticField>>& getMagneticField();
  const cras::optional<cras::optional<sensor_msgs::MagneticField>>& getMagneticField() const;

  cras::optional<cras::optional<RollPitch>>& getRollPitch();
  const cras::optional<cras::optional<RollPitch>>& getRollPitch() const;

  cras::optional<cras::optional<geometry_msgs::Vector3>>& getAngularVelocity();
  const cras::optional<cras::optional<geometry_msgs::Vector3>>& getAngularVelocity() const;

  cras::optional<cras::optional<geometry_msgs::Vector3>>& getAcceleration();
  const cras::optional<cras::optional<geometry_msgs::Vector3>>& getAcceleration() const;

  cras::optional<cras::optional<sensor_msgs::CameraInfo>>& getCameraInfo();
  const cras::optional<cras::optional<sensor_msgs::CameraInfo>>& getCameraInfo() const;

  cras::optional<cras::optional<sensor_msgs::Imu>>& getImu();
  const cras::optional<cras::optional<sensor_msgs::Imu>>& getImu() const;

  cras::optional<cras::optional<geometry_msgs::Transform>>& getOpticalFrameTF();
  const cras::optional<cras::optional<geometry_msgs::Transform>>& getOpticalFrameTF() const;

  cras::optional<cras::optional<vision_msgs::Detection2DArray>>& getFaces();
  const cras::optional<cras::optional<vision_msgs::Detection2DArray>>& getFaces() const;

private:
  struct Impl;
  std::shared_ptr<Impl> data;  //!< PIMPL
};

/**
 * \brief Metadata cache.
 */
struct TimedMetadataCache final
{
  TimedMetadataCache();
  ~TimedMetadataCache();

  void clear();

  std::vector<TimedMetadata<int>>& rotation();
  const std::vector<TimedMetadata<int>>& rotation() const;

  std::vector<TimedMetadata<double>>& cropFactor();
  const std::vector<TimedMetadata<double>>& cropFactor() const;

  std::vector<TimedMetadata<SensorSize>>& sensorSizeMM();
  const std::vector<TimedMetadata<SensorSize>>& sensorSizeMM() const;

  std::vector<TimedMetadata<double>>& focalLength35MM();
  const std::vector<TimedMetadata<double>>& focalLength35MM() const;

  std::vector<TimedMetadata<double>>& focalLengthMM();
  const std::vector<TimedMetadata<double>>& focalLengthMM() const;

  std::vector<TimedMetadata<double>>& focalLengthPx();
  const std::vector<TimedMetadata<double>>& focalLengthPx() const;

  std::vector<TimedMetadata<IntrinsicMatrix>>& intrinsicMatrix();
  const std::vector<TimedMetadata<IntrinsicMatrix>>& intrinsicMatrix() const;

  std::vector<TimedMetadata<std::pair<DistortionType, Distortion>>>& distortion();
  const std::vector<TimedMetadata<std::pair<DistortionType, Distortion>>>& distortion() const;

  std::vector<TimedMetadata<compass_msgs::Azimuth>>& azimuth();
  const std::vector<TimedMetadata<compass_msgs::Azimuth>>& azimuth() const;

  std::vector<TimedMetadata<sensor_msgs::MagneticField>>& magneticField();
  const std::vector<TimedMetadata<sensor_msgs::MagneticField>>& magneticField() const;

  std::vector<TimedMetadata<RollPitch>>& rollPitch();
  const std::vector<TimedMetadata<RollPitch>>& rollPitch() const;

  std::vector<TimedMetadata<geometry_msgs::Vector3>>& acceleration();
  const std::vector<TimedMetadata<geometry_msgs::Vector3>>& acceleration() const;

  std::vector<TimedMetadata<geometry_msgs::Vector3>>& angularVelocity();
  const std::vector<TimedMetadata<geometry_msgs::Vector3>>& angularVelocity() const;

  std::vector<TimedMetadata<vision_msgs::Detection2DArray>>& faces();
  const std::vector<TimedMetadata<vision_msgs::Detection2DArray>>& faces() const;

  std::vector<TimedMetadata<sensor_msgs::CameraInfo>>& cameraInfo();
  const std::vector<TimedMetadata<sensor_msgs::CameraInfo>>& cameraInfo() const;

  std::vector<TimedMetadata<sensor_msgs::Imu>>& imu();
  const std::vector<TimedMetadata<sensor_msgs::Imu>>& imu() const;

  std::vector<TimedMetadata<geometry_msgs::Transform>>& opticalFrameTF();
  const std::vector<TimedMetadata<geometry_msgs::Transform>>& opticalFrameTF() const;

  std::vector<TimedMetadata<GNSSFixAndDetail>>& gnssPosition();
  const std::vector<TimedMetadata<GNSSFixAndDetail>>& gnssPosition() const;

private:
  struct Impl;
  std::unique_ptr<Impl> data;  //!< PIMPL
};

/**
 * \brief Metadata cache.
 */
struct MetadataCache final
{
  LatestMetadataCache latest;  //!< Latest metadata of each type.
  TimedMetadataCache timed;  //!< Timed metadata.
};

}
