// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Metadata cache.
 * \author Martin Pecka
 */

#include <string>

#include <movie_publisher/metadata_cache.h>
#include <movie_publisher/metadata_extractor.h>

namespace movie_publisher
{

/**
 * \brief Latest data for each metadata type.
 */
struct LatestMetadataCache::Impl
{
  cras::optional<cras::optional<std::string>> getCameraGeneralName;
  cras::optional<cras::optional<std::string>> getCameraUniqueName;
  cras::optional<cras::optional<std::string>> getCameraSerialNumber;
  cras::optional<cras::optional<std::string>> getCameraMake;
  cras::optional<cras::optional<std::string>> getCameraModel;
  cras::optional<cras::optional<std::string>> getLensMake;
  cras::optional<cras::optional<std::string>> getLensModel;
  cras::optional<cras::optional<int>> getRotation;
  cras::optional<cras::optional<ros::Time>> getCreationTime;
  cras::optional<cras::optional<double>> getCropFactor;
  cras::optional<cras::optional<SensorSize>> getSensorSizeMM;
  cras::optional<cras::optional<double>> getFocalLength35MM;
  cras::optional<cras::optional<double>> getFocalLengthPx;
  cras::optional<cras::optional<double>> getFocalLengthMM;
  cras::optional<cras::optional<IntrinsicMatrix>> getIntrinsicMatrix;
  cras::optional<cras::optional<std::pair<DistortionType, Distortion>>> getDistortion;
  cras::optional<GNSSFixAndDetail> getGNSSPosition;
  cras::optional<cras::optional<compass_msgs::Azimuth>> getAzimuth;
  cras::optional<cras::optional<sensor_msgs::MagneticField>> getMagneticField;
  cras::optional<cras::optional<RollPitch>> getRollPitch;
  cras::optional<cras::optional<geometry_msgs::Vector3>> getAngularVelocity;
  cras::optional<cras::optional<geometry_msgs::Vector3>> getAcceleration;
  cras::optional<cras::optional<sensor_msgs::CameraInfo>> getCameraInfo;
  cras::optional<cras::optional<sensor_msgs::Imu>> getImu;
  cras::optional<cras::optional<geometry_msgs::Transform>> getOpticalFrameTF;
  cras::optional<cras::optional<geometry_msgs::Transform>> getZeroRollPitchTF;
  cras::optional<cras::optional<vision_msgs::Detection2DArray>> getFaces;
};

LatestMetadataCache::LatestMetadataCache() : data(new Impl())
{
}

LatestMetadataCache::~LatestMetadataCache() = default;

void LatestMetadataCache::clear()
{
  this->data->getCameraGeneralName.reset();
  this->data->getCameraUniqueName.reset();
  this->data->getCameraSerialNumber.reset();
  this->data->getCameraMake.reset();
  this->data->getCameraModel.reset();
  this->data->getLensMake.reset();
  this->data->getLensModel.reset();
  this->data->getRotation.reset();
  this->data->getCreationTime.reset();
  this->data->getCropFactor.reset();
  this->data->getSensorSizeMM.reset();
  this->data->getFocalLength35MM.reset();
  this->data->getFocalLengthPx.reset();
  this->data->getFocalLengthMM.reset();
  this->data->getIntrinsicMatrix.reset();
  this->data->getDistortion.reset();
  this->data->getGNSSPosition.reset();
  this->data->getAzimuth.reset();
  this->data->getMagneticField.reset();
  this->data->getRollPitch.reset();
  this->data->getAngularVelocity.reset();
  this->data->getAcceleration.reset();
  this->data->getCameraInfo.reset();
  this->data->getImu.reset();
  this->data->getOpticalFrameTF.reset();
  this->data->getZeroRollPitchTF.reset();
  this->data->getFaces.reset();
}

cras::optional<cras::optional<std::string>>& LatestMetadataCache::getCameraGeneralName()
{
  return this->data->getCameraGeneralName;
}
const cras::optional<cras::optional<std::string>>& LatestMetadataCache::getCameraGeneralName() const
{
  return this->data->getCameraGeneralName;
}

cras::optional<cras::optional<std::string>>& LatestMetadataCache::getCameraUniqueName()
{
  return this->data->getCameraUniqueName;
}
const cras::optional<cras::optional<std::string>>& LatestMetadataCache::getCameraUniqueName() const
{
  return this->data->getCameraUniqueName;
}

cras::optional<cras::optional<std::string>>& LatestMetadataCache::getCameraSerialNumber()
{
  return this->data->getCameraSerialNumber;
}
const cras::optional<cras::optional<std::string>>& LatestMetadataCache::getCameraSerialNumber() const
{
  return this->data->getCameraSerialNumber;
}

cras::optional<cras::optional<std::string>>& LatestMetadataCache::getCameraMake() { return this->data->getCameraMake; }
const cras::optional<cras::optional<std::string>>& LatestMetadataCache::getCameraMake() const
{
  return this->data->getCameraMake;
}

cras::optional<cras::optional<std::string>>& LatestMetadataCache::getCameraModel()
{
  return this->data->getCameraModel;
}
const cras::optional<cras::optional<std::string>>& LatestMetadataCache::getCameraModel() const
{
  return this->data->getCameraModel;
}

cras::optional<cras::optional<std::string>>& LatestMetadataCache::getLensMake() { return this->data->getLensMake; }
const cras::optional<cras::optional<std::string>>& LatestMetadataCache::getLensMake() const
{
  return this->data->getLensMake;
}

cras::optional<cras::optional<std::string>>& LatestMetadataCache::getLensModel() { return this->data->getLensModel; }
const cras::optional<cras::optional<std::string>>& LatestMetadataCache::getLensModel() const
{
  return this->data->getLensModel;
}

cras::optional<cras::optional<int>>& LatestMetadataCache::getRotation() { return this->data->getRotation; }
const cras::optional<cras::optional<int>>& LatestMetadataCache::getRotation() const { return this->data->getRotation; }

cras::optional<cras::optional<ros::Time>>& LatestMetadataCache::getCreationTime()
{
  return this->data->getCreationTime;
}
const cras::optional<cras::optional<ros::Time>>& LatestMetadataCache::getCreationTime() const
{
  return this->data->getCreationTime;
}

cras::optional<cras::optional<double>>& LatestMetadataCache::getCropFactor() { return this->data->getCropFactor; }
const cras::optional<cras::optional<double>>& LatestMetadataCache::getCropFactor() const
{
  return this->data->getCropFactor;
}

cras::optional<cras::optional<SensorSize>>& LatestMetadataCache::getSensorSizeMM()
{
  return this->data->getSensorSizeMM;
}
const cras::optional<cras::optional<SensorSize>>& LatestMetadataCache::getSensorSizeMM() const
{
  return this->data->getSensorSizeMM;
}

cras::optional<cras::optional<double>>& LatestMetadataCache::getFocalLength35MM()
{
  return this->data->getFocalLength35MM;
}
const cras::optional<cras::optional<double>>& LatestMetadataCache::getFocalLength35MM() const
{
  return this->data->getFocalLength35MM;
}

cras::optional<cras::optional<double>>& LatestMetadataCache::getFocalLengthPx() { return this->data->getFocalLengthPx; }
const cras::optional<cras::optional<double>>& LatestMetadataCache::getFocalLengthPx() const
{
  return this->data->getFocalLengthPx;
}

cras::optional<cras::optional<double>>& LatestMetadataCache::getFocalLengthMM() { return this->data->getFocalLengthMM; }
const cras::optional<cras::optional<double>>& LatestMetadataCache::getFocalLengthMM() const
{
  return this->data->getFocalLengthMM;
}

cras::optional<cras::optional<IntrinsicMatrix>>& LatestMetadataCache::getIntrinsicMatrix()
{
  return this->data->getIntrinsicMatrix;
}
const cras::optional<cras::optional<IntrinsicMatrix>>& LatestMetadataCache::getIntrinsicMatrix() const
{
  return this->data->getIntrinsicMatrix;
}

cras::optional<cras::optional<DistortionData>>& LatestMetadataCache::getDistortion()
{
  return this->data->getDistortion;
}
const cras::optional<cras::optional<DistortionData>>& LatestMetadataCache::getDistortion() const
{
  return this->data->getDistortion;
}

cras::optional<GNSSFixAndDetail>& LatestMetadataCache::getGNSSPosition() { return this->data->getGNSSPosition; }
const cras::optional<GNSSFixAndDetail>& LatestMetadataCache::getGNSSPosition() const
{
  return this->data->getGNSSPosition;
}

cras::optional<cras::optional<compass_msgs::Azimuth>>& LatestMetadataCache::getAzimuth()
{
  return this->data->getAzimuth;
}
const cras::optional<cras::optional<compass_msgs::Azimuth>>& LatestMetadataCache::getAzimuth() const
{
  return this->data->getAzimuth;
}

cras::optional<cras::optional<sensor_msgs::MagneticField>>& LatestMetadataCache::getMagneticField()
{
  return this->data->getMagneticField;
}
const cras::optional<cras::optional<sensor_msgs::MagneticField>>& LatestMetadataCache::getMagneticField() const
{
  return this->data->getMagneticField;
}

cras::optional<cras::optional<RollPitch>>& LatestMetadataCache::getRollPitch() { return this->data->getRollPitch; }
const cras::optional<cras::optional<RollPitch>>& LatestMetadataCache::getRollPitch() const
{
  return this->data->getRollPitch;
}

cras::optional<cras::optional<geometry_msgs::Vector3>>& LatestMetadataCache::getAngularVelocity()
{
  return this->data->getAngularVelocity;
}
const cras::optional<cras::optional<geometry_msgs::Vector3>>& LatestMetadataCache::getAngularVelocity() const
{
  return this->data->getAngularVelocity;
}

cras::optional<cras::optional<geometry_msgs::Vector3>>& LatestMetadataCache::getAcceleration()
{
  return this->data->getAcceleration;
}
const cras::optional<cras::optional<geometry_msgs::Vector3>>& LatestMetadataCache::getAcceleration() const
{
  return this->data->getAcceleration;
}

cras::optional<cras::optional<sensor_msgs::CameraInfo>>& LatestMetadataCache::getCameraInfo()
{
  return this->data->getCameraInfo;
}
const cras::optional<cras::optional<sensor_msgs::CameraInfo>>& LatestMetadataCache::getCameraInfo() const
{
  return this->data->getCameraInfo;
}

cras::optional<cras::optional<sensor_msgs::Imu>>& LatestMetadataCache::getImu() { return this->data->getImu; }
const cras::optional<cras::optional<sensor_msgs::Imu>>& LatestMetadataCache::getImu() const
{
  return this->data->getImu;
}

cras::optional<cras::optional<geometry_msgs::Transform>>& LatestMetadataCache::getOpticalFrameTF()
{
  return this->data->getOpticalFrameTF;
}
const cras::optional<cras::optional<geometry_msgs::Transform>>& LatestMetadataCache::getOpticalFrameTF() const
{
  return this->data->getOpticalFrameTF;
}

cras::optional<cras::optional<geometry_msgs::Transform>>& LatestMetadataCache::getZeroRollPitchTF()
{
  return this->data->getZeroRollPitchTF;
}
const cras::optional<cras::optional<geometry_msgs::Transform>>& LatestMetadataCache::getZeroRollPitchTF() const
{
  return this->data->getZeroRollPitchTF;
}

cras::optional<cras::optional<vision_msgs::Detection2DArray>>& LatestMetadataCache::getFaces()
{
  return this->data->getFaces;
}
const cras::optional<cras::optional<vision_msgs::Detection2DArray>>& LatestMetadataCache::getFaces() const
{
  return this->data->getFaces;
}


/**
 * \brief Timed metadata cache.
 */
struct TimedMetadataCache::Impl
{
  std::vector<TimedMetadata<int>> rotation;
  std::vector<TimedMetadata<double>> cropFactor;
  std::vector<TimedMetadata<SensorSize>> sensorSizeMM;
  std::vector<TimedMetadata<double>> focalLength35MM;
  std::vector<TimedMetadata<double>> focalLengthMM;
  std::vector<TimedMetadata<double>> focalLengthPx;
  std::vector<TimedMetadata<IntrinsicMatrix>> intrinsicMatrix;
  std::vector<TimedMetadata<std::pair<DistortionType, Distortion>>> distortion;
  std::vector<TimedMetadata<compass_msgs::Azimuth>> azimuth;
  std::vector<TimedMetadata<sensor_msgs::MagneticField>> magneticField;
  std::vector<TimedMetadata<RollPitch>> rollPitch;
  std::vector<TimedMetadata<geometry_msgs::Vector3>> acceleration;
  std::vector<TimedMetadata<geometry_msgs::Vector3>> angularVelocity;
  std::vector<TimedMetadata<vision_msgs::Detection2DArray>> faces;
  std::vector<TimedMetadata<sensor_msgs::CameraInfo>> cameraInfo;
  std::vector<TimedMetadata<sensor_msgs::Imu>> imu;
  std::vector<TimedMetadata<geometry_msgs::Transform>> opticalFrameTF;
  std::vector<TimedMetadata<geometry_msgs::Transform>> zeroRollPitchTF;
  std::vector<TimedMetadata<GNSSFixAndDetail>> gnssPosition;
};

TimedMetadataCache::TimedMetadataCache() : data(new Impl())
{
}

TimedMetadataCache::~TimedMetadataCache() = default;

void TimedMetadataCache::clear()
{
  this->data->rotation.clear();
  this->data->cropFactor.clear();
  this->data->sensorSizeMM.clear();
  this->data->focalLength35MM.clear();
  this->data->focalLengthMM.clear();
  this->data->focalLengthPx.clear();
  this->data->intrinsicMatrix.clear();
  this->data->distortion.clear();
  this->data->azimuth.clear();
  this->data->magneticField.clear();
  this->data->rollPitch.clear();
  this->data->acceleration.clear();
  this->data->angularVelocity.clear();
  this->data->faces.clear();
  this->data->cameraInfo.clear();
  this->data->imu.clear();
  this->data->opticalFrameTF.clear();
  this->data->zeroRollPitchTF.clear();
  this->data->gnssPosition.clear();
}

std::vector<TimedMetadata<int>>& TimedMetadataCache::rotation() { return this->data->rotation; }
const std::vector<TimedMetadata<int>>& TimedMetadataCache::rotation() const { return this->data->rotation; }

std::vector<TimedMetadata<double>>& TimedMetadataCache::cropFactor() { return this->data->cropFactor; }
const std::vector<TimedMetadata<double>>& TimedMetadataCache::cropFactor() const { return this->data->cropFactor; }

std::vector<TimedMetadata<SensorSize>>& TimedMetadataCache::sensorSizeMM() { return this->data->sensorSizeMM; }
const std::vector<TimedMetadata<SensorSize>>& TimedMetadataCache::sensorSizeMM() const
{
  return this->data->sensorSizeMM;
}

std::vector<TimedMetadata<double>>& TimedMetadataCache::focalLength35MM() { return this->data->focalLength35MM; }
const std::vector<TimedMetadata<double>>& TimedMetadataCache::focalLength35MM() const
{
  return this->data->focalLength35MM;
}

std::vector<TimedMetadata<double>>& TimedMetadataCache::focalLengthMM() { return this->data->focalLengthMM; }
const std::vector<TimedMetadata<double>>& TimedMetadataCache::focalLengthMM() const
{
  return this->data->focalLengthMM;
}

std::vector<TimedMetadata<double>>& TimedMetadataCache::focalLengthPx() { return this->data->focalLengthPx; }
const std::vector<TimedMetadata<double>>& TimedMetadataCache::focalLengthPx() const
{
  return this->data->focalLengthPx;
}

std::vector<TimedMetadata<IntrinsicMatrix>>& TimedMetadataCache::intrinsicMatrix()
{
  return this->data->intrinsicMatrix;
}
const std::vector<TimedMetadata<IntrinsicMatrix>>& TimedMetadataCache::intrinsicMatrix() const
{
  return this->data->intrinsicMatrix;
}

std::vector<TimedMetadata<std::pair<DistortionType, Distortion>>>& TimedMetadataCache::distortion()
{
  return this->data->distortion;
}
const std::vector<TimedMetadata<std::pair<DistortionType, Distortion>>>& TimedMetadataCache::distortion() const
{
  return this->data->distortion;
}

std::vector<TimedMetadata<compass_msgs::Azimuth>>& TimedMetadataCache::azimuth() { return this->data->azimuth; }
const std::vector<TimedMetadata<compass_msgs::Azimuth>>& TimedMetadataCache::azimuth() const
{
  return this->data->azimuth;
}

std::vector<TimedMetadata<sensor_msgs::MagneticField>>& TimedMetadataCache::magneticField()
{
  return this->data->magneticField;
}
const std::vector<TimedMetadata<sensor_msgs::MagneticField>>& TimedMetadataCache::magneticField() const
{
  return this->data->magneticField;
}

std::vector<TimedMetadata<RollPitch>>& TimedMetadataCache::rollPitch() { return this->data->rollPitch; }
const std::vector<TimedMetadata<RollPitch>>& TimedMetadataCache::rollPitch() const { return this->data->rollPitch; }

std::vector<TimedMetadata<geometry_msgs::Vector3>>& TimedMetadataCache::acceleration()
{
  return this->data->acceleration;
}
const std::vector<TimedMetadata<geometry_msgs::Vector3>>& TimedMetadataCache::acceleration() const
{
  return this->data->acceleration;
}

std::vector<TimedMetadata<geometry_msgs::Vector3>>& TimedMetadataCache::angularVelocity()
{
  return this->data->angularVelocity;
}
const std::vector<TimedMetadata<geometry_msgs::Vector3>>& TimedMetadataCache::angularVelocity() const
{
  return this->data->angularVelocity;
}

std::vector<TimedMetadata<vision_msgs::Detection2DArray>>& TimedMetadataCache::faces() { return this->data->faces; }
const std::vector<TimedMetadata<vision_msgs::Detection2DArray>>& TimedMetadataCache::faces() const
{
  return this->data->faces;
}

std::vector<TimedMetadata<sensor_msgs::CameraInfo>>& TimedMetadataCache::cameraInfo() { return this->data->cameraInfo; }
const std::vector<TimedMetadata<sensor_msgs::CameraInfo>>& TimedMetadataCache::cameraInfo() const
{
  return this->data->cameraInfo;
}

std::vector<TimedMetadata<sensor_msgs::Imu>>& TimedMetadataCache::imu() { return this->data->imu; }
const std::vector<TimedMetadata<sensor_msgs::Imu>>& TimedMetadataCache::imu() const { return this->data->imu; }

std::vector<TimedMetadata<geometry_msgs::Transform>>& TimedMetadataCache::opticalFrameTF()
{
  return this->data->opticalFrameTF;
}
const std::vector<TimedMetadata<geometry_msgs::Transform>>& TimedMetadataCache::opticalFrameTF() const
{
  return this->data->opticalFrameTF;
}

std::vector<TimedMetadata<geometry_msgs::Transform>>& TimedMetadataCache::zeroRollPitchTF()
{
  return this->data->zeroRollPitchTF;
}
const std::vector<TimedMetadata<geometry_msgs::Transform>>& TimedMetadataCache::zeroRollPitchTF() const
{
  return this->data->zeroRollPitchTF;
}

std::vector<TimedMetadata<GNSSFixAndDetail>>& TimedMetadataCache::gnssPosition() { return this->data->gnssPosition; }
const std::vector<TimedMetadata<GNSSFixAndDetail>>& TimedMetadataCache::gnssPosition() const
{
  return this->data->gnssPosition;
}
}
