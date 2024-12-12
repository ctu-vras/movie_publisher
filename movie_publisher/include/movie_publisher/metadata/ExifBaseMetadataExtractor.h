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

#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/optional.hpp>
#include <geometry_msgs/Quaternion.h>
#include <movie_publisher/metadata_extractor.h>
#include <movie_publisher/metadata_manager.h>
#include <ros/time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/NavSatFix.h>

namespace compass_conversions
{
  class CompassConverter;
}

namespace movie_publisher
{

typedef std::string ExifAscii;
typedef uint8_t ExifByte;
typedef uint16_t ExifShort;
typedef int16_t ExifSShort;
typedef uint32_t ExifLong;
typedef int32_t ExifSLong;
typedef double ExifRational;
typedef double ExifSRational;
typedef std::vector<uint8_t> ExifUnknown;

template<typename T>
struct ExifData
{
  std::string key;
  T value;
};

struct ExifBaseMetadataExtractorPrivate;

class ExifBaseMetadataExtractor : public MetadataExtractor
{
public:
  ExifBaseMetadataExtractor(
    const cras::LogHelperPtr& log, const std::weak_ptr<MetadataManager>& manager, size_t width, size_t height);
  ~ExifBaseMetadataExtractor() override;

  cras::optional<ros::Time> getCreationTime() override;
  cras::optional<std::string> getCameraSerialNumber() override;
  cras::optional<std::string> getCameraMake() override;
  cras::optional<std::string> getCameraModel() override;
  cras::optional<std::string> getLensMake() override;
  cras::optional<std::string> getLensModel() override;
  cras::optional<int> getRotation() override;
  cras::optional<double> getCropFactor() override;
  cras::optional<std::pair<double, double>> getSensorSizeMM() override;
  cras::optional<double> getFocalLength35MM() override;
  cras::optional<double> getFocalLengthMM() override;
  std::pair<cras::optional<sensor_msgs::NavSatFix>, cras::optional<gps_common::GPSFix>> getGNSSPosition() override;
  cras::optional<compass_msgs::Azimuth> getAzimuth() override;
  cras::optional<std::pair<double, double>> getRollPitch() override;
  cras::optional<geometry_msgs::Vector3> getAcceleration() override;

protected:
  size_t width;
  size_t height;

  compass_conversions::CompassConverter& getCompassConverter();

  virtual cras::optional<double> getGPSLatitude();
  virtual cras::optional<double> getGPSLongitude();
  virtual cras::optional<double> getGPSAltitude();
  virtual cras::optional<double> getGPSSpeed();
  virtual cras::optional<double> getGPSTrack();
  virtual cras::optional<double> getGPSImgDirection();
  virtual cras::optional<std::string> getGPSImgDirectionRef();
  virtual cras::optional<ros::Time> getGPSTime();

  virtual cras::optional<ExifData<ExifAscii>> getExifMake() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifAscii>> getExifModel() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifAscii>> getExifLensMake() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifAscii>> getExifLensModel() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifAscii>> getExifBodySerialNumber() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifAscii>> getExifLensSerialNumber() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifAscii>> getExifDateTimeOriginal() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifAscii>> getExifOffsetTimeOriginal() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifAscii>> getExifSubSecTimeOriginal() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifShort>> getExifOrientation() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifRational>> getExifFocalPlaneXRes() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifRational>> getExifFocalPlaneYRes() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifShort>> getExifFocalPlaneResUnit() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifShort>> getExifResUnit() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifShort>> getExifFocalLength35MM() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifRational>> getExifFocalLength() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifAscii>> getExifGpsLatRef() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifRational>> getExifGpsLat(size_t n) {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifAscii>> getExifGpsLonRef() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifRational>> getExifGpsLon(size_t n) {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifByte>> getExifGpsAltRef() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifRational>> getExifGpsAlt() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifAscii>> getExifGpsMeasureMode() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifRational>> getExifGpsDOP() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifAscii>> getExifGpsSpeedRef() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifRational>> getExifGpsSpeed() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifAscii>> getExifGpsTrackRef() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifRational>> getExifGpsTrack() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifRational>> getExifGpsTimeStamp(size_t n) {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifAscii>> getExifGpsDateStamp() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifShort>> getExifGpsDifferential() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifRational>> getExifGpsHPositioningError() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifAscii>> getExifGpsImgDirectionRef() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifRational>> getExifGpsImgDirection() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifSRational>> getExifAcceleration(size_t n) {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifSRational>> getExifRollAngle() {return cras::nullopt;}
  virtual cras::optional<ExifData<ExifSRational>> getExifPitchAngle() {return cras::nullopt;}

private:
  std::unique_ptr<ExifBaseMetadataExtractorPrivate> data;  //!< \brief PIMPL
};

}
