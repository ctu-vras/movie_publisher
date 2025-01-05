// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Common base for all metadata extractors that utilize EXIF data.
 * \author Martin Pecka
 */

#pragma once

#include <memory>
#include <string>

#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/optional.hpp>
#include <movie_publisher/metadata_extractor.h>
#include <movie_publisher/metadata_manager.h>
#include <ros/time.h>
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

/**
 * \brief Common base for all metadata extractors that utilize EXIF data.
 *
 * This base class contains implementations of the standard MetadataExtractor interface by combining various EXIF
 * fields. Descendant classes are only responsible for reading the EXIF tags.
 */
class ExifBaseMetadataExtractor : public MetadataExtractor
{
public:
  /**
   * \brief Constructor.
   * \param[in] log Logger.
   * \param[in] manager Metadata manager.
   * \param[in] width Width of the movie [px].
   * \param[in] height Height of the movie [px].
   */
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
  size_t width;  //!< Width of the movie [px].
  size_t height;  //!< Height of the movie [px].

  /**
   * \brief Get a helper for converting various azimuth representations and references.
   *
   * Only call this method if you actually need the converter. Its construction is not free. It will be created lazily.
   *
   * \return The compass converter.
   */
  compass_conversions::CompassConverter& getCompassConverter();

  /**
   * \brief Construct latitude from GPSLatitude and GPSLatitudeRef.
   * \return Latitude [deg].
   */
  virtual cras::optional<double> getGPSLatitude();
  /**
   * \brief Construct latitude from GPSLongitude and GPSLongitudeRef.
   * \return Longitude [deg].
   */
  virtual cras::optional<double> getGPSLongitude();
  /**
   * \brief Construct altitude from GPSAltitude and GPSAltitudeRef.
   * \return Altitude [m].
   */
  virtual cras::optional<double> getGPSAltitude();
  /**
   * \brief Construct ground speed from GPSSpeed and GPSSpeedRef.
   * \return Ground speed [m/s].
   */
  virtual cras::optional<double> getGPSSpeed();
  /**
   * \brief Construct GPS track (heading) from GPSTrack and GPSTrackRef.
   * \return Track [deg] (referenced to geographic North).
   */
  virtual cras::optional<double> getGPSTrack();
  /**
   * \brief GPS image direction from GPSImgDirection.
   * \return Direction [deg] (referenced to geographic North by default, or to magnetic North if getGPSImgDirectionRef()
   *         returns "M").
   */
  virtual cras::optional<double> getGPSImgDirection();
  /**
   * \brief Get the image direction reference.
   * \return Image direction reference ("M" for magnetic North, otherwise geographic North).
   */
  virtual cras::optional<std::string> getGPSImgDirectionRef();
  /**
   * \brief Get the GPS time corresponding to the current frame.
   * \return The GPS time.
   */
  virtual cras::optional<ros::Time> getGPSTime();

  // Methods to be implemented by subclasses.

  /**
   * \return IFD0.Make (0x010f)
   */
  virtual cras::optional<ExifData<ExifAscii>> getExifMake() {return cras::nullopt;}
  /**
   * \return IFD0.Model (0x0110)
   */
  virtual cras::optional<ExifData<ExifAscii>> getExifModel() {return cras::nullopt;}
  /**
   * \return ExifIFD.LensMake (0xa433)
   */
  virtual cras::optional<ExifData<ExifAscii>> getExifLensMake() {return cras::nullopt;}
  /**
   * \return ExifIFD.LensModel (0xa434)
   */
  virtual cras::optional<ExifData<ExifAscii>> getExifLensModel() {return cras::nullopt;}
  /**
   * \return ExifIFD.BodySerialNumber (0xa431)
   */
  virtual cras::optional<ExifData<ExifAscii>> getExifBodySerialNumber() {return cras::nullopt;}
  /**
   * \return ExifIFD.LensSerialNumber (0xa435)
   */
  virtual cras::optional<ExifData<ExifAscii>> getExifLensSerialNumber() {return cras::nullopt;}
  /**
   * \return ExifIFD.DateTimeOriginal (0x9003)
   */
  virtual cras::optional<ExifData<ExifAscii>> getExifDateTimeOriginal() {return cras::nullopt;}
  /**
   * \return ExifIFD.OffsetTimeOriginal (0x9011)
   */
  virtual cras::optional<ExifData<ExifAscii>> getExifOffsetTimeOriginal() {return cras::nullopt;}
  /**
   * \return ExifIFD.SubsecTimeOriginal (0x9291)
   */
  virtual cras::optional<ExifData<ExifAscii>> getExifSubSecTimeOriginal() {return cras::nullopt;}
  /**
   * \return IFD0.Orientation (0x0112)
   */
  virtual cras::optional<ExifData<ExifShort>> getExifOrientation() {return cras::nullopt;}
  /**
   * \return ExifIFD.FocalPlaneXResolution (0xa20e)
   */
  virtual cras::optional<ExifData<ExifRational>> getExifFocalPlaneXRes() {return cras::nullopt;}
  /**
   * \return ExifIFD.FocalPlaneYResolution (0xa20f)
   */
  virtual cras::optional<ExifData<ExifRational>> getExifFocalPlaneYRes() {return cras::nullopt;}
  /**
   * \return ExifIFD.FocalPlaneResolutionUnit (0xa210)
   */
  virtual cras::optional<ExifData<ExifShort>> getExifFocalPlaneResUnit() {return cras::nullopt;}
  /**
   * \return IFD0.ResolutionUnit (0x0128)
   */
  virtual cras::optional<ExifData<ExifShort>> getExifResUnit() {return cras::nullopt;}
  /**
   * \return ExifIFD.FocalLengthIn35mmFormat (0xa405)
   */
  virtual cras::optional<ExifData<ExifShort>> getExifFocalLength35MM() {return cras::nullopt;}
  /**
   * \return ExifIFD.FocalLength (0x920a)
   */
  virtual cras::optional<ExifData<ExifRational>> getExifFocalLength() {return cras::nullopt;}
  /**
   * \return GPS.GPSLatitudeRef (0x0001)
   */
  virtual cras::optional<ExifData<ExifAscii>> getExifGpsLatRef() {return cras::nullopt;}
  /**
   * \return GPS.GPSLatitude (0x0002)
   */
  virtual cras::optional<ExifData<ExifRational>> getExifGpsLat(size_t n) {return cras::nullopt;}
  /**
   * \return GPS.GPSLongitudeRef (0x0003)
   */
  virtual cras::optional<ExifData<ExifAscii>> getExifGpsLonRef() {return cras::nullopt;}
  /**
   * \return GPS.GPSLongitude (0x0004)
   */
  virtual cras::optional<ExifData<ExifRational>> getExifGpsLon(size_t n) {return cras::nullopt;}
  /**
   * \return GPS.GPSAltitudeRef (0x0005)
   */
  virtual cras::optional<ExifData<ExifByte>> getExifGpsAltRef() {return cras::nullopt;}
  /**
   * \return GPS.GPSAltitude (0x0006)
   */
  virtual cras::optional<ExifData<ExifRational>> getExifGpsAlt() {return cras::nullopt;}
  /**
   * \return GPS.GPSMeasureMode (0x000a)
   */
  virtual cras::optional<ExifData<ExifAscii>> getExifGpsMeasureMode() {return cras::nullopt;}
  /**
   * \return GPS.GPSDOP (0x0006)
   */
  virtual cras::optional<ExifData<ExifRational>> getExifGpsDOP() {return cras::nullopt;}
  /**
   * \return GPS.GPSSpeedRef (0x000c)
   */
  virtual cras::optional<ExifData<ExifAscii>> getExifGpsSpeedRef() {return cras::nullopt;}
  /**
   * \return GPS.GPSSpeed (0x000d)
   */
  virtual cras::optional<ExifData<ExifRational>> getExifGpsSpeed() {return cras::nullopt;}
  /**
   * \return GPS.GPSTrackRef (0x000e)
   */
  virtual cras::optional<ExifData<ExifAscii>> getExifGpsTrackRef() {return cras::nullopt;}
  /**
   * \return GPS.GPSTrack (0x000f)
   */
  virtual cras::optional<ExifData<ExifRational>> getExifGpsTrack() {return cras::nullopt;}
  /**
   * \return GPS.GPSTimeStamp (0x0007)
   */
  virtual cras::optional<ExifData<ExifRational>> getExifGpsTimeStamp(size_t n) {return cras::nullopt;}
  /**
   * \return GPS.GPSDateStamp (0x001d)
   */
  virtual cras::optional<ExifData<ExifAscii>> getExifGpsDateStamp() {return cras::nullopt;}
  /**
   * \return GPS.GPSDifferential (0x001e)
   */
  virtual cras::optional<ExifData<ExifShort>> getExifGpsDifferential() {return cras::nullopt;}
  /**
   * \return GPS.GPSHPositioningError (0x001f)
   */
  virtual cras::optional<ExifData<ExifRational>> getExifGpsHPositioningError() {return cras::nullopt;}
  /**
   * \return GPS.GPSImgDirectionRef (0x0010)
   */
  virtual cras::optional<ExifData<ExifAscii>> getExifGpsImgDirectionRef() {return cras::nullopt;}
  /**
   * \return GPS.GPSImgDirection (0x0011)
   */
  virtual cras::optional<ExifData<ExifRational>> getExifGpsImgDirection() {return cras::nullopt;}
  /**
   * \return Acceleration read from makernotes (3 signed rationals, in geometrical frame of camera, m/s^2).
   */
  virtual cras::optional<ExifData<ExifSRational>> getExifAcceleration(size_t n) {return cras::nullopt;}
  /**
   * \return Roll angle [rad] read from makernotes.
   */
  virtual cras::optional<ExifData<ExifSRational>> getExifRollAngle() {return cras::nullopt;}
  /**
   * \return Pitch angle [rad] read from makernotes.
   */
  virtual cras::optional<ExifData<ExifSRational>> getExifPitchAngle() {return cras::nullopt;}

private:
  std::unique_ptr<ExifBaseMetadataExtractorPrivate> data;  //!< \brief PIMPL
};

}
