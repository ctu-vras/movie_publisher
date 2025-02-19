// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Common base for all metadata extractors that utilize EXIF data.
 * \author Martin Pecka
 */

#include <limits>

#include <compass_conversions/compass_converter.h>
#include <cras_cpp_common/type_utils.hpp>
#include <movie_publisher/metadata/ExifBaseMetadataExtractor.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace movie_publisher
{
/**
 * \brief Convert a triple of hours, minutes and seconds into fractional number of seconds.
 * \param[in] hours Hours.
 * \param[in] minutes Minutes.
 * \param[in] seconds Seconds.
 * \return The fractional num of seconds.
 */
inline cras::optional<ExifData<ExifSRational>> hmsToSeconds(
  const cras::optional<ExifData<ExifRational>>& hours,
  const cras::optional<ExifData<ExifRational>>& minutes,
  const cras::optional<ExifData<ExifRational>>& seconds)
{
  if (!hours.has_value() || !minutes.has_value() || !seconds.has_value())
    return cras::nullopt;

  const auto h = hours->value;
  const auto m = minutes->value;
  const auto s = seconds->value;

  return ExifData<ExifSRational>{hours->key, h * 3600 + m * 60 + s};
}

/**
 * \brief Convert a triple of degrees, arvc minutes and arc seconds into fractional number of arc seconds.
 * \param[in] degrees Degrees.
 * \param[in] minutes Minutes.
 * \param[in] seconds Seconds.
 * \return The fractional num of seconds.
 */
inline cras::optional<ExifData<ExifSRational>> dmsToSeconds(
  const cras::optional<ExifData<ExifRational>>& degrees,
  const cras::optional<ExifData<ExifRational>>& minutes,
  const cras::optional<ExifData<ExifRational>>& seconds)
{
  return hmsToSeconds(degrees, minutes, seconds);
}

/**
 * \brief Convert a triple of hours, minutes and seconds into fractional number of hours.
 * \param[in] hours Hours.
 * \param[in] minutes Minutes.
 * \param[in] seconds Seconds.
 * \return The fractional num of hours.
 */
inline cras::optional<ExifData<ExifSRational>> hmsToHours(
  const cras::optional<ExifData<ExifRational>>& hours,
  const cras::optional<ExifData<ExifRational>>& minutes,
  const cras::optional<ExifData<ExifRational>>& seconds)
{
  const auto s = hmsToSeconds(hours, minutes, seconds);
  if (!s.has_value())
    return cras::nullopt;
  return ExifData<ExifSRational>{s->key, s->value / 3600.0};
}

/**
 * \brief Convert a triple of degrees, arc minutes and arc seconds into fractional number of degrees.
 * \param[in] degrees Degrees.
 * \param[in] minutes Minutes.
 * \param[in] seconds Seconds.
 * \return The fractional num of degrees.
 */
inline cras::optional<ExifData<ExifSRational>> dmsToDegrees(
  const cras::optional<ExifData<ExifRational>>& degrees,
  const cras::optional<ExifData<ExifRational>>& minutes,
  const cras::optional<ExifData<ExifRational>>& seconds)
{
  return hmsToHours(degrees, minutes, seconds);
}

struct ExifBaseMetadataExtractorPrivate
{
  std::unique_ptr<compass_conversions::CompassConverter> compassConverter;  //!< Compass converter.
  std::weak_ptr<MetadataManager> manager;  //!< Metadata manager.
};

ExifBaseMetadataExtractor::ExifBaseMetadataExtractor(
  const cras::LogHelperPtr& log, const std::weak_ptr<MetadataManager>& manager, const size_t width, const size_t height)
  : MetadataExtractor(log), width(width), height(height), data(new ExifBaseMetadataExtractorPrivate{})
{
  this->width = width;
  this->height = height;
  this->data->manager = manager;
}

ExifBaseMetadataExtractor::~ExifBaseMetadataExtractor() = default;

cras::optional<ros::Time> ExifBaseMetadataExtractor::getCreationTime()
{
  const auto maybeDate = this->getExifDateTimeOriginal();
  if (!maybeDate.has_value())
    return cras::nullopt;

  std::list<std::string> tags;
  tags.emplace_back(maybeDate->key);
  auto dateStr = maybeDate->value;

  const auto maybeOffset = this->getExifOffsetTimeOriginal();
  ros::Duration offset;
  if (maybeOffset.has_value())
  {
    try
    {
      offset = cras::parseTimezoneOffset(maybeOffset->value);
      tags.emplace_back(maybeOffset->key);
    }
    catch (const std::invalid_argument& e)
    {
      CRAS_WARN_NAMED("exif_base", "Error parsing OffsetTimeOriginal: %s", e.what());
    }
  }

  const auto maybeSubsec = this->getExifSubSecTimeOriginal();
  if (maybeSubsec.has_value() && !maybeSubsec->value.empty())
  {
    dateStr += "." + maybeSubsec->value;
    tags.push_back(maybeSubsec->key);
  }

  try
  {
    ros::Time result = cras::parseTime(dateStr, offset);
    if (result.isZero())
      return cras::nullopt;

    CRAS_DEBUG_NAMED("exif_base", "Creation time read from EXIF tags %s.", cras::to_string(tags).c_str());
    return result;
  }
  catch (const std::invalid_argument& e)
  {
    return cras::nullopt;
  }
}

cras::optional<std::string> ExifBaseMetadataExtractor::getCameraSerialNumber()
{
  std::list<std::string> tags;
  std::list<std::string> parts;

  const auto maybeBodySerial = this->getExifBodySerialNumber();
  const auto maybeLensSerial = this->getExifLensSerialNumber();

  if (maybeBodySerial.has_value() && !maybeBodySerial->value.empty())
  {
    parts.emplace_back(cras::strip(maybeBodySerial->value));
    tags.emplace_back(maybeBodySerial->key);
  }

  if (maybeLensSerial.has_value() && !maybeLensSerial->value.empty())
  {
    parts.emplace_back(cras::strip(maybeLensSerial->value));
    tags.emplace_back(maybeLensSerial->key);
  }

  if (parts.empty())
    return cras::nullopt;

  const auto serial = cras::join(parts, "-");
  CRAS_DEBUG_NAMED("exif_base",
    "Camera serial '%s' composed from EXIF tags %s.", serial.c_str(), cras::to_string(tags).c_str());
  return serial;
}

cras::optional<std::string> ExifBaseMetadataExtractor::getCameraMake()
{
  const auto maybeMake = this->getExifMake();
  if (!maybeMake.has_value())
    return cras::nullopt;

  const auto make = cras::strip(maybeMake->value);
  if (make.empty())
    return cras::nullopt;

  CRAS_DEBUG_NAMED("exif_base", "Camera make '%s' read from EXIF tag %s.", make.c_str(), maybeMake->key.c_str());
  return make;
}

cras::optional<std::string> ExifBaseMetadataExtractor::getCameraModel()
{
  const auto maybeModel = this->getExifModel();
  if (!maybeModel.has_value())
    return cras::nullopt;

  const auto model = cras::strip(maybeModel->value);
  if (model.empty())
    return cras::nullopt;

  CRAS_DEBUG_NAMED("exif_base", "Camera model '%s' read from EXIF tag %s.", model.c_str(), maybeModel->key.c_str());
  return model;
}

cras::optional<std::string> ExifBaseMetadataExtractor::getLensMake()
{
  const auto maybeMake = this->getExifLensMake();
  if (!maybeMake.has_value())
    return cras::nullopt;

  const auto make = cras::strip(maybeMake->value);
  if (make.empty())
    return cras::nullopt;

  CRAS_DEBUG_NAMED("exif_base", "Lens make '%s' read from EXIF tag %s.", make.c_str(), maybeMake->key.c_str());
  return make;
}

cras::optional<std::string> ExifBaseMetadataExtractor::getLensModel()
{
  const auto maybeModel = this->getExifLensModel();
  if (!maybeModel.has_value())
    return cras::nullopt;

  const auto model = cras::strip(maybeModel->value);
  if (model.empty())
    return cras::nullopt;

  CRAS_DEBUG_NAMED("exif_base", "Lens model '%s' read from EXIF tag %s.", model.c_str(), maybeModel->key.c_str());
  return model;
}

cras::optional<int> ExifBaseMetadataExtractor::getRotation()
{
  const auto maybeOrientation = this->getExifOrientation();
  if (!maybeOrientation.has_value())
    return cras::nullopt;

  const auto& orientation = maybeOrientation->value;
  int rotation;
  if (orientation == 6)
    rotation = 90;
  else if (orientation == 3)
    rotation = 180;
  else if (orientation == 8)
    rotation = 270;
  else
    rotation = 0;

  cras::TempLocale l(LC_ALL, "en_US.UTF-8");
  CRAS_DEBUG_NAMED("exif_base",
    "Image rotation %d° determined from Exif tag %s.", rotation, maybeOrientation->key.c_str());
  return rotation;
}

cras::optional<double> ExifBaseMetadataExtractor::getCropFactor()
{
  std::list<std::string> tags;

  const auto maybeFocResX = this->getExifFocalPlaneXRes();
  if (!maybeFocResX.has_value())
    return cras::nullopt;

  const auto maybeFocResUnit = this->getExifFocalPlaneResUnit();
  const auto maybeResUnit = this->getExifResUnit();

  ExifShort unit = 2;  // inches by default
  if (maybeFocResUnit.has_value())
  {
    unit = maybeFocResUnit->value;
    tags.emplace_back(maybeFocResUnit->key);
  }
  else if (maybeResUnit.has_value())
  {
    unit = maybeResUnit->value;
    tags.emplace_back(maybeResUnit->key);
  }

  const auto unitMM = unit == 2 ? 25.4 : 10.0;  // inch or cm

  const auto focResX = maybeFocResX->value;
  const auto sensorSizeMM = std::max(this->width, this->height) / (focResX / unitMM);

  const auto cropFactor = 36.0 / sensorSizeMM;
  CRAS_DEBUG_NAMED("exif_base",
    "Crop factor %.2f was determined from Exif tags %s.", cropFactor, cras::to_string(tags).c_str());

  return cropFactor;
}

cras::optional<std::pair<double, double>> ExifBaseMetadataExtractor::getSensorSizeMM()
{
  const auto cropFactor = this->getCropFactor();
  if (cropFactor == cras::nullopt)
    return cras::nullopt;

  const auto& w = this->width;
  const auto& h = this->height;
  const auto sensorWidthMM = 36.0 / *cropFactor;
  const auto sensorHeightMM = sensorWidthMM * std::min(w, h) / std::max(w, h);
  CRAS_DEBUG_NAMED("exif_base",
    "Sensor size %.1fx%1.f mm was determined from crop factor.", sensorWidthMM, sensorHeightMM);

  return std::pair{sensorWidthMM, sensorHeightMM};
}

cras::optional<double> ExifBaseMetadataExtractor::getFocalLength35MM()
{
  const auto maybef35mm = this->getExifFocalLength35MM();
  if (!maybef35mm.has_value())
    return cras::nullopt;

  const auto f35mm = maybef35mm->value;
  if (f35mm == 0)
    return cras::nullopt;

  CRAS_DEBUG_NAMED("exif_base",
    "Focal length %u mm (35 mm equiv) determined from Exif tag %s.", f35mm, maybef35mm->key.c_str());
  return f35mm;
}

cras::optional<double> ExifBaseMetadataExtractor::getFocalLengthMM()
{
  const auto maybeF = this->getExifFocalLength();
  if (!maybeF.has_value())
    return cras::nullopt;

  const auto f = maybeF->value;
  if (f == 0.0)
    return cras::nullopt;

  CRAS_DEBUG_NAMED("exif_base", "Real focal length %.1f mm determined from Exif tag %s.", f, maybeF->key.c_str());
  return f;
}

std::pair<cras::optional<sensor_msgs::NavSatFix>, cras::optional<gps_common::GPSFix>>
ExifBaseMetadataExtractor::getGNSSPosition()
{
  std::list<std::string> tags;

  sensor_msgs::NavSatFix navMsg {};
  gps_common::GPSFix gpsMsg {};

  bool hasNavData {false};
  bool hasGpsData {false};
  bool hasSpeed {false};

  const auto gpsLat = this->getGPSLatitude();
  if (gpsLat.has_value())
  {
    navMsg.latitude = gpsMsg.latitude = *gpsLat;
    hasNavData = hasGpsData = true;
  }

  const auto gpsLon = this->getGPSLongitude();
  if (gpsLon.has_value())
  {
    navMsg.longitude = gpsMsg.longitude = *gpsLon;
    hasNavData = hasGpsData = true;
  }

  const auto gpsAlt = this->getGPSAltitude();
  if (gpsAlt.has_value())
  {
    navMsg.altitude = gpsMsg.altitude = *gpsAlt;
    hasNavData = hasGpsData = true;
  }

  if (hasNavData && this->data->compassConverter != nullptr)
    this->data->compassConverter->setNavSatPos(navMsg);

  const auto gpsTime = this->getGPSTime();
  if (gpsTime.has_value())
  {
    gpsMsg.time = gpsTime->toSec();
    hasGpsData = true;
  }

  const auto gpsMeasureMode = this->getExifGpsMeasureMode();
  const auto gpsDOP = this->getExifGpsDOP();
  if (gpsMeasureMode.has_value() && gpsDOP.has_value())
  {
    if (gpsMeasureMode->value == "2")
      gpsMsg.hdop = gpsDOP->value;
    else
      gpsMsg.pdop = gpsDOP->value;
    tags.emplace_back(gpsMeasureMode->key); tags.emplace_back(gpsDOP->key);
    hasGpsData = true;
  }

  const auto gpsSpeed = this->getGPSSpeed();
  if (gpsSpeed.has_value())
  {
    gpsMsg.speed = *gpsSpeed;
    hasGpsData = true;
    hasSpeed = true;
  }

  const auto gpsTrack = this->getGPSTrack();
  if (gpsTrack.has_value())
  {
    gpsMsg.track = *gpsTrack;
    hasGpsData = true;
  }

  const auto gpsHPosError = this->getExifGpsHPositioningError();
  if (gpsHPosError.has_value())
  {
    tags.emplace_back(gpsHPosError->key);

    gpsMsg.err_horz = gpsHPosError->value;
    gpsMsg.position_covariance[0 * 3 + 0] = gpsMsg.position_covariance[1 * 3 + 1] = std::pow(gpsHPosError->value, 2);
    gpsMsg.position_covariance[2 * 3 + 2] = 10000 * 10000;

    navMsg.position_covariance = gpsMsg.position_covariance;
    navMsg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    gpsMsg.position_covariance_type = gps_common::GPSFix::COVARIANCE_TYPE_APPROXIMATED;
    hasNavData = hasGpsData = true;
  }

  const auto gpsCorrectionsUsed = this->getExifGpsDifferential();
  if (hasNavData)
  {
    navMsg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    if (gpsCorrectionsUsed.has_value())
    {
      tags.emplace_back(gpsCorrectionsUsed->key);
      if (gpsCorrectionsUsed->value > 0)
        navMsg.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
    }
  }

  if (hasGpsData)
  {
    gpsMsg.status.status = gps_common::GPSStatus::STATUS_FIX;
    if (gpsCorrectionsUsed.has_value())
    {
      tags.emplace_back(gpsCorrectionsUsed->key);
      if (gpsCorrectionsUsed->value > 0)
        gpsMsg.status.status = gps_common::GPSStatus::STATUS_DGPS_FIX;
    }
    gpsMsg.status.position_source = gps_common::GPSStatus::SOURCE_GPS;
    gpsMsg.status.motion_source =
      hasSpeed ? gps_common::GPSStatus::SOURCE_POINTS : gps_common::GPSStatus::SOURCE_NONE;
    gpsMsg.status.orientation_source = gps_common::GPSStatus::SOURCE_NONE;
  }

  std::pair<cras::optional<sensor_msgs::NavSatFix>, cras::optional<gps_common::GPSFix>> result;
  if (hasNavData)
    result.first = navMsg;
  if (hasGpsData)
    result.second = gpsMsg;

  if (hasNavData || hasGpsData)
  {
    const auto lat = navMsg.latitude ? navMsg.latitude : gpsMsg.latitude;
    const auto lon = navMsg.longitude ? navMsg.longitude : gpsMsg.longitude;
    const auto alt = navMsg.altitude ? navMsg.altitude : gpsMsg.altitude;
    cras::TempLocale l(LC_ALL, "en_US.UTF-8");
    CRAS_DEBUG_NAMED("exif_base",
      "GPS coordinates are %0.8f° %s, %0.8f° %s, %0.2f masl and other data have been read from Exif tags %s.",
      std::fabs(lat), lat >= 0 ? "N" : "S", std::fabs(lon), lon >= 0 ? "E" : "W", alt, cras::to_string(tags).c_str());
  }

  return result;
}

cras::optional<compass_msgs::Azimuth> ExifBaseMetadataExtractor::getAzimuth()
{
  const auto gpsImgDirection = this->getGPSImgDirection();
  const auto gpsImgDirectionRef = this->getGPSImgDirectionRef();

  if (!gpsImgDirection.has_value())
    return cras::nullopt;

  compass_msgs::Azimuth azimuthMsg;
  azimuthMsg.reference = gpsImgDirectionRef.value_or("T") == "M" ?
    compass_msgs::Azimuth::REFERENCE_MAGNETIC : compass_msgs::Azimuth::REFERENCE_GEOGRAPHIC;
  azimuthMsg.azimuth = *gpsImgDirection;
  azimuthMsg.unit = compass_msgs::Azimuth::UNIT_DEG;
  azimuthMsg.orientation = compass_msgs::Azimuth::ORIENTATION_NED;

  cras::TempLocale l(LC_ALL, "en_US.UTF-8");
  CRAS_DEBUG_NAMED("exif_base", "Azimuth %.1f° form North has been read from image direction.", *gpsImgDirection);
  return azimuthMsg;
}

cras::optional<std::pair<double, double>> ExifBaseMetadataExtractor::getRollPitch()
{
  const auto roll = this->getExifRollAngle();
  const auto pitch = this->getExifPitchAngle();

  if (!roll.has_value() || !pitch.has_value())
    return cras::nullopt;

  auto tags {std::list{roll->key, pitch->key}};
  tags.unique();

  const auto rollDeg = roll->value * 180.0 / M_PI;
  const auto pitchDeg = pitch->value * 180.0 / M_PI;

  cras::TempLocale l(LC_ALL, "en_US.UTF-8");
  CRAS_DEBUG_NAMED("exif_base",
    "Roll %.2f° and pitch %.2f° have been read from Exif tags %s.", rollDeg, pitchDeg, cras::to_string(tags).c_str());
  return std::pair{roll->value, pitch->value};
}

cras::optional<geometry_msgs::Vector3> ExifBaseMetadataExtractor::getAcceleration()
{
  const auto [accelX, accelY, accelZ] =
    std::tuple{this->getExifAcceleration(0), this->getExifAcceleration(1), this->getExifAcceleration(2)};

  if (!accelX.has_value() || !accelY.has_value() || !accelZ.has_value())
    return cras::nullopt;

  auto tags {std::list{accelX->key, accelY->key, accelZ->key}};
  tags.unique();

  geometry_msgs::Vector3 acc;
  acc.x = accelX->value;
  acc.y = accelY->value;
  acc.z = accelZ->value;

  CRAS_DEBUG_NAMED("exif_base",
    "Acceleration %.2f, %.2f, %.2f m/s^2 has been read from Exif tags %s.",
    acc.x, acc.y, acc.z, cras::to_string(tags).c_str());

  return acc;
}

compass_conversions::CompassConverter& ExifBaseMetadataExtractor::getCompassConverter()
{
  if (this->data->compassConverter == nullptr)
    this->data->compassConverter = std::make_unique<compass_conversions::CompassConverter>(this->log, false);
  return *this->data->compassConverter;
}

cras::optional<double> ExifBaseMetadataExtractor::getGPSLatitude()
{
  const auto gpsLatRef = this->getExifGpsLatRef();
  const auto gpsLatD = this->getExifGpsLat(0);
  const auto gpsLatM = this->getExifGpsLat(1);
  const auto gpsLatS = this->getExifGpsLat(2);

  const auto gpsLat = dmsToDegrees(gpsLatD, gpsLatM, gpsLatS);
  if (!gpsLatRef.has_value() || !gpsLat.has_value())
    return cras::nullopt;

  const auto result = gpsLat->value * (gpsLatRef->value == "N" ? 1 : -1);
  cras::TempLocale l(LC_ALL, "en_US.UTF-8");
  CRAS_DEBUG_NAMED("exif_base", "GPS latitude %.06f° has been read from Exif tags %s and %s.",
    result, gpsLatRef->key.c_str(), gpsLat->key.c_str());
  return result;
}

cras::optional<double> ExifBaseMetadataExtractor::getGPSLongitude()
{
  const auto gpsLonRef = this->getExifGpsLonRef();
  const auto gpsLonD = this->getExifGpsLon(0);
  const auto gpsLonM = this->getExifGpsLon(1);
  const auto gpsLonS = this->getExifGpsLon(2);

  const auto gpsLon = dmsToDegrees(gpsLonD, gpsLonM, gpsLonS);
  if (!gpsLonRef.has_value() || !gpsLon.has_value())
    return cras::nullopt;

  const auto result = gpsLon->value * (gpsLonRef->value == "E" ? 1 : -1);
  cras::TempLocale l(LC_ALL, "en_US.UTF-8");
  CRAS_DEBUG_NAMED("exif_base", "GPS longitude %.06f° has been read from Exif tags %s and %s.",
    result, gpsLonRef->key.c_str(), gpsLon->key.c_str());
  return result;
}

cras::optional<double> ExifBaseMetadataExtractor::getGPSAltitude()
{
  const auto gpsAltRef = this->getExifGpsAltRef();
  const auto gpsAlt = this->getExifGpsAlt();

  if (!gpsAltRef.has_value() || !gpsAlt.has_value())
    return cras::nullopt;

  const auto result = gpsAlt->value * (gpsAltRef->value == 0 ? 1 : -1);
  CRAS_DEBUG_NAMED("exif_base", "GPS altitude %.02f m.a.s.l. has been read from Exif tags %s and %s.",
    result, gpsAltRef->key.c_str(), gpsAlt->key.c_str());
  return result;
}

cras::optional<double> ExifBaseMetadataExtractor::getGPSSpeed()
{
  const auto gpsSpeedRef = this->getExifGpsSpeedRef();
  const auto gpsSpeed = this->getExifGpsSpeed();

  if (!gpsSpeedRef.has_value() || !gpsSpeed.has_value())
    return cras::nullopt;

  auto speed = gpsSpeed->value;
  if (gpsSpeedRef->value == "K")
    speed *= 3600.0 / 1000.0;
  else if (gpsSpeedRef->value == "M")
    speed *= 3600.0 / 1609.344;
  else if (gpsSpeedRef->value == "N")
    speed *= 3600.0 / 1870.0;

  const auto result = speed;
  CRAS_DEBUG_NAMED("exif_base", "GPS speed %.02f m/s has been read from Exif tags %s and %s.",
    result, gpsSpeedRef->key.c_str(), gpsSpeed->key.c_str());
  return result;
}

cras::optional<double> ExifBaseMetadataExtractor::getGPSTrack()
{
  const auto gpsTrackRef = this->getExifGpsTrackRef();
  const auto gpsTrack = this->getExifGpsTrack();

  if (!gpsTrackRef.has_value() || !gpsTrack.has_value())
    return cras::nullopt;

  auto track = gpsTrack->value;

  // If reference is "M", magnetic, convert the track to true north.
  if (gpsTrackRef.has_value() && gpsTrackRef->value == "M")
  {
    compass_msgs::Azimuth az;
    auto manager = this->data->manager.lock();
    az.header.stamp = cras::nowFallbackToWall();
    if (const auto creationTime = manager->getCreationTime(); creationTime.has_value())
      az.header.stamp = *creationTime;
    else if (const auto gnssPosition = manager->getGNSSPosition(); gnssPosition.second.has_value())
      az.header.stamp = ros::Time(gnssPosition.second->time);
    az.azimuth = track;
    az.unit = compass_msgs::Azimuth::UNIT_DEG;
    az.orientation = compass_msgs::Azimuth::ORIENTATION_NED;
    az.reference = compass_msgs::Azimuth::REFERENCE_UTM;

    const auto maybeAzimuth = this->getCompassConverter().convertAzimuth(
      az, compass_msgs::Azimuth::UNIT_DEG, compass_msgs::Azimuth::ORIENTATION_NED,
      compass_msgs::Azimuth::REFERENCE_GEOGRAPHIC);

    if (maybeAzimuth.has_value())
      track = maybeAzimuth->azimuth;
  }

  const auto result = track;
  cras::TempLocale l(LC_ALL, "en_US.UTF-8");
  CRAS_DEBUG_NAMED("exif_base", "GPS track %.03f° has been read from Exif tags %s and %s.",
    result, gpsTrackRef->key.c_str(), gpsTrack->key.c_str());
  return result;
}

cras::optional<double> ExifBaseMetadataExtractor::getGPSImgDirection()
{
  const auto gpsImgDirectionRef = this->getExifGpsImgDirectionRef();
  const auto gpsImgDirection = this->getExifGpsImgDirection();

  if (!gpsImgDirectionRef.has_value() || !gpsImgDirection.has_value())
    return cras::nullopt;

  const auto azimuth = gpsImgDirection->value;

  cras::TempLocale l(LC_ALL, "en_US.UTF-8");
  CRAS_DEBUG_NAMED("exif_base", "Image direction %.1f° form North has been read from Exif tags %s and %s.",
    azimuth, gpsImgDirectionRef->key.c_str(), gpsImgDirection->key.c_str());
  return azimuth;
}

cras::optional<std::string> ExifBaseMetadataExtractor::getGPSImgDirectionRef()
{
  const auto gpsImgDirectionRef = this->getExifGpsImgDirectionRef();

  if (!gpsImgDirectionRef.has_value())
    return cras::nullopt;

  return gpsImgDirectionRef->value;
}

cras::optional<ros::Time> ExifBaseMetadataExtractor::getGPSTime()
{
  const auto gpsTimestampH = this->getExifGpsTimeStamp(0);
  const auto gpsTimestampM = this->getExifGpsTimeStamp(1);
  const auto gpsTimestampS = this->getExifGpsTimeStamp(2);
  const auto gpsDatestamp = this->getExifGpsDateStamp();

  const auto gpsTime = hmsToSeconds(gpsTimestampH, gpsTimestampM, gpsTimestampS);
  if (!gpsTime.has_value() || !gpsDatestamp.has_value() || gpsDatestamp->value.empty())
    return cras::nullopt;

  try
  {
    const auto time = cras::parseTime(gpsDatestamp->value + " 00:00:00").toSec() + gpsTime->value;
    CRAS_DEBUG_NAMED("exif_base", "GPS time %.09f has been read from Exif tags %s and %s.",
      time, gpsTimestampH->key.c_str(), gpsDatestamp->key.c_str());
  }
  catch (const std::invalid_argument&) {}

  return cras::nullopt;
}

}
