// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Metadata extractor using exiftool backend.
 * \author Martin Pecka
 */

#include "ExiftoolMetadataExtractor.h"

#include <sys/stat.h>

#include "ExifTool.h"

#include <cras_cpp_common/string_utils/ros.hpp>
#include <cras_cpp_common/type_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace movie_publisher
{
/**
 * \brief Get the fully qualified name of an EXIF tag.
 * \param[in] exif Exif tag.
 * \return The full name.
 */
std::string fullKeyName(const TagInfo* exif)
{
  std::vector<std::string> keyParts;
  if (exif->group[0] != nullptr) keyParts.emplace_back(exif->group[0]);
  if (exif->group[1] != nullptr) keyParts.emplace_back(exif->group[1]);
  if (exif->group[2] != nullptr) keyParts.emplace_back(exif->group[2]);
  keyParts.emplace_back(exif->name);
  return cras::join(keyParts, ".");
}

/**
 * \brief Search for the given EXIF keys and return the value of the first one that is valid.
 * \param[in] exifData Exif data of the movie.
 * \param[in] keys Possible EXIF keys to try.
 * \return The first valid EXIF data.
 */
auto getFirstValid(const std::unordered_map<std::string, TagInfo*>& exifData, const std::list<std::string>& keys)
{
  for (const auto& key : keys)
  {
    const auto& it = exifData.find(key);
    if (it != exifData.end() && it->second->valueLen > 0)
      return it;
  }
  return exifData.end();
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifAscii>> getExifString(
  const TagInfo* data, const size_t n = 0)
{
  return movie_publisher::ExifData<movie_publisher::ExifAscii>{fullKeyName(data), data->value};
}

template<typename ExifValue>
cras::optional<movie_publisher::ExifData<ExifValue>> getExifComponent(
  const std::function<ExifValue(const std::string&)>& convert, const TagInfo* data, const size_t n = 0)
{
  if (data == nullptr)
    return cras::nullopt;

  try
  {
    const auto parts = cras::split(data->value, " ");
    if (n >= parts.size())
      return cras::nullopt;
    return movie_publisher::ExifData<ExifValue>{fullKeyName(data), convert(parts[n])};
  }
  catch (const std::invalid_argument&) {}
  return cras::nullopt;
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifLong>> getExifLong(
  const TagInfo* data, const size_t n = 0)
{
  return getExifComponent<movie_publisher::ExifLong>(
    [](const std::string& s) {return cras::parseUInt32(s, 10);}, data, n);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifShort>> getExifShort(
  const TagInfo* data, const size_t n = 0)
{
  return getExifComponent<movie_publisher::ExifShort>(
    [](const std::string& s) {return cras::parseUInt16(s, 10);}, data, n);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifByte>> getExifByte(
  const TagInfo* data, const size_t n = 0)
{
  return getExifComponent<movie_publisher::ExifByte>(
    [](const std::string& s) {return cras::parseUInt8(s, 10);}, data, n);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifRational>> getExifRational(
  const TagInfo* data, const size_t n = 0)
{
  return getExifComponent<movie_publisher::ExifRational>(
    [](const std::string& s) {return cras::parseDouble(s);}, data, n);
}

/**
 * \brief Convert the given decimal seconds to HH:MM:SS and return the requested component.
 * \param[in] decimal A decimal value.
 * \param[in] n If 0, return degrees. If 1, return minutes. If 2, return seconds.
 * \return The requested value.
 */
double decimalToDMS(const double decimal, const size_t n)
{
  const auto degrees = std::floor(decimal);
  if (n == 0)
    return degrees;
  const auto minutes = std::floor((decimal - degrees) * 60);
  if (n == 1)
    return minutes;
  const auto seconds = (decimal - degrees - minutes / 60) * 3600;
  return seconds;
}

/**
 * \brief Private data.
 */
struct ExiftoolMetadataPrivate : cras::HasLogger
{
  std::string filename;  //!< Filename of the movie.

  std::unique_ptr<ExifTool> exiftool;  //!< Instance of the exiftool API.
  TagInfo* exifDataList{};  //!< The parsed EXIF data.
  cras::optional<std::unordered_map<std::string, TagInfo*>> exifData;  //!< The EXIF data.

  explicit ExiftoolMetadataPrivate(const cras::LogHelperPtr& log) : cras::HasLogger(log) {}

  ~ExiftoolMetadataPrivate()
  {
    if (this->exifData.has_value())
      this->exifData->clear();
    delete this->exifDataList;
  }

  void loadExifData()
  {
    if (this->exifData.has_value())
      return;

    // Turn off exiftool watchdog process. It has some kind of problem with the nodelet architecture and keeps running
    // even after shutting down the nodelet manager. We actually only use the exiftool once and then no longer need
    // it, so the probability of needing the watchdog is low.
    ExifTool::sNoWatchdog = 1;
    this->exiftool = std::make_unique<ExifTool>();

    cras::TempLocale l(LC_ALL, "en_US.UTF-8");
    CRAS_DEBUG_NAMED("exiftool", "exiftool: Loading file %s .", this->filename.c_str());
    this->exifDataList = this->exiftool->ImageInfo(this->filename.c_str(), "-u\n-U\n-n");
    this->exifData.emplace();

    if (this->exiftool->GetErrorLen() > 0)
    {
      CRAS_ERROR_NAMED("exiftool", "exiftool error: %s", this->exiftool->GetError());
      return;
    }

    for (auto exif = this->exifDataList; exif; exif = exif->next)
    {
      if (exif->name == nullptr)
        continue;
      const auto key = fullKeyName(exif);
      (*this->exifData)[key] = exif;
      (*this->exifData)[std::string("*.") + exif->name] = exif;  // Create a *.key item to allow searching by last item
      CRAS_DEBUG_NAMED("exiftool.dump", "exiftool %s=%s", key.c_str(), exif->value);
    }
  }
};

ExiftoolMetadataExtractor::ExiftoolMetadataExtractor(
  const cras::LogHelperPtr& log, const std::weak_ptr<MetadataManager>& manager, const std::string& filename,
  const size_t width, const size_t height)
  : ExifBaseMetadataExtractor(log, manager, width, height), data(new ExiftoolMetadataPrivate(log))
{
  this->data->filename = filename;
}

ExiftoolMetadataExtractor::~ExiftoolMetadataExtractor() = default;

std::string ExiftoolMetadataExtractor::getName() const
{
  return cras::getTypeName<std::remove_cv_t<std::remove_reference_t<decltype(*this)>>>();
}

int ExiftoolMetadataExtractor::getPriority() const
{
  return 55;
}

#define RETURN(keys, getValue, n) \
  this->data->loadExifData(); \
  if (!this->data->exifData.has_value()) \
    return cras::nullopt; \
  const auto& exifData = *this->data->exifData; \
  const auto& data = getFirstValid(exifData, std::list<std::string>{(keys)}); \
  if (data == exifData.end()) \
    return cras::nullopt; \
  return getValue(data->second, n);

#define RETURN_STRING(keys) RETURN((keys), getExifString, 0)
#define RETURN_LONG(keys) RETURN((keys), getExifLong, 0)
#define RETURN_SHORT(keys) RETURN((keys), getExifShort, 0)
#define RETURN_BYTE(keys) RETURN((keys), getExifByte, 0)
#define RETURN_RATIONAL_N(keys, n) RETURN((keys), getExifRational, n)
#define RETURN_RATIONAL(keys) RETURN_RATIONAL_N((keys), 0)

cras::optional<ExifData<ExifAscii>> ExiftoolMetadataExtractor::getExifMake()
{
  const std::list<std::string> keys = {
    "EXIF.IFD0.Camera.Make", "QuickTime.Keys.Camera.Make", "QuickTime.QuickTime.Camera.Make"};
  RETURN_STRING(keys);
}

cras::optional<ExifData<ExifAscii>> ExiftoolMetadataExtractor::getExifModel()
{
  const std::list<std::string> keys = {
    "EXIF.IFD0.Camera.Model", "QuickTime.Keys.Camera.Model", "QuickTime.QuickTime.Camera.Model"};
  RETURN_STRING(keys);
}

cras::optional<ExifData<ExifAscii>> ExiftoolMetadataExtractor::getExifLensMake()
{
  RETURN_STRING("EXIF.ExifIFD.Image.LensMake");
}

cras::optional<ExifData<ExifAscii>> ExiftoolMetadataExtractor::getExifLensModel()
{
  const std::list<std::string> keys = {"Composite.Composite.Camera.LensID", "QuickTime.Keys.Audio.CameraLensModel",
    "*.CameraLensModel", "*.LensModel", "QuickTime.QuickTime.Audio.CameraLens_model"};
  RETURN_STRING(keys);
}

cras::optional<ExifData<ExifAscii>> ExiftoolMetadataExtractor::getExifBodySerialNumber()
{
  const std::list<std::string> keys = {"*.SerialNumber", "*.InternalSerialNumber"};
  RETURN_STRING(keys);
}

cras::optional<ExifData<ExifAscii>> ExiftoolMetadataExtractor::getExifLensSerialNumber()
{
  RETURN_STRING("*.LensSerialNumber");
}

cras::optional<ExifData<ExifAscii>> ExiftoolMetadataExtractor::getExifDateTimeOriginal()
{
  const std::list<std::string> keys = {"EXIF.ExifIFD.Time.DateTimeOriginal", "QuickTime.Keys.Time.CreationDate",
    "QuickTime.QuickTime.Time.CreateDate"};
  RETURN_STRING(keys);
}

cras::optional<ExifData<ExifAscii>> ExiftoolMetadataExtractor::getExifOffsetTimeOriginal()
{
  RETURN_STRING("EXIF.ExifIFD.Time.OffsetTimeOriginal");
}

cras::optional<ExifData<ExifAscii>> ExiftoolMetadataExtractor::getExifSubSecTimeOriginal()
{
  RETURN_STRING("EXIF.ExifIFD.Time.OffsetTimeOriginal");
}

cras::optional<ros::Time> ExiftoolMetadataExtractor::getCreationTime()
{
  this->data->loadExifData();
  if (!this->data->exifData.has_value())
    return cras::nullopt;
  const auto& exifData = *this->data->exifData;

  const auto it = exifData.find("Composite.Composite.Time.SubSecDateTimeOriginal");
  if (it != exifData.end())
  {
    const auto& tag = it->first;
    const std::string value = it->second->value;

    try
    {
      const auto result = cras::parseTime(value);
      CRAS_DEBUG_NAMED("exiftool", "Creation time read from EXIF tag %s.", it->first.c_str());
      return result;
    }
    catch (const std::invalid_argument& e)
    {
      CRAS_ERROR_NAMED("exiftool", "Error reading image metadata: %s", e.what());
    }
  }

  return ExifBaseMetadataExtractor::getCreationTime();
}

cras::optional<int> ExiftoolMetadataExtractor::getRotation()
{
  this->data->loadExifData();
  if (!this->data->exifData.has_value())
    return cras::nullopt;
  const auto& exifData = *this->data->exifData;

  const auto it = exifData.find("Composite.Composite.Video.Rotation");
  if (it != exifData.end())
  {
    try
    {
      cras::TempLocale l(LC_ALL, "en_US.UTF-8");
      const int rotation = cras::parseInt16(it->second->value, 10);
      CRAS_DEBUG_NAMED("exiftool", "Image rotation %d° determined from EXIF tag %s.", rotation, it->first.c_str());
      return rotation;
    }
    catch (const std::invalid_argument& e)
    {
      CRAS_ERROR_NAMED("exiftool", "Error reading image metadata: %s", e.what());
    }
  }

  return ExifBaseMetadataExtractor::getRotation();
}

cras::optional<double> ExiftoolMetadataExtractor::getCropFactor()
{
  this->data->loadExifData();
  if (!this->data->exifData.has_value())
    return cras::nullopt;
  const auto& exifData = *this->data->exifData;

  const auto scaleFactorIt = exifData.find("Composite.Composite.Camera.ScaleFactor35efl");
  if (scaleFactorIt != exifData.end())
  {
    const auto cropFactor = cras::parseDouble(scaleFactorIt->second->value);
    CRAS_DEBUG_NAMED("exiftool", "Crop factor %.2f was determined from %s.", cropFactor, scaleFactorIt->first.c_str());
    return cropFactor;
  }

  return ExifBaseMetadataExtractor::getCropFactor();
}

cras::optional<ExifData<ExifShort>> ExiftoolMetadataExtractor::getExifOrientation()
{
  RETURN_SHORT("EXIF.IFD0.Image.Orientation");
}

cras::optional<ExifData<ExifRational>> ExiftoolMetadataExtractor::getExifFocalPlaneXRes()
{
  RETURN_RATIONAL("EXIF.ExifIFD.Camera.FocalPlaneXResolution");
}

cras::optional<ExifData<ExifRational>> ExiftoolMetadataExtractor::getExifFocalPlaneYRes()
{
  RETURN_RATIONAL("EXIF.ExifIFD.Camera.FocalPlaneYResolution");
}

cras::optional<ExifData<ExifShort>> ExiftoolMetadataExtractor::getExifFocalPlaneResUnit()
{
  RETURN_SHORT("EXIF.ExifIFD.Camera.FocalPlaneResolutionUnit");
}

cras::optional<ExifData<ExifShort>> ExiftoolMetadataExtractor::getExifResUnit()
{
  RETURN_SHORT("EXIF.ExifIFD.Image.ResolutionUnit");
}

cras::optional<ExifData<ExifShort>> ExiftoolMetadataExtractor::getExifFocalLength35MM()
{
  this->data->loadExifData();
  if (!this->data->exifData.has_value())
    return cras::nullopt;
  const auto& exifData = *this->data->exifData;

  const auto focalLength35mmIt = getFirstValid(exifData, {
    "Composite.Composite.Camera.FocalLength35efl",
    "QuickTime.Keys.Audio.CameraFocalLength35mmEquivalent",
    "QuickTime.QuickTime.Audio.CameraFocal_length35mm_equivalent",
  });
  if (focalLength35mmIt == exifData.end() || focalLength35mmIt->second->valueLen == 0 ||
      strcmp(focalLength35mmIt->second->value, "0") == 0)
    return cras::nullopt;

  // Sometimes exiftool fills focalLength35efl even though it doesn't know the scale factor. In such case, it just
  // copies the physical focal length there, which is wrong.
  if (focalLength35mmIt->first == "Composite.Composite.Camera.FocalLength35efl" &&
      exifData.find("Composite.Composite.Camera.ScaleFactor35efl") == exifData.end())
    return cras::nullopt;

  try
  {
    const auto f35mm = cras::parseDouble(focalLength35mmIt->second->value);
    return movie_publisher::ExifData<movie_publisher::ExifShort>{
      fullKeyName(focalLength35mmIt->second), static_cast<movie_publisher::ExifShort>(f35mm)
    };
  }
  catch (const std::invalid_argument& e)
  {
    CRAS_ERROR_NAMED("exiftool", "Error reading image metadata: %s", e.what());
  }

  return cras::nullopt;
}

cras::optional<ExifData<ExifRational>> ExiftoolMetadataExtractor::getExifFocalLength()
{
  RETURN_RATIONAL("EXIF.ExifIFD.Camera.FocalLength");
}

cras::optional<double> ExiftoolMetadataExtractor::getGPSLatitude()
{
  this->data->loadExifData();
  if (!this->data->exifData.has_value())
    return cras::nullopt;
  const auto& exifData = *this->data->exifData;

  const auto& it = exifData.find("Composite.Composite.Location.GPSLatitude");
  if (it == exifData.end())
    return ExifBaseMetadataExtractor::getGPSLatitude();

  try
  {
    const auto result = cras::parseDouble(it->second->value);
    cras::TempLocale l(LC_ALL, "en_US.UTF-8");
    CRAS_DEBUG_NAMED("exif_base",
      "GPS latitude %.06f° has been read from Exif tag %s", result, fullKeyName(it->second).c_str());
    return result;
  }
  catch (const std::invalid_argument& e) {}

  return ExifBaseMetadataExtractor::getGPSLatitude();
}

cras::optional<double> ExiftoolMetadataExtractor::getGPSLongitude()
{
  this->data->loadExifData();
  if (!this->data->exifData.has_value())
    return cras::nullopt;
  const auto& exifData = *this->data->exifData;

  const auto& it = exifData.find("Composite.Composite.Location.GPSLongitude");
  if (it == exifData.end())
    return ExifBaseMetadataExtractor::getGPSLongitude();

  try
  {
    const auto result = cras::parseDouble(it->second->value);
    cras::TempLocale l(LC_ALL, "en_US.UTF-8");
    CRAS_DEBUG_NAMED("exif_base",
      "GPS longitude %.06f° has been read from Exif tag %s", result, fullKeyName(it->second).c_str());
    return result;
  }
  catch (const std::invalid_argument& e) {}

  return ExifBaseMetadataExtractor::getGPSLongitude();
}

cras::optional<double> ExiftoolMetadataExtractor::getGPSAltitude()
{
  this->data->loadExifData();
  if (!this->data->exifData.has_value())
    return cras::nullopt;
  const auto& exifData = *this->data->exifData;

  const auto& it = exifData.find("Composite.Composite.Location.GPSAltitude");
  if (it == exifData.end())
    return ExifBaseMetadataExtractor::getGPSAltitude();

  try
  {
    const auto result = cras::parseDouble(it->second->value);
    CRAS_DEBUG_NAMED("exif_base",
      "GPS altitude %.02f m.a.s.l. has been read from Exif tag %s", result, fullKeyName(it->second).c_str());
    return result;
  }
  catch (const std::invalid_argument& e) {}

  return ExifBaseMetadataExtractor::getGPSAltitude();
}
cras::optional<ros::Time> ExiftoolMetadataExtractor::getGPSTime()
{
  this->data->loadExifData();
  if (!this->data->exifData.has_value())
    return cras::nullopt;
  const auto& exifData = *this->data->exifData;

  const auto& it = exifData.find("Composite.Composite.Time.GPSDateTime");
  if (it == exifData.end())
    return ExifBaseMetadataExtractor::getGPSTime();

  try
  {
    const auto result = cras::parseTime(it->second->value);
    CRAS_DEBUG_NAMED("exif_base",
      "GPS time %.09f has been read from Exif tag %s", result.toSec(), fullKeyName(it->second).c_str());
    return result;
  }
  catch (const std::invalid_argument& e) {}

  return ExifBaseMetadataExtractor::getGPSTime();
}

cras::optional<ExifData<ExifAscii>> ExiftoolMetadataExtractor::getExifGpsLatRef()
{
  const std::list<std::string> keys = {"EXIF.GPS.Location.GPSLatitudeRef",
    "Composite.Composite.Location.GPSLatitudeRef"};
  RETURN_STRING(keys);
}

cras::optional<ExifData<ExifRational>> ExiftoolMetadataExtractor::getExifGpsLat(const size_t n)
{
  this->data->loadExifData();
  if (!this->data->exifData.has_value())
    return cras::nullopt;
  const auto& exifData = *this->data->exifData;

  if (n > 2)
    return cras::nullopt;

  const auto& it = exifData.find("EXIF.GPS.Location.GPSLatitude");
  if (it == exifData.end())
    return cras::nullopt;

  try
  {
    const auto floatVal = std::fabs(cras::parseDouble(it->second->value));
    return ExifData<ExifRational>{it->first, decimalToDMS(floatVal, n)};
  }
  catch (const std::invalid_argument& e) {}

  return cras::nullopt;
}

cras::optional<ExifData<ExifAscii>> ExiftoolMetadataExtractor::getExifGpsLonRef()
{
  const std::list<std::string> keys = {"EXIF.GPS.Location.GPSLongitudeRef",
    "Composite.Composite.Location.GPSLongitudeRef"};
  RETURN_STRING(keys);
}

cras::optional<ExifData<ExifRational>> ExiftoolMetadataExtractor::getExifGpsLon(const size_t n)
{
  this->data->loadExifData();
  if (!this->data->exifData.has_value())
    return cras::nullopt;
  const auto& exifData = *this->data->exifData;

  if (n > 2)
    return cras::nullopt;

  const auto& it = exifData.find("EXIF.GPS.Location.GPSLongitude");
  if (it == exifData.end())
    return cras::nullopt;

  try
  {
    const auto floatVal = std::fabs(cras::parseDouble(it->second->value));
    return ExifData<ExifRational>{it->first, decimalToDMS(floatVal, n)};
  }
  catch (const std::invalid_argument& e) {}

  return cras::nullopt;
}

cras::optional<ExifData<ExifByte>> ExiftoolMetadataExtractor::getExifGpsAltRef()
{
  const std::list<std::string> keys = {"EXIF.GPS.Location.GPSAltitudeRef",
    "Composite.Composite.Location.GPSAltitudeRef"};
  RETURN_BYTE(keys);
}

cras::optional<ExifData<ExifRational>> ExiftoolMetadataExtractor::getExifGpsAlt()
{
  RETURN_RATIONAL("EXIF.GPS.Location.GPSAltitude");
}

cras::optional<ExifData<ExifAscii>> ExiftoolMetadataExtractor::getExifGpsMeasureMode()
{
  RETURN_STRING("EXIF.GPS.Location.GPSMeasureMode");
}

cras::optional<ExifData<ExifRational>> ExiftoolMetadataExtractor::getExifGpsDOP()
{
  RETURN_RATIONAL("EXIF.GPS.Location.GPSDOP");
}

cras::optional<ExifData<ExifAscii>> ExiftoolMetadataExtractor::getExifGpsSpeedRef()
{
  RETURN_STRING("EXIF.GPS.Location.GPSSpeedRef");
}

cras::optional<ExifData<ExifRational>> ExiftoolMetadataExtractor::getExifGpsSpeed()
{
  RETURN_RATIONAL("EXIF.GPS.Location.GPSSpeed");
}

cras::optional<ExifData<ExifAscii>> ExiftoolMetadataExtractor::getExifGpsTrackRef()
{
  RETURN_STRING("EXIF.GPS.Location.GPSTrackRef");
}

cras::optional<ExifData<ExifRational>> ExiftoolMetadataExtractor::getExifGpsTrack()
{
  RETURN_RATIONAL("EXIF.GPS.Location.GPSTrack");
}

cras::optional<ExifData<ExifRational>> ExiftoolMetadataExtractor::getExifGpsTimeStamp(const size_t n)
{
  this->data->loadExifData();
  if (!this->data->exifData.has_value())
    return cras::nullopt;
  const auto& exifData = *this->data->exifData;

  if (n > 2)
    return cras::nullopt;

  const auto& it = exifData.find("EXIF.GPS.Time.GPSTimeStamp");
  if (it == exifData.end())
    return cras::nullopt;

  const auto parts = cras::split(it->second->value, ":");
  if (parts.size() != 3)
    return cras::nullopt;

  try
  {
    return ExifData<ExifRational>{it->first, static_cast<ExifSRational>(cras::parseUInt8(parts[n], 10))};
  }
  catch (const std::invalid_argument& e) {}

  return cras::nullopt;
}

cras::optional<ExifData<ExifAscii>> ExiftoolMetadataExtractor::getExifGpsDateStamp()
{
  RETURN_STRING("EXIF.GPS.Time.GPSDateStamp");
}

cras::optional<ExifData<ExifShort>> ExiftoolMetadataExtractor::getExifGpsDifferential()
{
  RETURN_SHORT("EXIF.GPS.Location.GPSDifferential");
}

cras::optional<ExifData<ExifRational>> ExiftoolMetadataExtractor::getExifGpsHPositioningError()
{
  const std::list<std::string> keys = {"EXIF.GPS.Location.GPSHPositioningError",
    "QuickTime.Keys.Audio.LocationAccuracyHorizontal", "*.LocationAccuracyHorizontal"};
  RETURN_RATIONAL(keys);
}

cras::optional<ExifData<ExifAscii>> ExiftoolMetadataExtractor::getExifGpsImgDirectionRef()
{
  RETURN_STRING("EXIF.GPS.Location.GPSImgDirectionRef");
}

cras::optional<ExifData<ExifRational>> ExiftoolMetadataExtractor::getExifGpsImgDirection()
{
  RETURN_RATIONAL("EXIF.GPS.Location.GPSImgDirection");
}

cras::optional<ExifData<ExifSRational>> ExiftoolMetadataExtractor::getExifAcceleration(const size_t n)
{
  this->data->loadExifData();
  if (!this->data->exifData.has_value())
    return cras::nullopt;
  const auto& exifData = *this->data->exifData;

  if (n > 2)
    return cras::nullopt;

  try
  {
    if (const auto& it = exifData.find("MakerNotes.Apple.Camera.AccelerationVector"); it != exifData.end())
    {
      const auto accels = cras::split(it->second->value, " ");
      if (accels.size() == 3)
      {
        // TODO Maybe Apple rotates the vector by screen orientation
        return ExifData<ExifSRational>{fullKeyName(it->second), cras::parseDouble(accels[n]) * 9.8};
      }
    }
    else if (const auto& itX = exifData.find("*.AccelerometerX"); itX != exifData.end())
    {
      const auto& itY = exifData.find("*.AccelerometerY");
      const auto& itZ = exifData.find("*.AccelerometerZ");

      if (itY != exifData.end() && itZ != exifData.end())
      {
        auto accel = tf2::Vector3{
          -cras::parseDouble(itY->second->value),
          cras::parseDouble(itX->second->value),
          cras::parseDouble(itZ->second->value)
        };
        accel.normalize();
        accel *= 9.8;
        std::string key;
        switch (n)
        {
          case 0:
            key = fullKeyName(itY->second);
            break;
          case 1:
            key = fullKeyName(itX->second);
            break;
          case 2:
            key = fullKeyName(itZ->second);
            break;
        }
        return ExifData<ExifSRational>{key, accel[n]};
      }
    }
  }
  catch (const std::invalid_argument& e)
  {
    CRAS_ERROR_NAMED("exiftool", "Error reading image metadata: %s", e.what());
  }

  return cras::nullopt;
}

cras::optional<ExifData<ExifSRational>> ExiftoolMetadataExtractor::getExifRollAngle()
{
  this->data->loadExifData();
  if (!this->data->exifData.has_value())
    return cras::nullopt;
  const auto& exifData = *this->data->exifData;

  const auto& it = exifData.find("*.RollAngle");
  if (it == exifData.end())
    return cras::nullopt;

  try
  {
    return ExifData<ExifSRational>{fullKeyName(it->second), cras::parseDouble(it->second->value) * M_PI / 180.0};
  }
  catch (const std::invalid_argument& e)
  {
    CRAS_ERROR_NAMED("exiftool", "Error reading image metadata: %s", e.what());
  }

  return cras::nullopt;
}

cras::optional<ExifData<ExifSRational>> ExiftoolMetadataExtractor::getExifPitchAngle()
{
  this->data->loadExifData();
  if (!this->data->exifData.has_value())
    return cras::nullopt;
  const auto& exifData = *this->data->exifData;

  const auto& it = exifData.find("*.PitchAngle");
  if (it == exifData.end())
    return cras::nullopt;

  try
  {
    // Pitch is positive upward in exiftool, so negate it
    return ExifData<ExifSRational>{fullKeyName(it->second), -cras::parseDouble(it->second->value) * M_PI / 180.0};
  }
  catch (const std::invalid_argument& e)
  {
    CRAS_ERROR_NAMED("exiftool", "Error reading image metadata: %s", e.what());
  }

  return cras::nullopt;
}

MetadataExtractor::Ptr ExiftoolMetadataExtractorPlugin::getExtractor(const MetadataExtractorParams& params)
{
  if (params.log == nullptr || params.info->filenameOrURL().empty() ||
      params.info->width() == 0 || params.info->height() == 0)
    return nullptr;

  return std::make_shared<ExiftoolMetadataExtractor>(
    params.log, params.manager, params.info->filenameOrURL(), params.info->width(), params.info->height());
}

}

PLUGINLIB_EXPORT_CLASS(movie_publisher::ExiftoolMetadataExtractorPlugin, movie_publisher::MetadataExtractorPlugin)
