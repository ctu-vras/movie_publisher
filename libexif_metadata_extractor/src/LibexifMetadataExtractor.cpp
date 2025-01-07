// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Metadata extractor using libexif backend.
 * \author Martin Pecka
 */

#include "LibexifMetadataExtractor.h"

#include <cctype>
#include <limits>

#include <libexif/exif-loader.h>
#include <libexif/exif-mnote-data.h>

#include <cras_cpp_common/type_utils.hpp>
#include <pluginlib/class_list_macros.h>

#include "LibexifCustomMakernotes.h"

struct _ExifDataPrivate
{
  ExifByteOrder order;

  ExifMnoteData *md;

  ExifLog *log;
  ExifMem *mem;

  unsigned int ref_count;

  /* Temporarily used while loading data */
  unsigned int offset_mnote;

  ExifDataOption options;
  ExifDataType data_type;
};

namespace movie_publisher
{

static constexpr auto EXIF_TAG_OFFSET_TIME_ORIGINAL {static_cast<ExifTag>(0x9011)};
static constexpr auto EXIF_TAG_SERIAL_NUMBER {static_cast<ExifTag>(0xa431)};
static constexpr auto EXIF_TAG_LENS_MAKE {static_cast<ExifTag>(0xa433)};
static constexpr auto EXIF_TAG_LENS_MODEL {static_cast<ExifTag>(0xa434)};
static constexpr auto EXIF_TAG_LENS_SERIAL_NUMBER {static_cast<ExifTag>(0xa435)};

#define EXIF_TAG_GPS_HPOS_ERROR 0x001f
#define APPLE_MAKERNOTE_ACCELERATION 0x08
#define PANASONIC_MAKERNOTE_INTERNAL_SERIAL_NUMBER 0x25
#define PANASONIC_MAKERNOTE_LENS_MODEL 0x51
#define PANASONIC_MAKERNOTE_LENS_SERIAL_NUMBER 0x52
#define PANASONIC_MAKERNOTE_ROLL_ANGLE 0x90
#define PANASONIC_MAKERNOTE_PITCH_ANGLE 0x91
#define PANASONIC_MAKERNOTE_ACCELEROMETER_Z 0x8c
#define PANASONIC_MAKERNOTE_ACCELEROMETER_X 0x8d
#define PANASONIC_MAKERNOTE_ACCELEROMETER_Y 0x8e

extern "C" {
extern int exif_mnote_data_apple_identify(const ::ExifData *, const ::ExifEntry *);
extern ExifMnoteData *exif_mnote_data_apple_new(::ExifMem *);
}

double q_to_float(const ::ExifRational& rational)
{
  if (rational.denominator == 0)
    return 0.0;
  return static_cast<double>(rational.numerator) / rational.denominator;
}

double q_to_float(const ::ExifSRational& rational)
{
  if (rational.denominator == 0)
    return 0.0;
  return static_cast<double>(rational.numerator) / rational.denominator;
}

std::string getIfdName(const ::ExifIfd ifd)
{
  switch (ifd)
  {
    case EXIF_IFD_0:
      return "Image";
    case EXIF_IFD_1:
      return "Photo";
    default:
      const auto name = exif_ifd_get_name(ifd);
      return name != nullptr ? name : "";
  }
}

template<typename ExifType>
inline cras::optional<movie_publisher::ExifData<ExifType>> getExifValue(
  const std::function<ExifType(unsigned char*, ::ExifByteOrder)>& convertValue, const ::ExifFormat exifFormat,
  ::ExifData* exifData, const ::ExifIfd ifd, const ::ExifTag& key, const std::string& textKey, const long n = 0)  // NOLINT
{
  const auto entry = exif_content_get_entry(exifData->ifd[ifd], key);
  if (entry == nullptr)
    return cras::nullopt;
  const auto componentSize = exif_format_get_size(entry->format);
  if (entry->format != exifFormat || n >= entry->components || componentSize * n > entry->size)
    return cras::nullopt;
  const auto tagName = exif_tag_get_name_in_ifd(key, ifd);
  const auto ifdName = getIfdName(ifd);
  std::string keyName = !ifdName.empty() ? ifdName + "::" : "";
  keyName += tagName != nullptr && strlen(tagName) > 0 ? tagName : textKey;
  const auto byteOrder = exif_data_get_byte_order(exifData);
  return movie_publisher::ExifData<ExifType>{keyName, convertValue(entry->data + n * componentSize, byteOrder)};
}

inline cras::optional<movie_publisher::ExifData<movie_publisher::ExifAscii>> getExifString(
  ::ExifData* exifData, const ::ExifIfd ifd, const ::ExifTag& key, const std::string& textKey, const long n = 0)  // NOLINT
{
  const auto convert = [](const unsigned char* data, ::ExifByteOrder)
  {
    return cras::strip(reinterpret_cast<const char*>(data));
  };
  return getExifValue<movie_publisher::ExifAscii>(convert, EXIF_FORMAT_ASCII, exifData, ifd, key, textKey, n);
}

inline cras::optional<movie_publisher::ExifData<movie_publisher::ExifLong>> getExifLong(
  ::ExifData* exifData, const ::ExifIfd ifd, const ::ExifTag& key, const std::string& textKey, const long n = 0)  // NOLINT
{
  return getExifValue<movie_publisher::ExifLong>(&exif_get_long, EXIF_FORMAT_LONG, exifData, ifd, key, textKey, n);
}

inline cras::optional<movie_publisher::ExifData<movie_publisher::ExifShort>> getExifShort(
  ::ExifData* exifData, const ::ExifIfd ifd, const ::ExifTag& key, const std::string& textKey, const long n = 0)  // NOLINT
{
  return getExifValue<movie_publisher::ExifShort>(&exif_get_short, EXIF_FORMAT_SHORT, exifData, ifd, key, textKey, n);
}

inline cras::optional<movie_publisher::ExifData<movie_publisher::ExifByte>> getExifByte(
  ::ExifData* exifData, const ::ExifIfd ifd, const ::ExifTag& key, const std::string& textKey, const long n = 0)  // NOLINT
{
  const auto convert = [](const unsigned char* data, ::ExifByteOrder) {return data[0];};
  return getExifValue<movie_publisher::ExifByte>(convert, EXIF_FORMAT_BYTE, exifData, ifd, key, textKey, n);
}

inline cras::optional<movie_publisher::ExifData<movie_publisher::ExifRational>> getExifRational(
  ::ExifData* exifData, const ::ExifIfd ifd, const ::ExifTag& key, const std::string& textKey, const long n = 0)  // NOLINT
{
  const auto convert = [](const unsigned char* data, const ::ExifByteOrder bo)
  {
    return q_to_float(exif_get_rational(data, bo));
  };
  return getExifValue<movie_publisher::ExifRational>(convert, EXIF_FORMAT_RATIONAL, exifData, ifd, key, textKey, n);
}

inline cras::optional<int16_t> getExifInt16FromString(const std::string& valStr)
{
  try
  {
    const auto val = cras::parseUInt16(valStr, 10);
    // We want to allow both standard int16 values and int16 saved as uint16 values
    if (val < std::numeric_limits<int16_t>::min() || val > std::numeric_limits<uint16_t>::max())
      return cras::nullopt;
    return static_cast<int16_t>(val);
  }
  catch (const std::invalid_argument&) {}

  return cras::nullopt;
}

struct LibexifMetadataPrivate : cras::HasLogger
{
  std::string filename;

  ::ExifMem* loaderMemPtr {nullptr};
  ::ExifLoader* loaderPtr {nullptr};
  cras::optional<::ExifLoader*> loader;
  cras::optional<::ExifData*> exifData;
  std::unordered_map<unsigned int, std::string> makerNotes;

  explicit LibexifMetadataPrivate(const cras::LogHelperPtr& log) : cras::HasLogger(log) {}
  ~LibexifMetadataPrivate()
  {
    if (this->loaderPtr != nullptr)
    {
      this->loader.reset();
      exif_loader_unref(this->loaderPtr);
    }
    exif_mem_unref(this->loaderMemPtr);
  }

  ::ExifLoader* getLoader()
  {
    if (this->loader.has_value())
      return *this->loader;

    this->loaderMemPtr = exif_mem_new_default();
    this->loaderPtr = exif_loader_new_mem(this->loaderMemPtr);
    this->loader = this->loaderPtr;
    if (this->loaderPtr == nullptr)
      CRAS_ERROR_NAMED("libexif", "Error creating libexif loader.");

    return *this->loader;
  }

  ::ExifData* getExifData()
  {
    if (this->exifData.has_value())
      return *this->exifData;

    const auto loader = this->getLoader();
    if (loader == nullptr)
      return nullptr;

    cras::TempLocale l(LC_ALL, "en_US.UTF-8");
    CRAS_DEBUG_NAMED("libexif", "libexif: Loading file %s .", this->filename.c_str());
    exif_loader_write_file(loader, this->filename.c_str());  // Weird name, this actually loads the file

    const unsigned char* buf {nullptr};
    unsigned int buf_size;
    exif_loader_get_buf(loader, &buf, &buf_size);

    if (buf == nullptr || buf_size == 0)
      return this->exifData.emplace(nullptr);

    this->exifData = exif_data_new_mem(this->loaderMemPtr);
    exif_data_unset_option(*this->exifData, EXIF_DATA_OPTION_IGNORE_UNKNOWN_TAGS);
    exif_data_unset_option(*this->exifData, EXIF_DATA_OPTION_FOLLOW_SPECIFICATION);
    exif_data_load_data(*this->exifData, buf, buf_size);

    const auto& e = *this->exifData.value();
    char value[2048];
    for (auto ifd = EXIF_IFD_0; ifd < EXIF_IFD_COUNT; ifd = static_cast<decltype(ifd)>(ifd + 1))
    {
      for (size_t i = 0; i < e.ifd[ifd]->count; ++i)
      {
        const auto& entry = e.ifd[ifd]->entries[i];
        const auto tagNameC = exif_tag_get_name_in_ifd(entry->tag, ifd);
        const std::string tagName = tagNameC != nullptr ? tagNameC : cras::format("0x%0x", entry->tag);
        exif_entry_get_value(entry, value, sizeof(value));
        const auto& format = exif_format_get_name(entry->format);
        if (entry->format == EXIF_FORMAT_UNDEFINED)
        {
          std::stringstream ss;
          for (size_t n = 0; n < entry->components; n++)
            ss << cras::format("%02x", entry->data[n]);
          CRAS_DEBUG_NAMED("libexif.dump",
            "libexif IFD[%d].%s : 0x%s [%s]", ifd, tagName.c_str(), ss.str().c_str(), format);
        }
        else
        {
          CRAS_DEBUG_NAMED("libexif.dump", "libexif IFD[%d].%s : %s [%s]", ifd, tagName.c_str(), value, format);
        }
      }
    }

    // Try loading default maker notes
    auto mn = exif_data_get_mnote_data(*this->exifData);

    // If not found, try the custom ones
    if (mn == nullptr)
    {
      const auto mnEntry = exif_data_get_entry((*this->exifData), EXIF_TAG_MAKER_NOTE);
      if (mnEntry != nullptr)
      {
        for (const auto& [make, notes] : LibexifCustomMakernotes().customMakerNotes)
        {
          if (notes.identify(*this->exifData, mnEntry) != 0)
          {
            mn = notes.create(this->loaderMemPtr);
            if (mn != nullptr)
            {
              exif_mnote_data_set_byte_order(mn, exif_data_get_byte_order(*this->exifData));
              exif_mnote_data_set_offset(mn, e.priv->offset_mnote);
              exif_mnote_data_load(mn, buf, buf_size);
            }
          }
        }
      }
    }

    // If some maker notes were found, extract them
    if (mn != nullptr)
    {
      const auto num = exif_mnote_data_count(mn);
      for (size_t i = 0; i < num; ++i)
      {
        if (exif_mnote_data_get_value(mn, i, value, sizeof(value)) != nullptr)
        {
          const auto id = exif_mnote_data_get_id(mn, i);
          this->makerNotes[id] = value;
          CRAS_DEBUG_NAMED("libexif.dump", "libexif MakerNote.0x%0x : %s", id, value);
        }
      }
      exif_mnote_data_unref(mn);  // we no longer need the makernote
    }

    return *this->exifData;
  }
};

LibexifMetadataExtractor::LibexifMetadataExtractor(
  const cras::LogHelperPtr& log, const std::weak_ptr<MetadataManager>& manager,
  const std::string& filename, const size_t width, const size_t height)
  : ExifBaseMetadataExtractor(log, manager, width, height), data(new LibexifMetadataPrivate(log))
{
  this->data->filename = filename;
}

LibexifMetadataExtractor::~LibexifMetadataExtractor() = default;

std::string LibexifMetadataExtractor::getName() const
{
  return cras::getTypeName<std::remove_cv_t<std::remove_reference_t<decltype(*this)>>>();
}

int LibexifMetadataExtractor::getPriority() const
{
  return 50;
}

#define RETURN(ifd, tag, getValue, n) \
  const auto exifData = this->data->getExifData();\
  if (exifData == nullptr)\
    return cras::nullopt;\
  return getValue(exifData, ifd, static_cast<::ExifTag>(tag), #tag, n);\

#define RETURN_STRING(ifd, tag) RETURN(ifd, tag, getExifString, 0)
#define RETURN_LONG(ifd, tag) RETURN(ifd, tag, getExifLong, 0)
#define RETURN_SHORT(ifd, tag) RETURN(ifd, tag, getExifShort, 0)
#define RETURN_BYTE(ifd, tag) RETURN(ifd, tag, getExifByte, 0)
#define RETURN_RATIONAL_N(ifd, tag, n) RETURN(ifd, tag, getExifRational, n)
#define RETURN_RATIONAL(ifd, tag) RETURN_RATIONAL_N(ifd, tag, 0)

cras::optional<movie_publisher::ExifData<movie_publisher::ExifAscii>> LibexifMetadataExtractor::getExifMake()
{
  RETURN_STRING(EXIF_IFD_0, EXIF_TAG_MAKE);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifAscii>> LibexifMetadataExtractor::getExifModel()
{
  RETURN_STRING(EXIF_IFD_0, EXIF_TAG_MODEL);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifAscii>> LibexifMetadataExtractor::getExifLensMake()
{
  RETURN_STRING(EXIF_IFD_EXIF, EXIF_TAG_LENS_MAKE);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifAscii>> LibexifMetadataExtractor::getExifLensModel()
{
  const auto exifData = this->data->getExifData();
  if (exifData == nullptr)
    return cras::nullopt;

  auto result = getExifString(exifData, EXIF_IFD_EXIF, EXIF_TAG_LENS_MODEL, "EXIF_TAG_LENS_MODEL");
  if (result.has_value())
    return result;

  const auto& mn = this->data->makerNotes;

  if (this->getExifMake().value_or(movie_publisher::ExifData<movie_publisher::ExifAscii>{}).value == "Panasonic")
  {
    if (mn.find(PANASONIC_MAKERNOTE_INTERNAL_SERIAL_NUMBER) != mn.end())
      return movie_publisher::ExifData<movie_publisher::ExifAscii>{
        "MakerNote::LensModel", cras::strip(mn.at(PANASONIC_MAKERNOTE_LENS_MODEL))};
  }

  return cras::nullopt;
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifAscii>>
LibexifMetadataExtractor::getExifBodySerialNumber()
{
  const auto exifData = this->data->getExifData();
  if (exifData == nullptr)
    return cras::nullopt;

  auto result = getExifString(exifData, EXIF_IFD_EXIF, EXIF_TAG_SERIAL_NUMBER, "EXIF_TAG_SERIAL_NUMBER");
  if (result.has_value())
    return result;

  auto& mn = this->data->makerNotes;

  if (this->getExifMake().value_or(movie_publisher::ExifData<movie_publisher::ExifAscii>{}).value == "Panasonic")
  {
    if (mn.find(PANASONIC_MAKERNOTE_INTERNAL_SERIAL_NUMBER) != mn.end())
    {
      auto bodySerialHexStr = this->data->makerNotes[PANASONIC_MAKERNOTE_INTERNAL_SERIAL_NUMBER];
      if (bodySerialHexStr.size() > 4 && bodySerialHexStr.size() % 2 == 0 && cras::startsWith(bodySerialHexStr, "0x"))
      {
        std::stringstream ss;
        bodySerialHexStr = bodySerialHexStr.substr(2);
        for (size_t i = 0; !bodySerialHexStr.empty(); ++i)
        {
          const char c = cras::parseInt8(bodySerialHexStr.substr(0, 2), 16);
          if (std::isprint(c))
            ss << c;
          bodySerialHexStr = bodySerialHexStr.substr(2);
        }
        return movie_publisher::ExifData<movie_publisher::ExifAscii>{"MakerNote::InternalSerialNumber", ss.str()};
      }
    }
  }

  return cras::nullopt;
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifAscii>>
LibexifMetadataExtractor::getExifLensSerialNumber()
{
  const auto exifData = this->data->getExifData();
  if (exifData == nullptr)
    return cras::nullopt;

  auto result = getExifString(exifData, EXIF_IFD_EXIF, EXIF_TAG_LENS_SERIAL_NUMBER, "EXIF_TAG_LENS_SERIAL_NUMBER");
  if (result.has_value())
    return result;

  auto& mn = this->data->makerNotes;

  if (this->getExifMake().value_or(movie_publisher::ExifData<movie_publisher::ExifAscii>{}).value == "Panasonic")
  {
    if (mn.find(PANASONIC_MAKERNOTE_LENS_SERIAL_NUMBER) != mn.end())
    {
      auto lensSerialStr = this->data->makerNotes[PANASONIC_MAKERNOTE_LENS_SERIAL_NUMBER];
      if (!lensSerialStr.empty())
        return movie_publisher::ExifData<movie_publisher::ExifAscii>{"MakerNote::LensSerialNumber", lensSerialStr};
    }
  }

  return cras::nullopt;
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifAscii>>
LibexifMetadataExtractor::getExifDateTimeOriginal()
{
  RETURN_STRING(EXIF_IFD_EXIF, EXIF_TAG_DATE_TIME_ORIGINAL);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifAscii>>
LibexifMetadataExtractor::getExifOffsetTimeOriginal()
{
  RETURN_STRING(EXIF_IFD_EXIF, EXIF_TAG_OFFSET_TIME_ORIGINAL);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifAscii>>
LibexifMetadataExtractor::getExifSubSecTimeOriginal()
{
  RETURN_STRING(EXIF_IFD_EXIF, EXIF_TAG_SUB_SEC_TIME_ORIGINAL);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifShort>> LibexifMetadataExtractor::getExifOrientation()
{
  RETURN_SHORT(EXIF_IFD_0, EXIF_TAG_ORIENTATION);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifRational>>
LibexifMetadataExtractor::getExifFocalPlaneXRes()
{
  RETURN_RATIONAL(EXIF_IFD_EXIF, EXIF_TAG_FOCAL_PLANE_X_RESOLUTION);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifRational>>
LibexifMetadataExtractor::getExifFocalPlaneYRes()
{
  RETURN_RATIONAL(EXIF_IFD_EXIF, EXIF_TAG_FOCAL_PLANE_Y_RESOLUTION);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifShort>>
LibexifMetadataExtractor::getExifFocalPlaneResUnit()
{
  RETURN_SHORT(EXIF_IFD_EXIF, EXIF_TAG_FOCAL_PLANE_RESOLUTION_UNIT);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifShort>> LibexifMetadataExtractor::getExifResUnit()
{
  RETURN_SHORT(EXIF_IFD_EXIF, EXIF_TAG_RESOLUTION_UNIT);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifShort>> LibexifMetadataExtractor::getExifFocalLength35MM()
{
  RETURN_SHORT(EXIF_IFD_EXIF, EXIF_TAG_FOCAL_LENGTH_IN_35MM_FILM);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifRational>> LibexifMetadataExtractor::getExifFocalLength()
{
  RETURN_RATIONAL(EXIF_IFD_EXIF, EXIF_TAG_FOCAL_LENGTH);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifAscii>> LibexifMetadataExtractor::getExifGpsLatRef()
{
  RETURN_STRING(EXIF_IFD_GPS, EXIF_TAG_GPS_LATITUDE_REF);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifRational>>
LibexifMetadataExtractor::getExifGpsLat(const size_t n)
{
  RETURN_RATIONAL_N(EXIF_IFD_GPS, EXIF_TAG_GPS_LATITUDE, n);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifAscii>> LibexifMetadataExtractor::getExifGpsLonRef()
{
  RETURN_STRING(EXIF_IFD_GPS, EXIF_TAG_GPS_LONGITUDE_REF);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifRational>>
LibexifMetadataExtractor::getExifGpsLon(const size_t n)
{
  RETURN_RATIONAL_N(EXIF_IFD_GPS, EXIF_TAG_GPS_LONGITUDE, n);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifByte>> LibexifMetadataExtractor::getExifGpsAltRef()
{
  RETURN_BYTE(EXIF_IFD_GPS, EXIF_TAG_GPS_ALTITUDE_REF);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifRational>> LibexifMetadataExtractor::getExifGpsAlt()
{
  RETURN_RATIONAL(EXIF_IFD_GPS, EXIF_TAG_GPS_ALTITUDE);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifAscii>> LibexifMetadataExtractor::getExifGpsMeasureMode()
{
  RETURN_STRING(EXIF_IFD_GPS, EXIF_TAG_GPS_MEASURE_MODE);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifRational>> LibexifMetadataExtractor::getExifGpsDOP()
{
  RETURN_RATIONAL(EXIF_IFD_GPS, EXIF_TAG_GPS_DOP);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifAscii>> LibexifMetadataExtractor::getExifGpsSpeedRef()
{
  RETURN_STRING(EXIF_IFD_GPS, EXIF_TAG_GPS_SPEED_REF);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifRational>> LibexifMetadataExtractor::getExifGpsSpeed()
{
  RETURN_RATIONAL(EXIF_IFD_GPS, EXIF_TAG_GPS_SPEED);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifAscii>> LibexifMetadataExtractor::getExifGpsTrackRef()
{
  RETURN_STRING(EXIF_IFD_GPS, EXIF_TAG_GPS_TRACK_REF);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifRational>> LibexifMetadataExtractor::getExifGpsTrack()
{
  RETURN_RATIONAL(EXIF_IFD_GPS, EXIF_TAG_GPS_TRACK);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifRational>>
LibexifMetadataExtractor::getExifGpsTimeStamp(const size_t n)
{
  RETURN_RATIONAL_N(EXIF_IFD_GPS, EXIF_TAG_GPS_TIME_STAMP, n);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifAscii>> LibexifMetadataExtractor::getExifGpsDateStamp()
{
  RETURN_STRING(EXIF_IFD_GPS, EXIF_TAG_GPS_DATE_STAMP);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifShort>> LibexifMetadataExtractor::getExifGpsDifferential()
{
  RETURN_SHORT(EXIF_IFD_GPS, EXIF_TAG_GPS_DIFFERENTIAL);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifRational>>
LibexifMetadataExtractor::getExifGpsHPositioningError()
{
  RETURN_RATIONAL(EXIF_IFD_GPS, EXIF_TAG_GPS_HPOS_ERROR);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifAscii>>
LibexifMetadataExtractor::getExifGpsImgDirectionRef()
{
  RETURN_STRING(EXIF_IFD_GPS, EXIF_TAG_GPS_IMG_DIRECTION_REF);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifRational>>
LibexifMetadataExtractor::getExifGpsImgDirection()
{
  RETURN_RATIONAL(EXIF_IFD_GPS, EXIF_TAG_GPS_IMG_DIRECTION);
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifSRational>>
LibexifMetadataExtractor::getExifAcceleration(const size_t n)
{
  const auto exifData = this->data->getExifData();
  if (exifData == nullptr)
    return cras::nullopt;

  if (n > 2)
    return cras::nullopt;

  const auto& mnEnd = this->data->makerNotes.end();

  if (this->getExifMake().value_or(movie_publisher::ExifData<movie_publisher::ExifAscii>{}).value == "Apple")
  {
    if (this->data->makerNotes.find(APPLE_MAKERNOTE_ACCELERATION) != mnEnd)
    {
      const auto appleIosAccelerationStr = this->data->makerNotes[APPLE_MAKERNOTE_ACCELERATION];
      const auto parts = cras::split(cras::strip(appleIosAccelerationStr), " ", 3);
      if (parts.size() == 3)
      {
        try
        {
          const auto accScale = 9.8;  // Apple seems to normalize this value somehow
          // Apple has X left, Y down, Z towards user when the phone is upright, which is exactly what we want.
          return movie_publisher::ExifData<movie_publisher::ExifSRational>{
            "MakerNote::AppleIosAccelerationVector", cras::parseDouble(parts[n]) * accScale};
        }
        catch (const std::invalid_argument&) {}
      }
    }
  }

  if (this->getExifMake().value_or(movie_publisher::ExifData<movie_publisher::ExifAscii>{}).value == "Panasonic")
  {
    const auto accScale = 0.034795;  // This is a magic number that seems to fit the samples I saw
    int key;
    std::string keyName;
    double scale;
    // Panasonic has X left, Y towards user, Z up
    switch (n)
    {
      case 0:
        key = PANASONIC_MAKERNOTE_ACCELEROMETER_Y;
        keyName = "MakerNote::AccelerometerY";
        scale = -accScale;
        break;
      case 1:
        key = PANASONIC_MAKERNOTE_ACCELEROMETER_X;
        keyName = "MakerNote::AccelerometerX";
        scale = accScale;
        break;
      case 2:
        key = PANASONIC_MAKERNOTE_ACCELEROMETER_Z;
        keyName = "MakerNote::AccelerometerZ";
        scale = accScale;
        break;
    }

    const auto accIt = this->data->makerNotes.find(key);
    if (accIt != mnEnd)
    {
      const auto panaAcc =  getExifInt16FromString(accIt->second);
      if (panaAcc.has_value())
        return movie_publisher::ExifData<movie_publisher::ExifSRational>{keyName, scale * *panaAcc};
    }
  }

  return cras::nullopt;
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifSRational>> LibexifMetadataExtractor::getExifRollAngle()
{
  if (this->getExifMake().value_or(movie_publisher::ExifData<movie_publisher::ExifAscii>{}).value == "Panasonic")
  {
    const auto rollIt = this->data->makerNotes.find(PANASONIC_MAKERNOTE_ROLL_ANGLE);
    if (rollIt != this->data->makerNotes.end())
    {
      const auto rollInt = getExifInt16FromString(rollIt->second);
      if (rollInt.has_value())
        return movie_publisher::ExifData<movie_publisher::ExifSRational>{
          "MakerNote::RollAngle", (*rollInt / 10.0) / 180.0 * M_PI};
    }
  }

  return cras::nullopt;
}

cras::optional<movie_publisher::ExifData<movie_publisher::ExifSRational>> LibexifMetadataExtractor::getExifPitchAngle()
{
  if (this->getExifMake().value_or(movie_publisher::ExifData<movie_publisher::ExifAscii>{}).value == "Panasonic")
  {
    const auto pitchIt = this->data->makerNotes.find(PANASONIC_MAKERNOTE_PITCH_ANGLE);
    if (pitchIt != this->data->makerNotes.end())
    {
      const auto pitchInt = getExifInt16FromString(pitchIt->second);
      if (pitchInt.has_value())
        return movie_publisher::ExifData<movie_publisher::ExifSRational>{
          "MakerNote::PitchAngle", (*pitchInt / 10.0) / 180.0 * M_PI};
    }
  }

  return cras::nullopt;
}

MetadataExtractor::Ptr LibexifMetadataExtractorPlugin::getExtractor(const MetadataExtractorParams& params)
{
  if (params.log == nullptr || params.filename.empty() || params.width == 0 || params.height == 0)
    return nullptr;

  return std::make_shared<LibexifMetadataExtractor>(
    params.log, params.manager, params.filename, params.width, params.height);
}

}

PLUGINLIB_EXPORT_CLASS(movie_publisher::LibexifMetadataExtractorPlugin, movie_publisher::MetadataExtractorPlugin)
