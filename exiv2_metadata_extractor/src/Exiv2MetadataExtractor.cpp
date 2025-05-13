// SPDX-License-Identifier: GPL-2.0-or-later
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Metadata extractor using exiv2 backend.
 * \author Martin Pecka
 */


#include "Exiv2MetadataExtractor.h"

#include <limits>

#include <exiv2/exiv2.hpp>
#include <sys/stat.h>

#include "Exiv2CustomMakernotes.h"

#include <cras_cpp_common/type_utils.hpp>
#include <pluginlib/class_list_macros.h>

namespace movie_publisher
{

inline cras::optional<long int> getExifLong(  // NOLINT
  const Exiv2::ExifData& exifData, const Exiv2::ExifKey& key, const long n = 0)  // NOLINT
{
  const auto it = exifData.findKey(key);
  return (it == exifData.end() || n >= it->count()) ? cras::nullopt : cras::optional{it->value().toLong(n)};
}

inline cras::optional<int16_t> getExifInt16(
  const Exiv2::ExifData& exifData, const Exiv2::ExifKey& key, const long n = 0)  // NOLINT
{
  const auto val = getExifLong(exifData, key, n);
  // We want to allow both standard int16 values and int16 saved as uint16 values
  if (!val || *val < std::numeric_limits<int16_t>::min() || *val > std::numeric_limits<uint16_t>::max())
    return cras::nullopt;
  return static_cast<int16_t>(*val);
}

inline cras::optional<double> getExifRational(
  const Exiv2::ExifData& exifData, const Exiv2::ExifKey& key, const long n = 0)  // NOLINT
{
  const auto it = exifData.findKey(key);
  return (it == exifData.end() || n >= it->count()) ? cras::nullopt : cras::optional{it->value().toFloat(n)};
}

inline std::ostream& printValue(std::ostream& os, const Exiv2::Value& value, const Exiv2::ExifData*)
{
  return os << value;
}

inline std::ostream& printFloat(std::ostream& os, const Exiv2::Value& value, const Exiv2::ExifData*) {
  Exiv2::Rational r = value.toRational();
  if (r.second != 0) {
    os << value.toFloat();
  } else {
    os << "(" << value << ")";
  }
  return os;
}

/**
 * \brief Private data.
 */
struct Exiv2MetadataPrivate : cras::HasLogger
{
  std::string filename;  //!< Filename of the movie.

  Exiv2::Image::AutoPtr imagePtr;  //!< Instance of the parsed metadata.
  cras::optional<Exiv2::Image*> image;  //!< Raw pointer to the parsed metadata.
  cras::optional<Exiv2::ExifData*> exifData;  //!< Processed metadata.

  std::unique_ptr<CustomMakernotes> makernotes;  //!< Custom MakerNotes.

  bool metadataRead {false};  //!< Whether metadata have already been read.

  explicit Exiv2MetadataPrivate(const cras::LogHelperPtr& log) : cras::HasLogger(log) {}

  Exiv2::Image* getImage()
  {
    if (this->image.has_value())
      return *this->image;

    cras::TempLocale l(LC_ALL, "en_US.UTF-8");
    CRAS_DEBUG_NAMED("exiv2", "exiv2: Loading file %s .", this->filename.c_str());
    try
    {
      this->imagePtr = Exiv2::ImageFactory::open(this->filename);
      this->image = this->imagePtr.get();
    }
    catch (const Exiv2::AnyError& e)
    {
      CRAS_ERROR_NAMED("exiv2", "Error reading image metadata: %s", e.what());
      this->image = nullptr;
    }

    return *this->image;
  }

  Exiv2::ExifData* getExifData()
  {
    if (this->exifData.has_value())
      return *this->exifData;

    const auto image = this->getImage();
    if (image == nullptr)
      return nullptr;

    try
    {
      if (!this->metadataRead)
      {
        image->readMetadata();
        this->metadataRead = true;
        this->exifData = &image->exifData();
        this->readCustomMakerNotes();
      }

      const auto& e = *this->exifData.value();
      for (const auto& exif : e)
        CRAS_DEBUG_NAMED("exiv2.dump", "exiv2 %s : %s", exif.key().c_str(), exif.value().toString().c_str());
    }
    catch (const Exiv2::AnyError& e)
    {
      CRAS_ERROR_NAMED("exiv2", "Error reading EXIF data: %s", e.what());
      this->exifData = nullptr;
    }

    return *this->exifData;
  }

  void readCustomMakerNotes()
  {
    if (!this->exifData.has_value() || *this->exifData == nullptr)
      return;

    try
    {
      auto& exif = *this->exifData.value();

      const auto& makerNote = exif.findKey(Exiv2::ExifKey("Exif.Photo.MakerNote"));
      if (makerNote == exif.end())
        return;

      const auto makeIt = Exiv2::make(exif);
      if (makeIt == exif.end())
        return;

      const auto& make = makeIt->value().toString();
      if (make.empty())
        return;

      this->makernotes = std::make_unique<CustomMakernotes>();

      const auto& makerNoteValue = makerNote->value();
      if (this->makernotes->headers.find(make) == this->makernotes->headers.end())
        return;

      if (this->makernotes->tagsById.find(make) == this->makernotes->tagsById.end())
        return;
      const auto& exifKeys = this->makernotes->tagsById.at(make);

      std::vector<uint8_t> buf(makerNoteValue.size());
      makerNoteValue.copy(buf.data(), Exiv2::ByteOrder::littleEndian);

      auto header = this->makernotes->headers.at(make);
      if (!header->read(buf.data(), buf.size()))
        return;

      const auto* dataStart = buf.data() + header->size();
      auto* data = dataStart;

      const Exiv2::UShortValue numEntries(data, 2, header->byteOrder());
      data += 2;

      for (size_t i = 0; i < numEntries.toLong(); ++i)
      {
        Exiv2::UShortValue tagIdVal(data, 2, header->byteOrder());
        data += 2;
        Exiv2::UShortValue dataFormatIdVal(data, 2, header->byteOrder());
        data += 2;
        Exiv2::ULongValue countVal(data, 4, header->byteOrder());
        data += 4;
        Exiv2::ULongValue valueOrOffsetVal(data, 4, header->byteOrder());
        data += 4;

        const auto& tagId = tagIdVal.value_[0];
        const auto& type = static_cast<Exiv2::TypeId>(dataFormatIdVal.value_[0]);
        const auto& count = countVal.value_[0];
        const auto componentSize = Exiv2::TypeInfo::typeSize(type);
        const auto numBytes = componentSize * count;

        const auto val = Exiv2::Value::create(type);
        if (numBytes <= 4)
          val->read(header->byteOrder() == Exiv2::littleEndian ?
            data - numBytes : data - 4, numBytes, header->byteOrder());
        else
          val->read(dataStart + valueOrOffsetVal.value_[0], numBytes, header->byteOrder());

        if (exifKeys.find(tagId) == exifKeys.end())
          continue;

        const auto& tagInfo = exifKeys.at(tagId);
        if (tagInfo.typeId_ != type)
          CRAS_WARN_NAMED("exiv2", "Tag %s was expected to have type %u but the provided one has type %u.",
            tagInfo.name_, tagInfo.typeId_, type);

        Exiv2::ExifKey key(tagInfo);
        exif.add(key, val.get());
        CRAS_DEBUG_NAMED("exiv2", "Decoded custom makernote: %s=%s", key.key().c_str(), val->toString().c_str());
      }
    }
    catch (const Exiv2::AnyError& e)
    {
      CRAS_ERROR_NAMED("exiv2", "Error reading image metadata: %s", e.what());
    }
  }
};

Exiv2MetadataExtractor::Exiv2MetadataExtractor(
  const cras::LogHelperPtr& log, const std::weak_ptr<MetadataManager>& manager,
  const std::string& filename, const size_t width, const size_t height)
  : ExifBaseMetadataExtractor(log, manager, width, height), data(new Exiv2MetadataPrivate(log))
{
  this->data->filename = filename;
}

Exiv2MetadataExtractor::~Exiv2MetadataExtractor() = default;

std::string Exiv2MetadataExtractor::getName() const
{
  return cras::getTypeName<std::remove_cv_t<std::remove_reference_t<decltype(*this)>>>();
}

int Exiv2MetadataExtractor::getPriority() const
{
  return 45;
}

#define RETURN(expr, ReturnType, getValue) \
  try \
  { \
    const auto exifData = this->data->getExifData(); \
    if (exifData == nullptr) \
      return cras::nullopt; \
    const auto it = (expr); \
    if (it == exifData->end()) \
      return cras::nullopt; \
    return ExifData<ReturnType>{it->key(), getValue}; \
  } \
  catch (const Exiv2::AnyError& exc) \
  { \
    CRAS_ERROR_NAMED("exiv2", "Error reading image metadata: %s", exc.what()); \
    return cras::nullopt; \
  }

#define RETURN_STRING(expr) RETURN((expr), ExifAscii, cras::strip(it->value().toString()))
#define RETURN_LONG(expr) RETURN((expr), ExifLong, it->value().toLong())
#define RETURN_SHORT(expr) RETURN((expr), ExifShort, static_cast<ExifShort>(it->value().toLong()))
#define RETURN_BYTE(expr) RETURN((expr), ExifByte, static_cast<ExifByte>(it->value().toLong()))
#define RETURN_RATIONAL_N(expr, n) RETURN((expr), ExifRational, static_cast<ExifRational>(it->value().toFloat(n)))
#define RETURN_RATIONAL(expr) RETURN_RATIONAL_N((expr), 0)

cras::optional<ExifData<ExifAscii>> Exiv2MetadataExtractor::getExifMake()
{
  RETURN_STRING(Exiv2::make(*exifData));
}

cras::optional<ExifData<ExifAscii>> Exiv2MetadataExtractor::getExifModel()
{
  RETURN_STRING(Exiv2::model(*exifData));
}

cras::optional<ExifData<ExifAscii>> Exiv2MetadataExtractor::getExifLensMake()
{
  RETURN_STRING(exifData->findKey(Exiv2::ExifKey("Exif.Photo.LensMake")));
}

cras::optional<ExifData<ExifAscii>> Exiv2MetadataExtractor::getExifLensModel()
{
  RETURN_STRING(Exiv2::lensName(*exifData));
}

cras::optional<ExifData<ExifAscii>> Exiv2MetadataExtractor::getExifBodySerialNumber()
{
  // TODO: Panasonic has it in an unparsed MakerNote
  RETURN_STRING(Exiv2::serialNumber(*exifData));
}

cras::optional<ExifData<ExifAscii>> Exiv2MetadataExtractor::getExifDateTimeOriginal()
{
#if EXIV2_TEST_VERSION(0, 27, 4)
  RETURN_STRING(Exiv2::dateTimeOriginal(*exifData));
#else
  RETURN_STRING(exifData->findKey(Exiv2::ExifKey("Exif.Photo.DateTimeOriginal")));
#endif
}

cras::optional<ExifData<ExifAscii>> Exiv2MetadataExtractor::getExifOffsetTimeOriginal()
{
#if EXIV2_TEST_VERSION(0, 27, 4)
  RETURN_STRING(exifData->findKey(Exiv2::ExifKey("Exif.Photo.OffsetTimeOriginal")));
#else
  Exiv2::TagInfo ti(0x9011, "0x9011", "", "", 5, 10, Exiv2::asciiString, 1, printValue);
  RETURN_STRING(exifData->findKey(Exiv2::ExifKey(ti)));
#endif
}

cras::optional<ExifData<ExifAscii>> Exiv2MetadataExtractor::getExifSubSecTimeOriginal()
{
  RETURN_STRING(exifData->findKey(Exiv2::ExifKey("Exif.Photo.SubSecTimeOriginal")));
}

cras::optional<ExifData<ExifShort>> Exiv2MetadataExtractor::getExifOrientation()
{
  RETURN_SHORT(Exiv2::orientation(*exifData));
}

cras::optional<ExifData<ExifRational>> Exiv2MetadataExtractor::getExifFocalPlaneXRes()
{
  RETURN_RATIONAL(exifData->findKey(Exiv2::ExifKey("Exif.Photo.FocalPlaneXResolution")));
}

cras::optional<ExifData<ExifRational>> Exiv2MetadataExtractor::getExifFocalPlaneYRes()
{
  RETURN_RATIONAL(exifData->findKey(Exiv2::ExifKey("Exif.Photo.FocalPlaneYResolution")));
}

cras::optional<ExifData<ExifShort>> Exiv2MetadataExtractor::getExifFocalPlaneResUnit()
{
  RETURN_SHORT(exifData->findKey(Exiv2::ExifKey("Exif.Photo.FocalPlaneResolutionUnit")));
}

cras::optional<ExifData<ExifShort>> Exiv2MetadataExtractor::getExifResUnit()
{
  RETURN_SHORT(exifData->findKey(Exiv2::ExifKey("Exif.Image.ResolutionUnit")));
}

cras::optional<ExifData<ExifShort>> Exiv2MetadataExtractor::getExifFocalLength35MM()
{
  RETURN_SHORT(exifData->findKey(Exiv2::ExifKey("Exif.Photo.FocalLengthIn35mmFilm")));
}

cras::optional<ExifData<ExifRational>> Exiv2MetadataExtractor::getExifFocalLength()
{
  RETURN_RATIONAL(Exiv2::focalLength(*exifData));
}

cras::optional<ExifData<ExifAscii>> Exiv2MetadataExtractor::getExifGpsLatRef()
{
  RETURN_STRING(exifData->findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSLatitudeRef")));
}

cras::optional<ExifData<ExifRational>> Exiv2MetadataExtractor::getExifGpsLat(const size_t n)
{
  RETURN_RATIONAL_N(exifData->findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSLatitude")), n);
}

cras::optional<ExifData<ExifAscii>> Exiv2MetadataExtractor::getExifGpsLonRef()
{
  RETURN_STRING(exifData->findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSLongitudeRef")));
}

cras::optional<ExifData<ExifRational>> Exiv2MetadataExtractor::getExifGpsLon(const size_t n)
{
  RETURN_RATIONAL_N(exifData->findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSLongitude")), n);
}

cras::optional<ExifData<ExifByte>> Exiv2MetadataExtractor::getExifGpsAltRef()
{
  RETURN_BYTE(exifData->findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSAltitudeRef")));
}

cras::optional<ExifData<ExifRational>> Exiv2MetadataExtractor::getExifGpsAlt()
{
  RETURN_RATIONAL(exifData->findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSAltitude")));
}

cras::optional<ExifData<ExifAscii>> Exiv2MetadataExtractor::getExifGpsMeasureMode()
{
  RETURN_STRING(exifData->findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSMeasureMode")));
}

cras::optional<ExifData<ExifRational>> Exiv2MetadataExtractor::getExifGpsDOP()
{
  RETURN_RATIONAL(exifData->findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSDOP")));
}

cras::optional<ExifData<ExifAscii>> Exiv2MetadataExtractor::getExifGpsSpeedRef()
{
  RETURN_STRING(exifData->findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSSpeedRef")));
}

cras::optional<ExifData<ExifRational>> Exiv2MetadataExtractor::getExifGpsSpeed()
{
  RETURN_RATIONAL(exifData->findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSSpeed")));
}

cras::optional<ExifData<ExifAscii>> Exiv2MetadataExtractor::getExifGpsTrackRef()
{
  RETURN_STRING(exifData->findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSTrackRef")));
}

cras::optional<ExifData<ExifRational>> Exiv2MetadataExtractor::getExifGpsTrack()
{
  RETURN_RATIONAL(exifData->findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSTrack")));
}

cras::optional<ExifData<ExifRational>> Exiv2MetadataExtractor::getExifGpsTimeStamp(const size_t n)
{
  RETURN_RATIONAL_N(exifData->findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSTimeStamp")), n);
}

cras::optional<ExifData<ExifAscii>> Exiv2MetadataExtractor::getExifGpsDateStamp()
{
  RETURN_STRING(exifData->findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSDateStamp")));
}

cras::optional<ExifData<ExifShort>> Exiv2MetadataExtractor::getExifGpsDifferential()
{
  RETURN_SHORT(exifData->findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSDifferential")));
}

cras::optional<ExifData<ExifRational>> Exiv2MetadataExtractor::getExifGpsHPositioningError()
{
#if EXIV2_TEST_VERSION(0, 27, 4)
  RETURN_RATIONAL(exifData->findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSHPositioningError")));
#else
  Exiv2::TagInfo ti(0x001f, "0x001f", "", "", 6, 12, Exiv2::unsignedRational, 1, printFloat);
  RETURN_RATIONAL(exifData->findKey(Exiv2::ExifKey(ti)));
#endif
}

cras::optional<ExifData<ExifAscii>> Exiv2MetadataExtractor::getExifGpsImgDirectionRef()
{
  RETURN_STRING(exifData->findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSImgDirectionRef")));
}

cras::optional<ExifData<ExifRational>> Exiv2MetadataExtractor::getExifGpsImgDirection()
{
  RETURN_RATIONAL(exifData->findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSImgDirection")));
}

cras::optional<ExifData<ExifSRational>> Exiv2MetadataExtractor::getExifAcceleration(const size_t n)
{
  const auto exifData = this->data->getExifData();
  if (exifData == nullptr)
    return cras::nullopt;

  if (n > 2)
    return cras::nullopt;

  try
  {
    if (this->data->makernotes != nullptr)
    {
      // Apple
      // TODO the computation is not general enough. It worked for iPhone 12, but doesn't work for iPhone SE.
      const auto accScale = 1.0 / 12.8;  // This is a magic number that seems to fit the samples I saw
      size_t axis;
      double scale;
      // Apple has X left, Y down, Z towards user when the phone is upright.
      switch (n)
      {
        case 0:
          axis = 2;
          scale = -accScale;
          break;
        case 1:
          axis = 0;
          scale = accScale;
          break;
        case 2:
          axis = 1;
          scale = -accScale;
          break;
      }

      // We cannot construct these custom EXIF keys from strings normally because they are not registered with Exiv2
      const auto appleIosAccelerationVectorKey = this->data->makernotes->keys.at("AppleIosAccelerationVector");
      const auto accel = getExifRational(*exifData, appleIosAccelerationVectorKey, axis);

      if (accel.has_value())
        return ExifData<ExifSRational>{appleIosAccelerationVectorKey.key(), scale * accel.value()};
    }

    // Panasonic
    {
      const auto accScale = 0.034795;  // This is a magic number that seems to fit the samples I saw
      std::string key;
      double scale;
      // Panasonic has X left, Y towards user, Z up
      switch (n)
      {
        case 0:
          key = "Exif.Panasonic.AccelerometerY";
          scale = -accScale;
          break;
        case 1:
          key = "Exif.Panasonic.AccelerometerX";
          scale = accScale;
          break;
        case 2:
          key = "Exif.Panasonic.AccelerometerZ";
          scale = accScale;
          break;
      }

      const auto accel = getExifInt16(*exifData, Exiv2::ExifKey(key));
      if (accel.has_value())
        return ExifData<ExifSRational>{key, scale * accel.value()};
    }
  }
  catch (const Exiv2::AnyError& e)
  {
    CRAS_ERROR_NAMED("exiv2", "Error reading image metadata: %s", e.what());
  }
  catch (const std::out_of_range& e)
  {
    CRAS_ERROR_NAMED("exiv2", "Error reading image metadata: %s", e.what());
  }

  return cras::nullopt;
}

cras::optional<ExifData<ExifSRational>> Exiv2MetadataExtractor::getExifRollAngle()
{
  const auto exifData = this->data->getExifData();
  if (exifData == nullptr)
    return cras::nullopt;

  try
  {
    // Panasonic
    const Exiv2::ExifKey key{"Exif.Panasonic.RollAngle"};
    const auto panaRoll = getExifInt16(*exifData, key, 0);
    if (panaRoll.has_value())
    {
      const auto rollDeg = *panaRoll / 10.0;
      return ExifData<ExifSRational>{key.key(), rollDeg / 180.0 * M_PI};
    }
  }
  catch (const Exiv2::AnyError& e)
  {
    CRAS_ERROR_NAMED("exiv2", "Error reading image metadata: %s", e.what());
  }
  catch (const std::out_of_range& e)
  {
    CRAS_ERROR_NAMED("exiv2", "Error reading image metadata: %s", e.what());
  }

  return cras::nullopt;
}

cras::optional<ExifData<ExifSRational>> Exiv2MetadataExtractor::getExifPitchAngle()
{
  const auto exifData = this->data->getExifData();
  if (exifData == nullptr)
    return cras::nullopt;

  try
  {
    // Panasonic
    const Exiv2::ExifKey key{"Exif.Panasonic.PitchAngle"};
    const auto panaPitch = getExifInt16(*exifData, key, 0);
    if (panaPitch.has_value())
    {
      const auto rollDeg = *panaPitch / 10.0;
      return ExifData<ExifSRational>{key.key(), rollDeg / 180.0 * M_PI};
    }
  }
  catch (const Exiv2::AnyError& e)
  {
    CRAS_ERROR_NAMED("exiv2", "Error reading image metadata: %s", e.what());
  }
  catch (const std::out_of_range& e)
  {
    CRAS_ERROR_NAMED("exiv2", "Error reading image metadata: %s", e.what());
  }

  return cras::nullopt;
}

MetadataExtractor::Ptr Exiv2MetadataExtractorPlugin::getExtractor(const MetadataExtractorParams& params)
{
  if (params.log == nullptr || params.info->filenameOrURL().empty() ||
      params.info->width() == 0 || params.info->height() == 0)
    return nullptr;

  return std::make_shared<Exiv2MetadataExtractor>(
    params.log, params.manager, params.info->filenameOrURL(), params.info->width(), params.info->height());
}

}

PLUGINLIB_EXPORT_CLASS(movie_publisher::Exiv2MetadataExtractorPlugin, movie_publisher::MetadataExtractorPlugin)
