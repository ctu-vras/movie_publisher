// SPDX-License-Identifier: GPL-2.0-or-later
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Exiv2 custom MakerNotes.
 * \author Martin Pecka
 */

#include <exiv2/exiv2.hpp>

#include "Exiv2CustomMakernotes.h"

namespace movie_publisher
{

class AppleIosMakerNoteHeader : public MakerNoteHeader
{
public:
  bool read(const Exiv2::byte* pData, size_t size) override;
  Exiv2::ByteOrder byteOrder() const override { return Exiv2::bigEndian; };
  size_t size() const override { return sizeof(signature_); }

private:
  static constexpr Exiv2::byte signature_[] {'A', 'p', 'p', 'l', 'e', ' ', 'i', 'O', 'S', 0x00, 0x00, 0x01, 'M', 'M'};
};

bool AppleIosMakerNoteHeader::read(const Exiv2::byte* pData, size_t size) {
  if (pData == nullptr || size < sizeof(signature_))
    return false;
  if (0 != memcmp(pData, signature_, sizeof(signature_)))
    return false;
  return true;
}

inline std::ostream& printValue(std::ostream& os, const Exiv2::Value& value, const Exiv2::ExifData*)
{
  return os << value;
}

CustomMakernotes::CustomMakernotes()
{
  const auto ifdId = Exiv2::ExifKey("Exif.MakerNote.Offset").ifdId();
  const auto sectionId = 15;

  // Apple makernotes decoding from https://github.com/exiftool/exiftool/blob/master/lib/Image/ExifTool/Apple.pm
  const std::list<std::tuple<std::string, std::shared_ptr<MakerNoteHeader>, std::list<Exiv2::TagInfo>>> tags = {
    {"Apple", std::make_shared<AppleIosMakerNoteHeader>(), {
      {0x0001, "AppleIosMakerNoteVersion", "Maker Note Version", "Maker Note Version", ifdId, sectionId, Exiv2::signedLong, 1, printValue},  // NOLINT
      {0x0004, "AppleIosAEStable", "AE Stable?", "Was auto exposure stable?", ifdId, sectionId, Exiv2::signedLong, 1, printValue},  // NOLINT
      {0x0007, "AppleIosAFStable", "AF Stable?", "Was auto focus stable?", ifdId, sectionId, Exiv2::signedLong, 1, printValue},  // NOLINT
      {0x0008, "AppleIosAccelerationVector", "Acceleration Vector", "XYZ coordinates of the acceleration vector in units of g. As viewed from the front of the phone, positive X is toward the left side, positive Y is toward the bottom, and positive Z points into the face of the phone.", ifdId, sectionId, Exiv2::signedRational, 3, printValue},  // NOLINT
      {0x002e, "AppleIosCameraType", "Camera Type", "Camera type (0=Back Wide, 1=Back Normal, 6=Front)", ifdId, sectionId, Exiv2::signedLong, 1, printValue},  // NOLINT
    }},
  };

  for (const auto& data : tags)
  {
    const auto make = std::get<0>(data);
    const auto header = std::get<1>(data);
    const auto tagList = std::get<2>(data);

    this->headers[make] = header;

    for (const auto& tag : tagList)
    {
      this->tagsById[make].emplace(tag.tag_, tag);
      this->tagsByName[make].emplace(tag.name_, tag);
      this->keys.emplace(tag.name_, Exiv2::ExifKey(tag));
    }
  }
}

}
