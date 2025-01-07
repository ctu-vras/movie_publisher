// SPDX-License-Identifier: GPL-2.0-or-later
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Exiv2 custom MakerNotes.
 * \author Martin Pecka
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <exiv2/exiv2.hpp>

namespace movie_publisher
{
/**
 * \brief Header of a custom MakeNote.
 */
class MakerNoteHeader
{
public:
  virtual ~MakerNoteHeader() = default;
  virtual bool read(const Exiv2::byte* pData, size_t size) = 0;
  virtual Exiv2::ByteOrder byteOrder() const = 0;
  virtual size_t size() const = 0;
};

/**
 * \brief All custom Makernotes.
 */
class CustomMakernotes
{
public:
  CustomMakernotes();
  std::unordered_map<std::string, std::shared_ptr<MakerNoteHeader>> headers;
  std::unordered_map<std::string, std::unordered_map<uint16_t, Exiv2::TagInfo>> tagsById;
  std::unordered_map<std::string, std::unordered_map<std::string, Exiv2::TagInfo>> tagsByName;
  std::unordered_map<std::string, Exiv2::ExifKey> keys;
};

}
