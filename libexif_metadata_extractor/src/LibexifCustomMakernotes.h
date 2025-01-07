// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Implementation of some custom MakerNotes parsers for libexif.
 * \author Martin Pecka
 */

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include <libexif/exif-data.h>
#include <libexif/exif-mnote-data.h>

// A few struct definitions from libexif had to be copied here because they are defined in headers which are not
// installed. libexif is issued under LGPL, but I hope just copying the structs (aka API) does not necessarily mean
// this whole would need to be under LGPL.

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <libexif/exif-byte-order.h>
#include <libexif/exif-log.h>

/*! \internal */
typedef struct _ExifMnoteDataMethods ExifMnoteDataMethods;

/*! \internal */
struct _ExifMnoteDataMethods {
  /* Life cycle */
  void (* free) (ExifMnoteData *);  // NOLINT

  /* Modification */
  void (* save) (ExifMnoteData *, unsigned char **, unsigned int *);  // NOLINT
  void (* load) (ExifMnoteData *, const unsigned char *, unsigned int);  // NOLINT
  void (* set_offset)     (ExifMnoteData *, unsigned int);  // NOLINT
  void (* set_byte_order) (ExifMnoteData *, ExifByteOrder);  // NOLINT

  /* Query */
  unsigned int (* count)           (ExifMnoteData *);  // NOLINT
  unsigned int (* get_id)          (ExifMnoteData *, unsigned int);  // NOLINT
  const char * (* get_name)        (ExifMnoteData *, unsigned int);  // NOLINT
  const char * (* get_title)       (ExifMnoteData *, unsigned int);  // NOLINT
  const char * (* get_description) (ExifMnoteData *, unsigned int);  // NOLINT
  char * (* get_value)             (ExifMnoteData *, unsigned int, char *val, unsigned int maxlen);  // NOLINT
};

/*! \internal */
typedef struct _ExifMnoteDataPriv ExifMnoteDataPriv;

/*! \internal */
struct _ExifMnoteData
{
  ExifMnoteDataPriv *priv;

  ExifMnoteDataMethods methods;

  /* Logging */
  ExifLog *log;

  /* Memory management */
  ExifMem *mem;
};

/*! \internal */
void exif_mnote_data_construct      (ExifMnoteData *, ExifMem *mem);  // NOLINT

/*! \internal */
void exif_mnote_data_set_byte_order (ExifMnoteData *, ExifByteOrder);  // NOLINT

/*! \internal */
void exif_mnote_data_set_offset     (ExifMnoteData *, unsigned int);  // NOLINT

#ifdef __cplusplus
}
#endif /* __cplusplus */

namespace movie_publisher
{
/**
 * \brief Description of a custom MakerNote parser for one vendor.
 */
struct LibexifCustomMakerNote
{
  /**
   * \brief Function that identifies the MakerNote.
   * \param[in] exifData The whole EXIF data.
   * \param[in] exifEntry The MakerNote entry.
   * \return Non-zero if the MakerNote matches this custom implementation.
   */
  std::function<int(const ::ExifData* exifData, const ::ExifEntry* exifEntry)> identify;

  /**
   * \brief Create a new instance of the custom MakerNote.
   * \param[in] mem The memory pool to allocate from.
   */
  std::function<::ExifMnoteData*(::ExifMem* mem)> create;
};

/**
 * \brief Container of all implemented custom MakerNote parsers.
 */
class LibexifCustomMakernotes
{
public:
  LibexifCustomMakernotes();

  //! All registered custom MakerNote parsers.
  std::unordered_map<std::string, LibexifCustomMakerNote> customMakerNotes;
};

}
