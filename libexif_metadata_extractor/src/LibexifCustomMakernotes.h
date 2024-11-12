// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief
 * \author Martin Pecka
 */

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include <libexif/exif-data.h>
#include <libexif/exif-mnote-data.h>

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

struct LibexifCustomMakerNote
{
  std::function<int(const ::ExifData*, const ::ExifEntry*)> identify;
  std::function<::ExifMnoteData*(::ExifMem*)> create;
};

class LibexifCustomMakernotes
{
public:
  LibexifCustomMakernotes();
  std::unordered_map<std::string, LibexifCustomMakerNote> customMakerNotes;
};

}
