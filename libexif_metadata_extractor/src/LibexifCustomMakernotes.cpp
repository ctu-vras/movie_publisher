// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Implementation of some custom MakerNotes parsing for libexif.
 * \author Martin Pecka
 */

#include <cstring>

#include <libexif/exif-mnote-data.h>

#include "LibexifCustomMakernotes.h"

namespace movie_publisher
{
enum _MnoteAppleTag
{
  MNOTE_APPLE_TAG_RUNTIME = 0x3,
  MNOTE_APPLE_TAG_ACCELERATIOVECTOR = 0x9,
  MNOTE_APPLE_TAG_HDR = 0xA,
  MNOTE_APPLE_TAG_BURST_UUID = 0xB,
  MNOTE_APPLE_TAG_MEDIA_GROUP_UUID = 0x11,
  MNOTE_APPLE_TAG_IMAGE_UNIQUE_ID = 0x15
};
typedef enum _MnoteAppleTag MnoteAppleTag;

enum _MnotePanasonicTag
{
  MNOTE_PANASONIC_TAG_INTERNAL_SERIAL_NUMBER = 0x25,
  MNOTE_PANASONIC_TAG_LENS_SERIAL_NUMBER = 0x52,
  MNOTE_PANASONIC_TAG_ACCELEROMETER_Z = 0x8c,
  MNOTE_PANASONIC_TAG_ACCELEROMETER_X = 0x8d,
  MNOTE_PANASONIC_TAG_ACCELEROMETER_Y = 0x8e,
  MNOTE_PANASONIC_TAG_ROLL_ANGLE = 0x90,
  MNOTE_PANASONIC_TAG_PITCH_ANGLE = 0x91,
};
typedef enum _MnotePanasonicTag MnotePanasonicTag;

typedef struct _MnoteIfdEntry MnoteIfdEntry;
struct _MnoteIfdEntry
{
  unsigned int tag;
  ExifFormat format;
  unsigned long components; // NOLINT
  unsigned char* data;
  unsigned int size;
  ExifByteOrder order;
};

typedef struct _ExifMnoteDataIfd ExifMnoteDataIfd;
struct _ExifMnoteDataIfd
{
  ExifMnoteData parent;
  ExifByteOrder order;
  unsigned int offset;
  MnoteIfdEntry* entries;
  unsigned int count;
};

static const struct
{
  MnoteAppleTag tag;
  const char* name;
  const char* title;
  const char* description;
} appleTable[] = {
  {MNOTE_APPLE_TAG_HDR, "HDR", ("HDR Mode"), ""},
  {MNOTE_APPLE_TAG_RUNTIME, "RUNTIME", ("Runtime"), ""},
  {MNOTE_APPLE_TAG_ACCELERATIOVECTOR, "ACCELERATIONVECTOR", ("Acceleration Vector"), ""},
  {MNOTE_APPLE_TAG_HDR, "HDR", ("HDR"), ""},
  {MNOTE_APPLE_TAG_BURST_UUID, "BURST_UUID", ("Burst UUID"), ""},
  {MNOTE_APPLE_TAG_MEDIA_GROUP_UUID, "MEDIA_GROUP_UUID", ("Media Group UUID"), ""},
  {MNOTE_APPLE_TAG_IMAGE_UNIQUE_ID, "IMAGE_UNIQUE_ID", ("Image Unique ID"), ""},
  {static_cast<MnoteAppleTag>(0), NULL, NULL, NULL}
};

static const struct
{
  MnotePanasonicTag tag;
  const char* name;
  const char* title;
  const char* description;
} panasonicTable[] = {
  {MNOTE_PANASONIC_TAG_INTERNAL_SERIAL_NUMBER, "InternalSerialNumber", ("Internal serial number"), ""},
  {MNOTE_PANASONIC_TAG_LENS_SERIAL_NUMBER, "LensSerialNumber", ("Lens serial number"), ""},
  {MNOTE_PANASONIC_TAG_ACCELEROMETER_Z, "AccelerometerZ", ("Accelerometer Z"), ""},
  {MNOTE_PANASONIC_TAG_ACCELEROMETER_X, "AccelerometerX", ("Accelerometer X"), ""},
  {MNOTE_PANASONIC_TAG_ACCELEROMETER_Y, "AccelerometerY", ("Accelerometer Y"), ""},
  {MNOTE_PANASONIC_TAG_ROLL_ANGLE, "RollAngle", ("Roll angle"), ""},
  {MNOTE_PANASONIC_TAG_PITCH_ANGLE, "PitchAngle", ("Pitch angle"), ""},
  {static_cast<MnotePanasonicTag>(0), NULL, NULL, NULL}
};

const char*
mnote_apple_tag_get_name(MnoteAppleTag t)
{
  unsigned int i;

  for (i = 0; i < sizeof (appleTable) / sizeof (appleTable[0]); i++)
  {
    if (appleTable[i].tag == t)
    {
      return appleTable[i].name;
    }
  }

  return NULL;
}

const char*
mnote_apple_tag_get_title(MnoteAppleTag t)
{
  unsigned int i;

  for (i = 0; i < sizeof (appleTable) / sizeof (appleTable[0]); i++)
  {
    if (appleTable[i].tag == t)
    {
      return (appleTable[i].title);
    }
  }

  return NULL;
}

const char*
mnote_apple_tag_get_description(MnoteAppleTag t)
{
  unsigned int i;

  for (i = 0; i < sizeof (appleTable) / sizeof (appleTable[0]); i++)
  {
    if (appleTable[i].tag == t)
    {
      if (!appleTable[i].description || !*appleTable[i].description)
      {
        return "";
      }
      return (appleTable[i].description);
    }
  }

  return NULL;
}

const char*
mnote_panasonic_tag_get_name(MnotePanasonicTag t)
{
  unsigned int i;

  for (i = 0; i < sizeof (panasonicTable) / sizeof (panasonicTable[0]); i++)
  {
    if (panasonicTable[i].tag == t)
    {
      return panasonicTable[i].name;
    }
  }

  return NULL;
}

const char*
mnote_panasonic_tag_get_title(MnotePanasonicTag t)
{
  unsigned int i;

  for (i = 0; i < sizeof (panasonicTable) / sizeof (panasonicTable[0]); i++)
  {
    if (panasonicTable[i].tag == t)
    {
      return (panasonicTable[i].title);
    }
  }

  return NULL;
}

const char*
mnote_panasonic_tag_get_description(MnotePanasonicTag t)
{
  unsigned int i;

  for (i = 0; i < sizeof (panasonicTable) / sizeof (panasonicTable[0]); i++)
  {
    if (panasonicTable[i].tag == t)
    {
      if (!panasonicTable[i].description || !*panasonicTable[i].description)
      {
        return "";
      }
      return (panasonicTable[i].description);
    }
  }

  return NULL;
}

char*
mnote_ifd_entry_get_value(MnoteIfdEntry* entry, char* v, unsigned int maxlen)
{
  ExifLong vl;
  ExifSLong vsl;
  ExifShort vs;
  ExifSShort vss;
  ExifRational vr;
  ExifSRational vsr;
  size_t size;
  unsigned char* data;

  if (!entry)
    return NULL;

  memset(v, 0, maxlen);
  maxlen--;

  size = entry->size;
  data = entry->data;
  switch (entry->tag)
  {
    case MNOTE_APPLE_TAG_HDR:
      if (size < 4) return NULL;
      if (entry->format != EXIF_FORMAT_SLONG) return NULL;
      if (entry->components != 1) return NULL;

      vsl = exif_get_slong(data, entry->order);
      snprintf(v, maxlen, "%d", vsl);
      break;
    case MNOTE_APPLE_TAG_IMAGE_UNIQUE_ID:
    case MNOTE_APPLE_TAG_BURST_UUID:
    case MNOTE_APPLE_TAG_MEDIA_GROUP_UUID:
      if (entry->format != EXIF_FORMAT_ASCII) return NULL;
      strncpy(v, (char*)data, MIN(maxlen-1, size));  // NOLINT
      v[MIN(maxlen-1, size)] = 0;
      break;
    default:
      switch (entry->format)
      {
        case EXIF_FORMAT_ASCII:
          strncpy(v, (char*)data, MIN(maxlen, size));  // NOLINT
          break;
        case EXIF_FORMAT_SHORT:
        {
          size_t i, len = 0;

          for (i = 0; i < entry->components; i++)
          {
            if (size < 2)
              break;
            if (len > maxlen)
              break;
            vs = exif_get_short(data, entry->order);
            snprintf(v + len, maxlen - len, "%hu ", vs);
            len = strlen(v);
            data += 2;
            size -= 2;
          }
        }
        break;
        case EXIF_FORMAT_SSHORT:
        {
          size_t i, len = 0;
          for (i = 0; i < entry->components; i++)
          {
            if (size < 2)
              break;
            if (len > maxlen)
              break;
            vss = exif_get_sshort(data, entry->order);
            snprintf(v + len, maxlen - len, "%hi ", vss);
            len = strlen(v);
            data += 2;
            size -= 2;
          }
        }
        break;
        case EXIF_FORMAT_LONG:
        {
          size_t i, len = 0;
          for (i = 0; i < entry->components; i++)
          {
            if (size < 4)
              break;
            if (len > maxlen)
              break;
            vl = exif_get_long(data, entry->order);
            snprintf(v + len, maxlen - len, "%lu ", (long unsigned)vl);  // NOLINT
            len = strlen(v);
            data += 4;
            size -= 4;
          }
        }
        break;
        case EXIF_FORMAT_SLONG:
        {
          size_t i, len = 0;
          for (i = 0; i < entry->components; i++)
          {
            if (size < 4)
              break;
            if (len > maxlen)
              break;
            vsl = exif_get_slong(data, entry->order);
            snprintf(v + len, maxlen - len, "%li ", (long int)vsl);  // NOLINT
            len = strlen(v);
            data += 4;
            size -= 4;
          }
        }
        break;
        case EXIF_FORMAT_RATIONAL:
        {
          if (size < exif_format_get_size(EXIF_FORMAT_RATIONAL)) return NULL;
          if (entry->components < 1) return NULL;
          size_t i, len = 0;
          for (i = 0; i < entry->components; i++)
          {
            if (size < 8)
              break;
            if (len > maxlen)
              break;
            vr = exif_get_rational(data, entry->order);
            snprintf(v + len, maxlen - len, "%2.4f ", (double)vr.numerator / vr.denominator);  // NOLINT
            len = strlen(v);
            data += 4;
            size -= 4;
          }
        }
        break;
        case EXIF_FORMAT_SRATIONAL:
        {
          if (size < exif_format_get_size(EXIF_FORMAT_SRATIONAL)) return NULL;
          if (entry->components < 1) return NULL;
          size_t i, len = 0;
          for (i = 0; i < entry->components; i++)
          {
            if (size < 8)
              break;
            if (len > maxlen)
              break;
            vsr = exif_get_srational(data, entry->order);
            snprintf(v + len, maxlen - len, "%2.4f ", (double)vsr.numerator / vsr.denominator);  // NOLINT
            len = strlen(v);
            data += 8;
            size -= 8;
          }
        }
        break;
        case EXIF_FORMAT_UNDEFINED:
        default:
        {
          size_t i, len = 0;
          snprintf(v + len, maxlen - len, "0x");
          len += 2;
          for (i = 0; i < std::min(entry->size, static_cast<unsigned int>(maxlen - len)); i++)
          {
            snprintf(v + len, maxlen - len, "%02x", data[i]);
            len += 2;
          }
        }
      }
      break;
  }

  return v;
}

static void
exif_mnote_data_ifd_free(ExifMnoteData* md)
{
  ExifMnoteDataIfd* d = (ExifMnoteDataIfd*)md;  // NOLINT
  unsigned int i;

  /*printf("%s\n", __FUNCTION__);*/

  if (!d)
  {
    return;
  }

  if (d->entries)
  {
    for (i = 0; i < d->count; i++)
    {
      if (d->entries[i].data)
      {
        exif_mem_free(md->mem, d->entries[i].data);
      }
    }
    exif_mem_free(md->mem, d->entries);
    d->entries = NULL;
    d->count = 0;
  }

  return;
}

static void
exif_mnote_data_ifd_load(ExifMnoteData* md,
                         const unsigned char* buf,
                         unsigned int buf_size,
                         ptrdiff_t data_start,
                         bool largeDataRelativeToMNote)
{
  ExifMnoteDataIfd* d = (ExifMnoteDataIfd*)md;  // NOLINT
  unsigned int tcount, i;
  unsigned int dsize;
  unsigned int ofs, dofs;

  /*printf("%s\n", __FUNCTION__);*/

  if (!d || !buf || (buf_size < 6 + data_start))
  {
    exif_log(md->log,
             EXIF_LOG_CODE_CORRUPT_DATA,
             "ExifMnoteDataApple",
             "Short MakerNote");
    return;
  }

  /* Start of interesting data */
  // ofs = 0;//d->offset + 6;
  ofs = d->offset + 6;
  if (ofs > buf_size - data_start)
  {
    exif_log(md->log,
             EXIF_LOG_CODE_CORRUPT_DATA,
             "ExifMnoteDataApple",
             "Short MakerNote");
    return;
  }

  if ((buf[ofs + (data_start - 4)] == 'M') && (buf[ofs + (data_start - 3)] == 'M'))
  {
    d->order = EXIF_BYTE_ORDER_MOTOROLA;
  }
  else if ((buf[ofs + (data_start - 4)] == 'I') && (buf[ofs + (data_start - 3)] == 'I'))
  {
    d->order = EXIF_BYTE_ORDER_INTEL;
  }

  tcount = (unsigned int)exif_get_short(buf + ofs + (data_start - 2), d->order);

  /* Sanity check the offset */
  if (buf_size < d->offset + 6 + data_start + tcount * 12 + 4)
  {
    exif_log(md->log,
             EXIF_LOG_CODE_CORRUPT_DATA,
             "ExifMnoteDataApple",
             "Short MakerNote");
    return;
  }

  ofs += data_start;

  exif_mnote_data_ifd_free(md);

  /* Reserve enough space for all the possible MakerNote tags */
  d->entries = static_cast<MnoteIfdEntry*>(exif_mem_alloc(md->mem, sizeof(MnoteIfdEntry) * tcount));
  if (!d->entries)
  {
    EXIF_LOG_NO_MEMORY(md->log, "ExifMnoteApple", sizeof(MnoteIfdEntry) * tcount);
    return;
  }
  memset(d->entries, 0, sizeof(MnoteIfdEntry) * tcount);

  for (i = 0; i < tcount; i++)
  {
    if (ofs + 12 > buf_size)
    {
      exif_log(md->log,
               EXIF_LOG_CODE_CORRUPT_DATA,
               "ExifMnoteApplet",
               "Tag size overflow detected (%u vs size %u)",
               ofs + 12,
               buf_size);
      break;
    }
    d->entries[i].tag = exif_get_short(buf + ofs, d->order);
    d->entries[i].format = static_cast<ExifFormat>(exif_get_short(buf + ofs + 2, d->order));
    d->entries[i].components = exif_get_long(buf + ofs + 4, d->order);
    d->entries[i].order = d->order;
    if ((d->entries[i].components) && (buf_size / d->entries[i].components <
      exif_format_get_size(d->entries[i].format)))
    {
      exif_log(md->log,
               EXIF_LOG_CODE_CORRUPT_DATA,
               "ExifMnoteApplet",
               "Tag size overflow detected (components %lu vs size %u)",
               d->entries[i].components,
               buf_size);
      break;
    }
    dsize = exif_format_get_size(d->entries[i].format) * d->entries[i].components;
    if ((dsize > 65536) || (dsize > buf_size))
    {
      /* Corrupt data: EXIF data size is limited to the
       * maximum size of a JPEG segment (64 kb).
       */
      break;
    }
    if (dsize > 4)
    {
      if (largeDataRelativeToMNote)
        dofs = d->offset + exif_get_long(buf + ofs + 8, d->order) + 6;
      else
        dofs = exif_get_long(buf + ofs + 8, d->order) + 6;
    }
    else
    {
      dofs = ofs + 8;
    }
    if (dofs > buf_size)
    {
      exif_log(md->log,
               EXIF_LOG_CODE_CORRUPT_DATA,
               "ExifMnoteApplet",
               "Tag size overflow detected (%u vs size %u)",
               dofs,
               buf_size);
      continue;
    }
    ofs += 12;
    d->entries[i].data = static_cast<unsigned char*>(exif_mem_alloc(md->mem, dsize));
    if (!d->entries[i].data)
    {
      EXIF_LOG_NO_MEMORY(md->log, "ExifMnoteApple", dsize);
      continue;
    }
    if (dofs + dsize > buf_size)
    {
      exif_log(md->log,
               EXIF_LOG_CODE_CORRUPT_DATA,
               "ExifMnoteApplet",
               "Tag size overflow detected (%u vs size %u)",
               dofs + dsize,
               buf_size);
      continue;
    }
    memcpy(d->entries[i].data, buf + dofs, dsize);
    d->entries[i].size = dsize;
  }
  d->count = tcount;

  return;
}

static void
exif_mnote_data_apple_load(ExifMnoteData* md, const unsigned char* buf, unsigned int buf_size)
{
  exif_mnote_data_ifd_load(md, buf, buf_size, 16, true);
}

static void
exif_mnote_data_panasonic_load(ExifMnoteData* md, const unsigned char* buf, unsigned int buf_size)
{
  exif_mnote_data_ifd_load(md, buf, buf_size, 14, false);
}

static void
exif_mnote_data_ifd_set_offset(ExifMnoteData* md, unsigned int o)
{
  /*printf("%s\n", __FUNCTION__);*/

  if (md)
  {
    ((ExifMnoteDataIfd*)md)->offset = o;  // NOLINT
  }

  return;
}

static void
exif_mnote_data_ifd_set_byte_order(ExifMnoteData* md, ExifByteOrder o)
{
  ExifMnoteDataIfd* d = (ExifMnoteDataIfd*)md;  // NOLINT
  unsigned int i;

  /*printf("%s\n", __FUNCTION__);*/

  if (!d || d->order == o)
  {
    return;
  }

  for (i = 0; i < d->count; i++)
  {
    if (d->entries[i].components && (d->entries[i].size / d->entries[i].components < exif_format_get_size(
      d->entries[i].format)))
      continue;
    exif_array_set_byte_order(d->entries[i].format,
                              d->entries[i].data,
                              d->entries[i].components,
                              d->entries[i].order,
                              o);
    d->entries[i].order = o;
  }
  d->order = o;

  return;
}

static unsigned int
exif_mnote_data_ifd_count(ExifMnoteData* md)
{
  /*printf("%s\n", __FUNCTION__);*/

  return md ? ((ExifMnoteDataIfd*)md)->count : 0;  // NOLINT
}

static unsigned int
exif_mnote_data_ifd_get_id(ExifMnoteData* md, unsigned int i)
{
  ExifMnoteDataIfd* d = (ExifMnoteDataIfd*)md;  // NOLINT

  if (!d || (d->count <= i))
  {
    return 0;
  }

  return d->entries[i].tag;
}

static const char*
exif_mnote_data_apple_get_name(ExifMnoteData* md, unsigned int i)
{
  ExifMnoteDataIfd* d = (ExifMnoteDataIfd*)md;  // NOLINT

  if (!d || (d->count <= i))
  {
    return NULL;
  }

  return mnote_apple_tag_get_name(static_cast<MnoteAppleTag>(d->entries[i].tag));
}

static const char*
exif_mnote_data_apple_get_title(ExifMnoteData* md, unsigned int i)
{
  ExifMnoteDataIfd* d = (ExifMnoteDataIfd*)md;  // NOLINT

  if (!d || (d->count <= i))
  {
    return NULL;
  }

  return mnote_apple_tag_get_title(static_cast<MnoteAppleTag>(d->entries[i].tag));
}

static const char*
exif_mnote_data_apple_get_description(ExifMnoteData* md, unsigned int i)
{
  ExifMnoteDataIfd* d = (ExifMnoteDataIfd*)md;  // NOLINT

  if (!d || (d->count <= i))
  {
    return NULL;
  }

  return mnote_apple_tag_get_description(static_cast<MnoteAppleTag>(d->entries[i].tag));
}

static const char*
exif_mnote_data_panasonic_get_name(ExifMnoteData* md, unsigned int i)
{
  ExifMnoteDataIfd* d = (ExifMnoteDataIfd*)md;  // NOLINT

  if (!d || (d->count <= i))
  {
    return NULL;
  }

  return mnote_panasonic_tag_get_name(static_cast<MnotePanasonicTag>(d->entries[i].tag));
}

static const char*
exif_mnote_data_panasonic_get_title(ExifMnoteData* md, unsigned int i)
{
  ExifMnoteDataIfd* d = (ExifMnoteDataIfd*)md;  // NOLINT

  if (!d || (d->count <= i))
  {
    return NULL;
  }

  return mnote_panasonic_tag_get_title(static_cast<MnotePanasonicTag>(d->entries[i].tag));
}

static const char*
exif_mnote_data_panasonic_get_description(ExifMnoteData* md, unsigned int i)
{
  ExifMnoteDataIfd* d = (ExifMnoteDataIfd*)md;  // NOLINT

  if (!d || (d->count <= i))
  {
    return NULL;
  }

  return mnote_panasonic_tag_get_description(static_cast<MnotePanasonicTag>(d->entries[i].tag));
}

static char*
exif_mnote_data_ifd_get_value(ExifMnoteData* md, unsigned int i, char* val, unsigned int maxlen)
{
  ExifMnoteDataIfd* d = (ExifMnoteDataIfd*)md;  // NOLINT

  if (!val || !d || (d->count <= i))
  {
    return NULL;
  }

  return mnote_ifd_entry_get_value(&d->entries[i], val, maxlen);
}

int
exif_mnote_data_apple_identify(const ExifData* ed, const ExifEntry* e)
{
  (void)ed;

  if (e->size < strlen("Apple iOS") + 1)
    return 0;

  return !memcmp((const char*)e->data, "Apple iOS", strlen("Apple iOS"));
}

int
exif_mnote_data_panasonic_identify(const ExifData* ed, const ExifEntry* e)
{
  (void)ed;

  if (e->size < strlen("Panasonic") + 1)
    return 0;

  return !memcmp((const char*)e->data, "Panasonic", strlen("Panasonic"));
}

ExifMnoteData*
exif_mnote_data_apple_new(ExifMem* mem)
{
  ExifMnoteData* md;

  /*printf("%s\n", __FUNCTION__);*/

  if (!mem)
  {
    return NULL;
  }

  md = static_cast<ExifMnoteData*>(exif_mem_alloc(mem, sizeof(ExifMnoteDataIfd)));
  if (!md)
  {
    return NULL;
  }

  exif_mnote_data_construct(md, mem);

  md->methods.free = exif_mnote_data_ifd_free;
  md->methods.load = exif_mnote_data_apple_load;
  md->methods.set_offset = exif_mnote_data_ifd_set_offset;
  md->methods.set_byte_order = exif_mnote_data_ifd_set_byte_order;
  md->methods.count = exif_mnote_data_ifd_count;
  md->methods.get_id = exif_mnote_data_ifd_get_id;
  md->methods.get_name = exif_mnote_data_apple_get_name;
  md->methods.get_title = exif_mnote_data_apple_get_title;
  md->methods.get_description = exif_mnote_data_apple_get_description;
  md->methods.get_value = exif_mnote_data_ifd_get_value;

  return md;
}

ExifMnoteData*
exif_mnote_data_panasonic_new(ExifMem* mem)
{
  ExifMnoteData* md;

  /*printf("%s\n", __FUNCTION__);*/

  if (!mem)
  {
    return NULL;
  }

  md = static_cast<ExifMnoteData*>(exif_mem_alloc(mem, sizeof(ExifMnoteDataIfd)));
  if (!md)
  {
    return NULL;
  }

  exif_mnote_data_construct(md, mem);

  md->methods.free = exif_mnote_data_ifd_free;
  md->methods.load = exif_mnote_data_panasonic_load;
  md->methods.set_offset = exif_mnote_data_ifd_set_offset;
  md->methods.set_byte_order = exif_mnote_data_ifd_set_byte_order;
  md->methods.count = exif_mnote_data_ifd_count;
  md->methods.get_id = exif_mnote_data_ifd_get_id;
  md->methods.get_name = exif_mnote_data_panasonic_get_name;
  md->methods.get_title = exif_mnote_data_panasonic_get_title;
  md->methods.get_description = exif_mnote_data_panasonic_get_description;
  md->methods.get_value = exif_mnote_data_ifd_get_value;

  return md;
}

LibexifCustomMakernotes::LibexifCustomMakernotes()
{
  this->customMakerNotes["apple"] = {&exif_mnote_data_apple_identify, &exif_mnote_data_apple_new};
  this->customMakerNotes["panasonic"] = {&exif_mnote_data_panasonic_identify, &exif_mnote_data_panasonic_new};
}
}
