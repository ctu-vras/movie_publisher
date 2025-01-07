// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Metadata extractor using libexif backend.
 * \author Martin Pecka
 */

#pragma once

#include <memory>
#include <string>

#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/log_utils.h>
#include <movie_publisher/metadata_manager.h>
#include <movie_publisher/metadata/ExifBaseMetadataExtractor.h>

namespace movie_publisher
{

struct LibexifMetadataPrivate;

/**
 * \brief Metadata extractor using libexif backend.
 */
class LibexifMetadataExtractor : public ExifBaseMetadataExtractor
{
public:
  /**
   * \brief Constructor.
   * \param[in] log Logger.
   * \param[in] manager Metadata manager.
   * \param[in] filename Filename of the movie.
   * \param[in] width Width of the movie [px].
   * \param[in] height Height of the movie [px].
   */
  LibexifMetadataExtractor(
    const cras::LogHelperPtr& log, const std::weak_ptr<MetadataManager>& manager,
    const std::string& filename, size_t width, size_t height);
  ~LibexifMetadataExtractor() override;

  std::string getName() const override;
  int getPriority() const override;

protected:
  cras::optional<ExifData<ExifAscii>> getExifMake() override;
  cras::optional<ExifData<ExifAscii>> getExifModel() override;
  cras::optional<ExifData<ExifAscii>> getExifLensMake() override;
  cras::optional<ExifData<ExifAscii>> getExifLensModel() override;
  cras::optional<ExifData<ExifAscii>> getExifBodySerialNumber() override;
  cras::optional<ExifData<ExifAscii>> getExifLensSerialNumber() override;
  cras::optional<ExifData<ExifAscii>> getExifDateTimeOriginal() override;
  cras::optional<ExifData<ExifAscii>> getExifOffsetTimeOriginal() override;
  cras::optional<ExifData<ExifAscii>> getExifSubSecTimeOriginal() override;
  cras::optional<ExifData<ExifShort>> getExifOrientation() override;
  cras::optional<ExifData<ExifRational>> getExifFocalPlaneXRes() override;
  cras::optional<ExifData<ExifRational>> getExifFocalPlaneYRes() override;
  cras::optional<ExifData<ExifShort>> getExifFocalPlaneResUnit() override;
  cras::optional<ExifData<ExifShort>> getExifResUnit() override;
  cras::optional<ExifData<ExifShort>> getExifFocalLength35MM() override;
  cras::optional<ExifData<ExifRational>> getExifFocalLength() override;
  cras::optional<ExifData<ExifAscii>> getExifGpsLatRef() override;
  cras::optional<ExifData<ExifRational>> getExifGpsLat(size_t n) override;
  cras::optional<ExifData<ExifAscii>> getExifGpsLonRef() override;
  cras::optional<ExifData<ExifRational>> getExifGpsLon(size_t n) override;
  cras::optional<ExifData<ExifByte>> getExifGpsAltRef() override;
  cras::optional<ExifData<ExifRational>> getExifGpsAlt() override;
  cras::optional<ExifData<ExifAscii>> getExifGpsMeasureMode() override;
  cras::optional<ExifData<ExifRational>> getExifGpsDOP() override;
  cras::optional<ExifData<ExifAscii>> getExifGpsSpeedRef() override;
  cras::optional<ExifData<ExifRational>> getExifGpsSpeed() override;
  cras::optional<ExifData<ExifAscii>> getExifGpsTrackRef() override;
  cras::optional<ExifData<ExifRational>> getExifGpsTrack() override;
  cras::optional<ExifData<ExifRational>> getExifGpsTimeStamp(size_t n) override;
  cras::optional<ExifData<ExifAscii>> getExifGpsDateStamp() override;
  cras::optional<ExifData<ExifShort>> getExifGpsDifferential() override;
  cras::optional<ExifData<ExifRational>> getExifGpsHPositioningError() override;
  cras::optional<ExifData<ExifAscii>> getExifGpsImgDirectionRef() override;
  cras::optional<ExifData<ExifRational>> getExifGpsImgDirection() override;
  cras::optional<ExifData<ExifSRational>> getExifAcceleration(size_t n) override;
  cras::optional<ExifData<ExifSRational>> getExifRollAngle() override;
  cras::optional<ExifData<ExifSRational>> getExifPitchAngle() override;

private:
  std::unique_ptr<LibexifMetadataPrivate> data;
};

/**
 * \brief Plugin for instantiating LibexifMetadataExtractor.
 */
struct LibexifMetadataExtractorPlugin : MetadataExtractorPlugin
{
  MetadataExtractor::Ptr getExtractor(const MetadataExtractorParams& params) override;
};

}
