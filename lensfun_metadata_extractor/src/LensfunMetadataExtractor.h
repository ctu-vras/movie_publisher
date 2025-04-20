// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Metadata extractor using lensfun backend.
 * \author Martin Pecka
 */

#pragma once

#include <memory>
#include <string>
#include <utility>

#include <cras_cpp_common/optional.hpp>
#include <movie_publisher/metadata_extractor.h>
#include <movie_publisher/metadata_manager.h>

struct lfDatabase;

namespace movie_publisher
{

struct LensfunMetadataPrivate;

/**
 * \brief Metadata extractor using lensfun backend.
 *
 * The extractor reads the following ROS parameters:
 * - `~lensfun_extra_db` (string, optional): If nonempty, the specified file or directory will be loaded as an
 *                                           additional lensfun database directory.
 */
class LensfunMetadataExtractor : public MetadataExtractor
{
public:
  /**
   * \brief Constructor.
   * \param[in] log Logger.
   * \param[in] manager Metadata manager.
   * \param[in] width Width of the movie [px].
   * \param[in] height Height of the movie [px].
   * \param[in] isStillImage Whether the movie is a still image (just one frame) or not.
   * \param[in] extraDb If nonempty, the specified file or directory will be loaded as an additional lensfun database
   *                    directory.
   */
  explicit LensfunMetadataExtractor(
    const cras::LogHelperPtr& log, const std::weak_ptr<MetadataManager>& manager, size_t width, size_t height,
    bool isStillImage, const std::string& extraDb = "");
  ~LensfunMetadataExtractor() override;

  /**
   * \brief Print a warning if the lensfun database is considered old and should be updated.
   * \param[in] db The lensfun database.
   */
  void warnIfDbOld(lfDatabase* db);

  std::string getName() const override;
  int getPriority() const override;
  cras::optional<double> getCropFactor() override;
  cras::optional<std::pair<double, double>> getSensorSizeMM() override;
  cras::optional<double> getFocalLengthMM() override;
  cras::optional<std::pair<DistortionType, Distortion>> getDistortion() override;

private:
  std::unique_ptr<LensfunMetadataPrivate> data;  //!< PIMPL
};

/**
 * \brief Plugin for instantiating LensfunMetadataExtractor.
 */
struct LensfunMetadataExtractorPlugin : MetadataExtractorPlugin
{
  MetadataExtractor::Ptr getExtractor(const MetadataExtractorParams& params) override;
};

}
