// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Extractor of metadata from camera info.
 * \author Martin Pecka
 */

#pragma once

#include <list>
#include <memory>
#include <string>
#include <utility>

#include <cras_cpp_common/optional.hpp>
#include <movie_publisher/metadata_extractor.h>
#include <movie_publisher/metadata_manager.h>

namespace movie_publisher
{

struct CamInfoManagerMetadataPrivate;

/**
 * \brief Metadata extractor that matches existing camera info to a particular camera and lens.
 *
 * The extractor reads the following ROS parameters:
 * - `~calibration_urls` (string[], optional): Calibration URLs this extractor searches in order to find camera info.
 */
class CamInfoManagerMetadataExtractor : public MetadataExtractor
{
public:
  /**
   * Constructor.
   * \param[in] log Logger.
   * \param[in] manager Metadata manager.
   * \param[in] width Width of the movie [px].
   * \param[in] height Height of the movie [px].
   * \param[in] calibrationURLs Calibration URLs this extractor searches in order to find camera info.
   */
  explicit CamInfoManagerMetadataExtractor(
    const cras::LogHelperPtr& log, const std::weak_ptr<MetadataManager>& manager, size_t width, size_t height,
    const std::list<std::string>& calibrationURLs);
  ~CamInfoManagerMetadataExtractor() override;

  std::string getName() const override;
  int getPriority() const override;
  cras::optional<CI::_K_type> getIntrinsicMatrix() override;
  cras::optional<std::pair<CI::_distortion_model_type, CI::_D_type>> getDistortion() override;

private:
  std::unique_ptr<CamInfoManagerMetadataPrivate> data;  //!< PIMPL
};

/**
 * \brief Plugin for instantiating CameraInfoManagerMetadataExtractor.
 */
struct CamInfoManagerMetadataExtractorPlugin : MetadataExtractorPlugin
{
  MetadataExtractor::Ptr getExtractor(const MetadataExtractorParams& params) override;
};

}
