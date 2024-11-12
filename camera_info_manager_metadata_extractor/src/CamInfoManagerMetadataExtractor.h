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

class CamInfoManagerMetadataExtractor : public MetadataExtractor
{
public:
  explicit CamInfoManagerMetadataExtractor(
    const cras::LogHelperPtr& log, const std::weak_ptr<MetadataManager>& manager,
    const std::list<std::string>& calibrationURLs);
  ~CamInfoManagerMetadataExtractor() override;

  std::string getName() const override;
  int getPriority() const override;
  cras::optional<CI::_K_type> getIntrinsicMatrix() override;
  cras::optional<std::pair<CI::_distortion_model_type, CI::_D_type>> getDistortion() override;

private:
  std::unique_ptr<CamInfoManagerMetadataPrivate> data;
};

struct CamInfoManagerMetadataExtractorPlugin : MetadataExtractorPlugin
{
  MetadataExtractor::Ptr getExtractor(const MetadataExtractorParams& params) override;
};

}
