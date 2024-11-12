// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief
 * \author Martin Pecka
 */

#pragma once

#include <memory>
#include <string>

#include <cras_cpp_common/optional.hpp>
#include <movie_publisher/metadata_extractor.h>
#include <movie_publisher/metadata_manager.h>

struct lfDatabase;

namespace movie_publisher
{

struct LensfunMetadataPrivate;

class LensfunMetadataExtractor : public MetadataExtractor
{
public:
  explicit LensfunMetadataExtractor(
    const cras::LogHelperPtr& log, const std::weak_ptr<MetadataManager>& manager, size_t width, size_t height);
  ~LensfunMetadataExtractor() override;
  void warnIfDbOld(lfDatabase* db);

  std::string getName() const override;
  int getPriority() const override;
  cras::optional<double> getCropFactor() override;
  cras::optional<std::pair<double, double>> getSensorSizeMM() override;
  cras::optional<double> getFocalLengthMM() override;
  cras::optional<std::pair<CI::_distortion_model_type, CI::_D_type>> getDistortion() override;

private:
  std::unique_ptr<LensfunMetadataPrivate> data;
};

struct LensfunMetadataExtractorPlugin : MetadataExtractorPlugin
{
  MetadataExtractor::Ptr getExtractor(const MetadataExtractorParams& params) override;
};

}
