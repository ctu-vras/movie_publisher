// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief
 * \author Martin Pecka
 */

#pragma once

#include <string>

#include <cras_cpp_common/optional.hpp>
#include <movie_publisher/metadata_extractor.h>
#include <ros/time.h>

namespace movie_publisher
{

class FilenameMetadataExtractor : public MetadataExtractor
{
public:
  FilenameMetadataExtractor(const cras::LogHelperPtr& log, const std::string& filename);
  std::string getName() const override;
  int getPriority() const override;
  cras::optional<ros::Time> getCreationTime() override;

private:
  std::string filename;
};

struct FilenameMetadataExtractorPlugin : MetadataExtractorPlugin
{
  MetadataExtractor::Ptr getExtractor(const MetadataExtractorParams& params) override;
};

}
