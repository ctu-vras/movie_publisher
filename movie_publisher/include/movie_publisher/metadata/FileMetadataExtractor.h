// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Extractor of metadata from filesystem properties of the movie file.
 * \author Martin Pecka
 */

#pragma once

#include <string>

#include <cras_cpp_common/optional.hpp>
#include <movie_publisher/metadata_extractor.h>
#include <ros/time.h>

namespace movie_publisher
{
/**
 * \brief Extractor of metadata from filesystem properties of the movie file.
 */
class FileMetadataExtractor : public MetadataExtractor
{
public:
  /**
   * \brief Constructor.
   * \param[in] log Logger.
   * \param[in] filename Filename of the movie.
   */
  FileMetadataExtractor(const cras::LogHelperPtr& log, const std::string& filename);

  std::string getName() const override;
  int getPriority() const override;
  cras::optional<ros::Time> getCreationTime() override;

private:
  std::string filename;  //!< Filename of the movie.
};

/**
 * \brief Loader plugin for FileMetadataExtractor.
 */
struct FileMetadataExtractorPlugin : MetadataExtractorPlugin
{
  MetadataExtractor::Ptr getExtractor(const MetadataExtractorParams& params) override;
};

}
