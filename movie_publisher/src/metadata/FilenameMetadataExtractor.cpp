// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Extractor of metadata from filename.
 * \author Martin Pecka
 */

#include <regex>

#include <cras_cpp_common/type_utils.hpp>
#include <movie_publisher/metadata/FilenameMetadataExtractor.h>
#include <pluginlib/class_list_macros.h>

#include CXX_FILESYSTEM_INCLUDE
namespace fs = CXX_FILESYSTEM_NAMESPACE;


namespace movie_publisher
{

FilenameMetadataExtractor::FilenameMetadataExtractor(const cras::LogHelperPtr& log, const std::string& filename)
  : MetadataExtractor(log), filename(filename)
{
}

std::string FilenameMetadataExtractor::getName() const
{
  return cras::getTypeName<std::remove_cv_t<std::remove_reference_t<decltype(*this)>>>();
}

int FilenameMetadataExtractor::getPriority() const
{
  return 95;
}

cras::optional<ros::Time> FilenameMetadataExtractor::getCreationTime()
{
  std::regex timeRegex {R"(((?:19|20|21)\d{2}).?([01]\d).?([0123]\d).?([012]\d).?([0-5]\d).?([0-5]\d))"};
  std::smatch matches;
  const auto basename = fs::path(this->filename).filename().string();
  if (std::regex_search(basename, matches, timeRegex))
  {
    try
    {
      const auto time = cras::parseTime(matches[0].str());
      CRAS_DEBUG("Creation time read from filename (%s).", matches[0].str().c_str());
      return time;
    }
    catch (const std::invalid_argument&) {}
  }

  return cras::nullopt;
}

MetadataExtractor::Ptr FilenameMetadataExtractorPlugin::getExtractor(const MetadataExtractorParams& params)
{
  if (params.log == nullptr || params.filename.empty())
    return nullptr;

  return std::make_shared<FilenameMetadataExtractor>(params.log, params.filename);
}

}

PLUGINLIB_EXPORT_CLASS(movie_publisher::FilenameMetadataExtractorPlugin, movie_publisher::MetadataExtractorPlugin)
