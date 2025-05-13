// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Extractor of metadata from filesystem properties of the movie file.
 * \author Martin Pecka
 */

#include <sys/stat.h>

#include <cras_cpp_common/type_utils.hpp>
#include <movie_publisher/metadata/FileMetadataExtractor.h>
#include <pluginlib/class_list_macros.h>

namespace movie_publisher
{

FileMetadataExtractor::FileMetadataExtractor(const cras::LogHelperPtr& log, const std::string& filename)
  : MetadataExtractor(log), filename(filename)
{
}

std::string FileMetadataExtractor::getName() const
{
  return cras::getTypeName<std::remove_cv_t<std::remove_reference_t<decltype(*this)>>>();
}

int FileMetadataExtractor::getPriority() const
{
  return 90;
}

cras::optional<ros::Time> FileMetadataExtractor::getCreationTime()
{
  struct stat fileStat{};
  if (stat(this->filename.c_str(), &fileStat) != 0)
    return cras::nullopt;

  if (fileStat.st_ctim.tv_sec != 0)
  {
    CRAS_DEBUG("Creation time read from file create time.");
    return ros::Time(fileStat.st_ctim.tv_sec, fileStat.st_ctim.tv_nsec);
  }
  else
  {
    CRAS_DEBUG("Creation time read from file modification time.");
    return ros::Time(fileStat.st_mtim.tv_sec, fileStat.st_mtim.tv_nsec);
  }
}

MetadataExtractor::Ptr FileMetadataExtractorPlugin::getExtractor(const MetadataExtractorParams& params)
{
  if (params.log == nullptr || params.info->filenameOrURL().empty())
    return nullptr;

  return std::make_shared<FileMetadataExtractor>(params.log, params.info->filenameOrURL());
}

}

PLUGINLIB_EXPORT_CLASS(movie_publisher::FileMetadataExtractorPlugin, movie_publisher::MetadataExtractorPlugin)
