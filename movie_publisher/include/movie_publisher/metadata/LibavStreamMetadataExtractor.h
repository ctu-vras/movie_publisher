// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Extractor of metadata from an open LibAV stream.
 * \author Martin Pecka
 */

#pragma once

#include <memory>
#include <string>

#include <cras_cpp_common/optional.hpp>
#include <movie_publisher/metadata_extractor.h>
#include <ros/time.h>

struct AVFormatContext;

namespace movie_publisher
{

struct LibavStreamMetadataPrivate;

/**
 * \brief Extractor of metadata from an open LibAV stream.
 */
class LibavStreamMetadataExtractor : public MetadataExtractor
{
public:
  /**
   * \brief Constructor.
   * \param[in] log Logger.
   * \param[in] avFormatContext LibAV context of the open movie.
   * \param[in] streamIndex Index of the stream in the open movie.
   */
  LibavStreamMetadataExtractor(
    const cras::LogHelperPtr& log, const AVFormatContext* avFormatContext, size_t streamIndex);
  ~LibavStreamMetadataExtractor() override;

  std::string getName() const override;
  int getPriority() const override;
  cras::optional<std::string> getCameraMake() override;
  cras::optional<std::string> getCameraModel() override;
  cras::optional<ros::Time> getCreationTime() override;
  cras::optional<int> getRotation() override;
  std::pair<cras::optional<sensor_msgs::NavSatFix>, cras::optional<gps_common::GPSFix>> getGNSSPosition() override;

private:
  std::unique_ptr<LibavStreamMetadataPrivate> data;  //!< PIMPL
};

/**
 * \brief Loader plugin for LibabStreamMetadataExtractor.
 */
struct LibavStreamMetadataExtractorPlugin : MetadataExtractorPlugin
{
  MetadataExtractor::Ptr getExtractor(const MetadataExtractorParams& params) override;
};

}
