// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Metadata extractor from GPMF streams.
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

struct GPMFMetadataPrivate;

/**
 * \brief Metadata extractor from GPMF streams.
 *
 * The extractor reads the following ROS parameters:
 */
class GPMFMetadataExtractor : public MetadataExtractor
{
public:
  /**
   * \brief Constructor.
   * \param[in] log Logger.
   * \param[in] manager Metadata manager.
   * \param[in] width Width of the movie [px].
   * \param[in] height Height of the movie [px].
   * \param[in] isStillImage Whether the movie is a still image (just one frame) or not.
   * \param[in] avFormatContext Libav context of the opened video file.
   * \param[in] videoStreamIndex Index of the video stream.
   */
  explicit GPMFMetadataExtractor(
    const cras::LogHelperPtr& log, const std::weak_ptr<MetadataManager>& manager, size_t width, size_t height,
    bool isStillImage, const AVFormatContext* avFormatContext, const size_t videoStreamIndex);
  ~GPMFMetadataExtractor() override;

  std::string getName() const override;
  int getPriority() const override;
  cras::optional<double> getCropFactor() override;
  cras::optional<std::pair<double, double>> getSensorSizeMM() override;
  cras::optional<double> getFocalLengthMM() override;
  cras::optional<std::pair<CI::_distortion_model_type, CI::_D_type>> getDistortion() override;
  cras::optional<std::string> getCameraSerialNumber() override;
  cras::optional<std::string> getCameraMake() override;
  cras::optional<std::string> getCameraModel() override;
  cras::optional<std::string> getLensMake() override;
  cras::optional<std::string> getLensModel() override;
  cras::optional<int> getRotation() override;
  cras::optional<ros::Time> getCreationTime() override;
  cras::optional<double> getFocalLength35MM() override;
  cras::optional<double> getFocalLengthPx() override;
  cras::optional<CI::_K_type> getIntrinsicMatrix() override;
  std::pair<cras::optional<sensor_msgs::NavSatFix>, cras::optional<gps_common::GPSFix>> getGNSSPosition() override;
  cras::optional<compass_msgs::Azimuth> getAzimuth() override;
  cras::optional<std::pair<double, double>> getRollPitch() override;
  cras::optional<geometry_msgs::Vector3> getAcceleration() override;

private:
  std::unique_ptr<GPMFMetadataPrivate> data;  //!< PIMPL
};

/**
 * \brief Plugin for instantiating GPMFMetadataExtractor.
 */
struct GPMFMetadataExtractorPlugin : MetadataExtractorPlugin
{
  MetadataExtractor::Ptr getExtractor(const MetadataExtractorParams& params) override;
};

}
