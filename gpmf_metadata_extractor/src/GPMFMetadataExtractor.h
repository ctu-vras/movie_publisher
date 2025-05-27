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

namespace movie_publisher
{

struct GPMFMetadataPrivate;

/**
 * \brief Metadata extractor from GPMF streams.
 *
 * The extractor reads the following ROS parameters:
 */
class GPMFMetadataExtractor : public TimedMetadataExtractor
{
public:
  /**
   * \brief Constructor.
   * \param[in] log Logger.
   * \param[in] manager Metadata manager.
   * \param[in] info Movie info.
   * \param[in] config Movie open config.
   * \param[in] avFormatContext Libav context of the opened video file.
   * \param[in] priority Priority of the extractor.
   */
  explicit GPMFMetadataExtractor(
    const cras::LogHelperPtr& log, const std::weak_ptr<MetadataManager>& manager, const MovieInfo::ConstPtr& info,
    const MovieOpenConfig& config, const AVFormatContext* avFormatContext, int priority);
  ~GPMFMetadataExtractor() override;

  std::string getName() const override;
  int getPriority() const override;

  cras::optional<std::string> getCameraMake() override;
  cras::optional<std::string> getCameraModel() override;
  cras::optional<std::string> getCameraSerialNumber() override;
  cras::optional<std::string> getLensMake() override;
  cras::optional<std::string> getLensModel() override;
  cras::optional<ros::Time> getCreationTime() override;

  cras::optional<double> getCropFactor() override;
  cras::optional<std::pair<double, double>> getSensorSizeMM() override;
  cras::optional<std::pair<DistortionType, Distortion>> getDistortion() override;
  cras::optional<int> getRotation() override;
  cras::optional<double> getFocalLengthMM() override;
  cras::optional<double> getFocalLength35MM() override;
  cras::optional<double> getFocalLengthPx() override;
  cras::optional<IntrinsicMatrix> getIntrinsicMatrix() override;
  cras::optional<compass_msgs::Azimuth> getAzimuth() override;
  cras::optional<std::pair<double, double>> getRollPitch() override;
  GNSSFixAndDetail getGNSSPosition() override;
  cras::optional<geometry_msgs::Vector3> getAcceleration() override;
  cras::optional<sensor_msgs::MagneticField> getMagneticField() override;
  cras::optional<geometry_msgs::Vector3> getAngularVelocity() override;
  cras::optional<vision_msgs::Detection2DArray> getFaces() override;

  size_t processTimedMetadata(MetadataType type, const StreamTime& maxTime, bool requireOptional) override;
  void seekTimedMetadata(const StreamTime& seekTime) override;
  bool hasTimedMetadata() const override;
  std::unordered_set<MetadataType> supportedTimedMetadata(
    const std::unordered_set<MetadataType>& availableMetadata) const override;
  void prepareTimedMetadata(const std::unordered_set<MetadataType>& types) override;
  void processPacket(const AVPacket* packet) override;

private:
  std::unique_ptr<GPMFMetadataPrivate> data;  //!< PIMPL
};

/**
 * \brief Plugin for instantiating GPMFMetadataExtractor.
 */
struct GPMFMetadataExtractorPlugin final : MetadataExtractorPlugin
{
  MetadataExtractor::Ptr getExtractor(const MetadataExtractorParams& params) override;
};

}
