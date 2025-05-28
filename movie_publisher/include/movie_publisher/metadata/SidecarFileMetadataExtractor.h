// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Extractor of metadata from sidecar YAML files.
 * \author Martin Pecka
 */

#pragma once

#include <memory>
#include <string>

#include <cras_cpp_common/optional.hpp>
#include <movie_publisher/metadata_extractor.h>
#include <ros/time.h>

namespace movie_publisher
{

/**
 * \brief Extractor of metadata from sidecar YAML files.
 */
class SidecarFileMetadataExtractor : public TimedMetadataExtractor
{
public:
  /**
   * \brief Constructor.
   * \param[in] log Logger.
   * \param[in] filename Filename of the movie.
   * \param[in] info Movie info.
   * \param[in] manager The metadata manager.
   */
  SidecarFileMetadataExtractor(const cras::LogHelperPtr& log, const std::string& filename,
    const MovieInfo::ConstPtr& info, const std::weak_ptr<MetadataManager>& manager);

  std::string getName() const override;
  int getPriority() const override;

  cras::optional<std::string> getCameraGeneralName() override;
  cras::optional<std::string> getCameraUniqueName() override;
  cras::optional<std::string> getCameraSerialNumber() override;
  cras::optional<std::string> getCameraMake() override;
  cras::optional<std::string> getCameraModel() override;
  cras::optional<std::string> getLensMake() override;
  cras::optional<std::string> getLensModel() override;
  cras::optional<int> getRotation() override;
  cras::optional<ros::Time> getCreationTime() override;
  cras::optional<double> getCropFactor() override;
  cras::optional<SensorSize> getSensorSizeMM() override;
  cras::optional<double> getFocalLength35MM() override;
  cras::optional<double> getFocalLengthMM() override;
  cras::optional<double> getFocalLengthPx() override;
  cras::optional<IntrinsicMatrix> getIntrinsicMatrix() override;
  cras::optional<DistortionData> getDistortion() override;
  GNSSFixAndDetail getGNSSPosition() override;
  cras::optional<compass_msgs::Azimuth> getAzimuth() override;
  cras::optional<sensor_msgs::MagneticField> getMagneticField() override;
  cras::optional<RollPitch> getRollPitch() override;
  cras::optional<geometry_msgs::Vector3> getAcceleration() override;
  cras::optional<geometry_msgs::Vector3> getAngularVelocity() override;
  cras::optional<vision_msgs::Detection2DArray> getFaces() override;
  cras::optional<sensor_msgs::CameraInfo> getCameraInfo() override;
  cras::optional<sensor_msgs::Imu> getImu() override;
  cras::optional<geometry_msgs::Transform> getOpticalFrameTF() override;
  cras::optional<geometry_msgs::Transform> getZeroRollPitchTF() override;

  void prepareTimedMetadata(const std::unordered_set<MetadataType>& metadataTypes) override;
  std::unordered_set<MetadataType> supportedTimedMetadata(
    const std::unordered_set<MetadataType>& availableMetadata) const override;
  size_t processTimedMetadata(MetadataType type, const StreamTime& maxTime, bool requireOptional) override;
  void seekTimedMetadata(const StreamTime& seekTime) override;
  bool hasTimedMetadata() const override;

private:
  struct Impl;
  std::unique_ptr<Impl> data;  //!< PIMPL
};

/**
 * \brief Loader plugin for FileMetadataExtractor.
 */
struct SidecarFileMetadataExtractorPlugin : MetadataExtractorPlugin
{
  MetadataExtractor::Ptr getExtractor(const MetadataExtractorParams& params) override;
};

}
