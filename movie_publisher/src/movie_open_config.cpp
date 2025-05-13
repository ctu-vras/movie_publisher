// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Configuration specifying what movie file to open and how.
 * \author Martin Pecka
 */

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <movie_publisher/metadata_type.h>
#include <movie_publisher/movie_metadata_processor.h>
#include <movie_publisher/movie_open_config.h>
#include <movie_publisher/parsing_utils.h>
#include <ros/duration.h>
#include <ros/common.h>
#include <sensor_msgs/image_encodings.h>

#if ROS_VERSION_MINIMUM(1, 17, 0)
#define MAGIC_ENUM_USING_ALIAS_OPTIONAL template <typename T> using optional = cras::optional<T>;
#include <magic_enum.hpp>
#endif

namespace movie_publisher
{

/**
 * \brief Configuration specifying what movie file to open and how.
 */
struct MovieOpenConfig::Impl
{
  std::string filenameOrURL {};  //!< The name of the file with the movie, or its URL.

  bool extractMetadata {true};  //!< Whether non-essential metadata should be extracted.
  std::vector<std::shared_ptr<MovieMetadataProcessor>> metadataProcessors;  //!< Metadata processors.

  //! \brief Default encoding to use if the pixel format has no corresponding ROS color encoding.
  std::string defaultEncoding {sensor_msgs::image_encodings::BGR8};
  cras::optional<std::string> forceEncoding;  //!< If set, this encoding will be forced.
  bool allowYUVFallback{true};  //!< Allow falling back to YUV encodings if the pixel format has no ROS encoding.
  //! If set, forces to read the given stream index instead of the automatically selected one.
  cras::optional<int> forceStreamIndex;
  size_t numThreads {1};  //!< Number of video decoding threads.

  std::string frameId;  //!< ID of the geometrical camera frame (used for position/orientation metadata).
  std::string opticalFrameId;  //!< ID of the optical camera frame (used for images).

  TimestampSource timestampSource {TimestampSource::RosTime};  //!< How to extract timestamps.
  ros::Duration timestampOffset;  //!< Optional offset to add to the extracted timestamps.

  cras::BoundParamHelperPtr rosParams {};  //!< ROS/YAML parameters that configure the reader and metadata extractors.

  std::unordered_set<MetadataType> metadataTypes {};  //!< Types of metadata to be extracted. Defaults to all.
};

MovieOpenConfig::MovieOpenConfig(const cras::BoundParamHelperPtr& rosParams) : data(new Impl())
{
#if ROS_VERSION_MINIMUM(1, 17, 0)
  const auto allMetadataTypes = magic_enum::enum_values<MetadataType>();
  this->data->metadataTypes.insert(allMetadataTypes.begin(), allMetadataTypes.end());
#else
  this->data->metadataTypes.insert(MetadataType::CAMERA_GENERAL_NAME);
  this->data->metadataTypes.insert(MetadataType::CAMERA_UNIQUE_NAME);
  this->data->metadataTypes.insert(MetadataType::CAMERA_SERIAL_NUMBER);
  this->data->metadataTypes.insert(MetadataType::CAMERA_MAKE);
  this->data->metadataTypes.insert(MetadataType::CAMERA_MODEL);
  this->data->metadataTypes.insert(MetadataType::LENS_MAKE);
  this->data->metadataTypes.insert(MetadataType::LENS_MODEL);
  this->data->metadataTypes.insert(MetadataType::CREATION_TIME);
  this->data->metadataTypes.insert(MetadataType::ROTATION);
  this->data->metadataTypes.insert(MetadataType::CROP_FACTOR);
  this->data->metadataTypes.insert(MetadataType::SENSOR_SIZE_MM);
  this->data->metadataTypes.insert(MetadataType::FOCAL_LENGTH_35MM);
  this->data->metadataTypes.insert(MetadataType::FOCAL_LENGTH_MM);
  this->data->metadataTypes.insert(MetadataType::FOCAL_LENGTH_PX);
  this->data->metadataTypes.insert(MetadataType::INTRINSIC_MATRIX);
  this->data->metadataTypes.insert(MetadataType::DISTORTION);
  this->data->metadataTypes.insert(MetadataType::GNSS_POSITION);
  this->data->metadataTypes.insert(MetadataType::AZIMUTH);
  this->data->metadataTypes.insert(MetadataType::MAGNETIC_FIELD);
  this->data->metadataTypes.insert(MetadataType::ROLL_PITCH);
  this->data->metadataTypes.insert(MetadataType::ACCELERATION);
  this->data->metadataTypes.insert(MetadataType::ANGULAR_VELOCITY);
  this->data->metadataTypes.insert(MetadataType::FACES);
  this->data->metadataTypes.insert(MetadataType::CAMERA_INFO);
  this->data->metadataTypes.insert(MetadataType::IMU);
  this->data->metadataTypes.insert(MetadataType::OPTICAL_FRAME_TF);
  this->data->metadataTypes.insert(MetadataType::ZERO_ROLL_PITCH_TF);
#endif
  this->data->rosParams = rosParams;
}

MovieOpenConfig::~MovieOpenConfig() = default;

MovieOpenConfig::MovieOpenConfig(const MovieOpenConfig& other) : MovieOpenConfig(other.data->rosParams)
{
  *this->data = *other.data;
}

MovieOpenConfig& MovieOpenConfig::operator=(const MovieOpenConfig& other)
{
  if (&other == this)
    return *this;
  *this->data = *other.data;
  return *this;
}

MovieOpenConfig::MovieOpenConfig(MovieOpenConfig&& other) noexcept
{
  this->data = std::move(other.data);
}

MovieOpenConfig& MovieOpenConfig::operator=(MovieOpenConfig&& other) noexcept
{
  this->data = std::move(other.data);
  return *this;
}

std::string MovieOpenConfig::filenameOrURL() const
{
  return this->data->filenameOrURL;
}

cras::expected<void, std::string> MovieOpenConfig::setFilenameOrURL(const std::string& data)
{
  this->data->filenameOrURL = data;
  return {};
}

bool MovieOpenConfig::extractMetadata() const
{
  return this->data->extractMetadata;
}

cras::expected<void, std::string> MovieOpenConfig::setExtractMetadata(const bool data)
{
  this->data->extractMetadata = data;
  return {};
}

const std::vector<std::shared_ptr<MovieMetadataProcessor>>& MovieOpenConfig::metadataProcessors() const
{
  return this->data->metadataProcessors;
}
std::vector<std::shared_ptr<MovieMetadataProcessor>>& MovieOpenConfig::metadataProcessors()
{
  return this->data->metadataProcessors;
}

cras::expected<void, std::string> MovieOpenConfig::setMetadataProcessors(
  const std::vector<std::shared_ptr<MovieMetadataProcessor>>& data)
{
  this->data->metadataProcessors = data;
  return {};
}

std::string MovieOpenConfig::defaultEncoding() const
{
  return this->data->defaultEncoding;
}

cras::expected<void, std::string> MovieOpenConfig::setDefaultEncoding(const std::string& data)
{
  if (!rosEncodingToAvPixFmt(data).has_value())
  {
    return cras::make_unexpected(cras::format(
      "Default encoding has to be either a color encoding, mono encoding or yuv422, but %s was given. "
      "The default encoding %s is not changing.", data.c_str(),
      this->data->defaultEncoding.c_str()));
  }

  this->data->defaultEncoding = data;
  return {};
}

cras::optional<std::string> MovieOpenConfig::forceEncoding() const
{
  return this->data->forceEncoding;
}

cras::expected<void, std::string> MovieOpenConfig::setForceEncoding(const cras::optional<std::string>& data)
{
  if (!data.has_value() || data->empty())
  {
    this->data->forceEncoding.reset();
    return {};
  }

  if (!rosEncodingToAvPixFmt(*data).has_value())
  {
    return cras::make_unexpected(cras::format(
      "Forced encoding has to be either a color encoding, mono encoding or yuv422, but %s was given. "
      "Not forcing the given encoding.", data->c_str()));
  }

  this->data->forceEncoding = data;
  return {};
}

bool MovieOpenConfig::allowYUVFallback() const
{
  return this->data->allowYUVFallback;
}

cras::expected<void, std::string> MovieOpenConfig::setAllowYUVFallback(const bool data)
{
  this->data->allowYUVFallback = data;
  return {};
}

cras::optional<int> MovieOpenConfig::forceStreamIndex() const
{
  return this->data->forceStreamIndex;
}

cras::expected<void, std::string> MovieOpenConfig::setForceStreamIndex(const cras::optional<int>& data)
{
  if (!data.has_value() || *data < 0)
    this->data->forceStreamIndex.reset();
  else
    this->data->forceStreamIndex = *data;
  return {};
}

std::string MovieOpenConfig::frameId() const
{
  return this->data->frameId;
}

cras::expected<void, std::string> MovieOpenConfig::setFrameId(const std::string& data)
{
  this->data->frameId = data;
  return {};
}

std::string MovieOpenConfig::opticalFrameId() const
{
  if (this->data->opticalFrameId.empty())
    return this->data->frameId;
  return this->data->opticalFrameId;
}

cras::expected<void, std::string> MovieOpenConfig::setOpticalFrameId(const std::string& data)
{
  this->data->opticalFrameId = data;
  return {};
}

TimestampSource MovieOpenConfig::timestampSource() const
{
  return this->data->timestampSource;
}

cras::expected<void, std::string> MovieOpenConfig::setTimestampSource(const TimestampSource data)
{
  this->data->timestampSource = data;
  return {};
}

ros::Duration MovieOpenConfig::timestampOffset() const
{
  return this->data->timestampOffset;
}

cras::expected<void, std::string> MovieOpenConfig::setTimestampOffset(const ros::Duration& data)
{
  this->data->timestampOffset = data;
  return {};
}

size_t MovieOpenConfig::numThreads() const
{
  return this->data->numThreads;
}

cras::expected<void, std::string> MovieOpenConfig::setNumThreads(const size_t data)
{
  this->data->numThreads = data;
  return {};
}

cras::BoundParamHelperPtr MovieOpenConfig::rosParams() const
{
  return this->data->rosParams;
}

cras::expected<void, std::string> MovieOpenConfig::setRosParams(const cras::BoundParamHelperPtr& data)
{
  this->data->rosParams = data;
  return {};
}

std::unordered_set<MetadataType> MovieOpenConfig::metadataTypes() const
{
  return this->data->metadataTypes;
}

cras::expected<void, std::string> MovieOpenConfig::setMetadataTypes(const std::unordered_set<MetadataType>& types)
{
  this->data->metadataTypes = types;
  return {};
}

}
