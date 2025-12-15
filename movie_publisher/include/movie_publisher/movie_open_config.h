// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Configuration specifying what movie file to open and how.
 * \author Martin Pecka
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <cras_cpp_common/expected.hpp>
#include <movie_publisher/metadata_type.h>
#include <ros/duration.h>

namespace movie_publisher
{

class MovieMetadataProcessor;

/**
 * \brief Configuration specifying what movie file to open and how.
 */
struct MovieOpenConfig final
{
  explicit MovieOpenConfig(const cras::BoundParamHelperPtr& rosParams);
  ~MovieOpenConfig();
  MovieOpenConfig(const MovieOpenConfig& other);
  MovieOpenConfig& operator=(const MovieOpenConfig& other);
  MovieOpenConfig(MovieOpenConfig&& other) noexcept;
  MovieOpenConfig& operator=(MovieOpenConfig&& other) noexcept;

  /**
   * \return The name of the file with the movie, or its URL.
   */
  std::string filenameOrURL() const;
  /**
   * \param[in] data The name of the file with the movie, or its URL.
   * \return Nothing or error string.
   */
  cras::expected<void, std::string> setFilenameOrURL(const std::string& data);

  /**
   * \return Whether non-essential metadata should be extracted.
   */
  bool extractMetadata() const;
  /**
   * \param[in] data Whether non-essential metadata should be extracted.
   * \return Nothing or error string.
   */
  cras::expected<void, std::string> setExtractMetadata(bool data);
  /**
   * \return Metadata processors.
   */
  const std::vector<std::shared_ptr<MovieMetadataProcessor>>& metadataProcessors() const;
  /**
   * \return Metadata processors.
   */
  std::vector<std::shared_ptr<MovieMetadataProcessor>>& metadataProcessors();
  /**
   * \param[in] data Metadata processors.
   * \return Nothing or error string.
   */
  cras::expected<void, std::string> setMetadataProcessors(
    const std::vector<std::shared_ptr<MovieMetadataProcessor>>& data);

  /**
   * \return Default encoding to use if the pixel format has no corresponding ROS color encoding.
   */
  std::string defaultEncoding() const;
  /**
   * \param[in] data Default encoding to use if the pixel format has no corresponding ROS color encoding.
   * \return Nothing or error string.
   */
  cras::expected<void, std::string> setDefaultEncoding(const std::string& data);
  /**
   * \return If set, this encoding will be forced.
   */
  cras::optional<std::string> forceEncoding() const;
  /**
   * \param[in] data If set, this encoding will be forced.
   * \return Nothing or error string.
   */
  cras::expected<void, std::string> setForceEncoding(const cras::optional<std::string>& data);
  /**
   * \return Allow falling back to YUV encodings if the pixel format has no ROS encoding.
   */
  bool allowYUVFallback() const;
  /**
   * \param[in] data Allow falling back to YUV encodings if the pixel format has no ROS encoding.
   * \return Nothing or error string.
   */
  cras::expected<void, std::string> setAllowYUVFallback(bool data);
  /**
   * \return If set, forces to read the given stream index instead of the automatically selected one.
   */
  cras::optional<int> forceStreamIndex() const;
  /**
   * \param[in] data If set, forces to read the given stream index instead of the automatically selected one.
   * \return Nothing or error string.
   */
  cras::expected<void, std::string> setForceStreamIndex(const cras::optional<int>& data);

  /**
   * \return ID of the geometrical camera frame (used for position/orientation metadata).
   */
  std::string frameId() const;
  /**
   * \param[in] data ID of the geometrical camera frame (used for position/orientation metadata).
   * \return Nothing or error string.
   */
  cras::expected<void, std::string> setFrameId(const std::string& data);
  /**
   * \return ID of the optical camera frame (used for images). If empty, frameId() is returned.
   */
  std::string opticalFrameId() const;
  /**
   * \param[in] data ID of the optical camera frame (used for images).
   * \return Nothing or error string.
   */
  cras::expected<void, std::string> setOpticalFrameId(const std::string& data);

  /**
   * \return How to extract timestamps.
   */
  TimestampSource timestampSource() const;
  /**
   * \param[in] data How to extract timestamps.
   * \return Nothing or error string.
   */
  cras::expected<void, std::string> setTimestampSource(TimestampSource data);
  /**
   * \return Optional offset to add to the extracted timestamps.
   */
  ros::Duration timestampOffset() const;
  /**
   * \param[in] data Optional offset to add to the extracted timestamps.
   * \return Nothing or error string.
   */
  cras::expected<void, std::string> setTimestampOffset(const ros::Duration& data);
  /**
   * \return Number of video decoding threads.
   */
  size_t numThreads() const;
  /**
   * \param[in] data Number of video decoding threads.
   * \return Nothing or error string.
   */
  cras::expected<void, std::string> setNumThreads(size_t data);

  /**
   * \return ROS/YAML parameters that configure the reader and metadata extractors.
   */
  cras::BoundParamHelperPtr rosParams() const;
  /**
   * \param[in] data ROS/YAML parameters that configure the reader and metadata extractors.
   * \return Nothing or error string.
   */
  cras::expected<void, std::string> setRosParams(const cras::BoundParamHelperPtr& data);

  /**
   * \return Types of metadata to be extracted (defaults to all metadata).
   */
  std::unordered_set<MetadataType> metadataTypes() const;
  /**
   * \param[in] types Types of metadata to be extracted.
   * \return Nothing or error string.
   */
  cras::expected<void, std::string> setMetadataTypes(const std::unordered_set<MetadataType>& types);

  /**
   * \brief Get the specification of subclip to play.
   * \return The subclip specification (start, end, duration).
   */
  std::tuple<cras::optional<StreamTime>, cras::optional<StreamTime>, cras::optional<StreamDuration>> getSubclip() const;

  /**
   * \brief Limit the part of the movie returned by nextFrame() calls to the given subclip.
   *
   * When all arguments are empty, the whole movie is played. If only `start` is non-empty, the movie plays from `start`
   * to movie end. If only `end` is non-empty, the movie plays from the movie start to `end`. If only `duration` is
   * non-empty, the movie plays from the movie start for `duration` seconds. If only `start` and `duration` are
   * non-empty, the movie plays from `start` to `start+duration`. If only `end` and `duration` are non-empty, the movie
   * plays from `end-duration` to `duration`. If only `start` and `end` are non-empty, the movie plays from `start` to
   * `end`.
   *
   * Error is returned when either:
   *  - all three arguments are non-empty
   *  - `end` is before `start`
   *  - `duration` is not positive
   *
   * \param[in] start The start time (relative to movie start).
   * \param[in] end The end time (relative to movie start).
   * \param[in] duration The duration.
   * \return Nothing or error message.
   */
  cras::expected<void, std::string> setSubClip(
    const cras::optional<StreamTime>& start, const cras::optional<StreamTime>& end,
    const cras::optional<StreamDuration>& duration);

  using Ptr = std::shared_ptr<MovieOpenConfig>;
  using ConstPtr = std::shared_ptr<const MovieOpenConfig>;

private:
  struct Impl;
  std::unique_ptr<Impl> data;  //!< PIMPL data
};

}
