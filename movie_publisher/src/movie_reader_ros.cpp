// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief MovieReader preconfigured from ROS parameters.
 * \author Martin Pecka
 */

#include <memory>
#include <string>
#include <unordered_map>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/functional.hpp>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/param_utils.hpp>
#include <movie_publisher/movie_reader_ros.h>
#include <movie_publisher/parsing_utils.h>
#include <ros/duration.h>
#include <ros/time.h>

namespace cras
{
DEFINE_CONVERTING_GET_PARAM_WITH_CONSTRUCTOR(::movie_publisher::StreamTime, double, "s")
DEFINE_CONVERTING_GET_PARAM_WITH_CONSTRUCTOR(::movie_publisher::StreamDuration, double, "s")
}

namespace movie_publisher
{
MovieReaderRos::MovieReaderRos(const cras::LogHelperPtr& log, const cras::BoundParamHelperPtr& params) :
  MovieReader(log, params), params(params)
{
}

cras::expected<MovieOpenConfig, std::string> MovieReaderRos::createDefaultConfig() const
{
  auto maybeConfig = MovieReader::createDefaultConfig();
  if (!maybeConfig.has_value())
    return cras::make_unexpected(maybeConfig.error());

  auto config = *maybeConfig;

  auto options = cras::GetParamConvertingOptions<TimestampSource, std::string>(
    &timestampSourceToStr, &parseTimestampSource);
  options.throwIfConvertFails = true;
  const auto timestampSource = this->params->getParam<TimestampSource, std::string>(
    "timestamp_source", TimestampSource::FromMetadata, "", options);
  if (auto result = config.setTimestampSource(timestampSource); !result.has_value())
    return cras::make_unexpected(result.error());

  if (auto result = config.setNumThreads(params->getParam("num_decoder_threads", 1_sz)); !result.has_value())
    return cras::make_unexpected(result.error());

  if (auto result = config.setAllowYUVFallback(params->getParam("allow_yuv_fallback", false)); !result.has_value())
    return cras::make_unexpected(result.error());
  if (params->hasParam("default_encoding"))
  {
    auto result = config.setDefaultEncoding(params->getParam<std::string>("default_encoding", cras::nullopt));
    if (!result.has_value())
      return cras::make_unexpected(result.error());
  }
  if (params->hasParam("encoding"))
  {
    auto result = config.setForceEncoding(params->getParam<std::string>("encoding", cras::nullopt));
    if (!result.has_value())
      return cras::make_unexpected(result.error());
  }

  const auto frameId = params->getParam("frame_id", "");
  if (auto result = config.setFrameId(frameId); !result.has_value())
    return cras::make_unexpected(result.error());

  const auto opticalFrameId = params->getParam(
    "optical_frame_id", frameId.empty() ? "" : frameId + "_optical_frame");
  if (auto result = config.setOpticalFrameId(opticalFrameId); !result.has_value())
    return cras::make_unexpected(result.error());

  return config;
}

void MovieReaderRos::addTimestampOffsetVar(const std::string& var, const double val)
{
  this->timestampOffsetVars[var] = val;
}

cras::expected<MoviePtr, std::string> MovieReaderRos::open(const std::string& filename, const MovieOpenConfig& config)
{
  auto maybeMovie = MovieReader::open(filename, config);
  if (!maybeMovie.has_value())
    return maybeMovie;

  const auto& movie = *maybeMovie;

  const auto timeOptions = cras::GetParamConvertingOptions<StreamTime, double>(
    cras::ParamToStringFn<StreamTime>::to_string, &parseTimeParam<StreamTime>);
  const auto durationOptions = cras::GetParamConvertingOptions<StreamDuration, double>(
    cras::ParamToStringFn<StreamDuration>::to_string, &parseTimeParam<StreamDuration>);

  cras::optional<StreamTime> start;
  if (this->params->hasParam("start"))
    start = this->params->getParamVerbose<StreamTime>("start", cras::nullopt, "s", timeOptions);

  cras::optional<StreamTime> end;
  if (this->params->hasParam("end"))
    end = this->params->getParamVerbose<StreamTime>("end", cras::nullopt, "s", timeOptions);

  cras::optional<StreamDuration> duration;
  if (this->params->hasParam("duration"))
    duration = this->params->getParamVerbose<StreamDuration>("duration", cras::nullopt, "s", durationOptions);

  const auto subclipResult = movie->setSubClip(start, end, duration);
  if (!subclipResult.has_value())
    return cras::make_unexpected(subclipResult.error());

  this->timestampOffsetVars["metadata_start"] = movie->info()->metadataStartTime().toSec();

  auto offsetOptions = cras::GetParamOptions<ros::Duration>{};
  offsetOptions.toParam = cras::bind_front(&parseTimestampOffset, this->timestampOffsetVars);
  const auto stampOffset = this->params->getParamVerbose<ros::Duration>(
    "timestamp_offset", ros::Duration{}, "", offsetOptions);
  if (!stampOffset.value.isZero())
    movie->setTimestampOffset(stampOffset);

  return movie;
}

}
