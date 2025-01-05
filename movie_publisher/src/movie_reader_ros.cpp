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

namespace movie_publisher
{
MovieReaderRos::MovieReaderRos(const cras::LogHelperPtr& log, const cras::BoundParamHelperPtr& params) :
  MovieReader(log, params), params(params)
{
  this->setNumThreads(params->getParam("num_decoder_threads", 1_sz));

  this->setAllowYUVFallback(params->getParam("allow_yuv_fallback", false));
  if (params->hasParam("default_encoding"))
    this->setDefaultEncoding(params->getParam<std::string>("default_encoding", cras::nullopt));
  if (params->hasParam("encoding"))
    this->forceEncoding(params->getParam<std::string>("encoding", cras::nullopt));

  this->frameId = params->getParam("frame_id", "");
  this->opticalFrameId = params->getParam(
    "optical_frame_id", this->frameId.empty() ? "" : this->frameId + "_optical_frame");
  this->setFrameId(this->frameId, this->opticalFrameId);
}

ros::Time MovieReaderRos::getStart() const
{
  return this->start;
}

ros::Time MovieReaderRos::getEnd() const
{
  return this->end;
}

std::string MovieReaderRos::getFrameId() const
{
  return this->frameId;
}

std::string MovieReaderRos::getOpticalFrameId() const
{
  return this->opticalFrameId;
}

void MovieReaderRos::addTimestampOffsetVar(const std::string& var, const double val)
{
  this->timestampOffsetVars[var] = val;
}

cras::expected<void, std::string> MovieReaderRos::open(const std::string& filename)
{
  auto options = cras::GetParamConvertingOptions<MovieReader::TimestampSource, std::string>(
    &timestampSourceToStr, &parseTimestampSource);
  options.throwIfConvertFails = true;
  const auto timestampSource = this->params->getParam<MovieReader::TimestampSource, std::string>(
    "timestamp_source", MovieReader::TimestampSource::FromMetadata, "", options);

  return this->open(filename, timestampSource);
}

cras::expected<void, std::string> MovieReaderRos::open(
  const std::string& filename, const TimestampSource timestampSource)
{
  auto result = MovieReader::open(filename, timestampSource);
  if (!result.has_value())
    return result;

  const auto timeOptions = cras::GetParamConvertingOptions<ros::Time, double>(
    cras::ParamToStringFn<ros::Time>::to_string, &parseTimeParam<ros::Time>);
  const auto durationOptions = cras::GetParamConvertingOptions<ros::Duration, double>(
    cras::ParamToStringFn<ros::Duration>::to_string, &parseTimeParam<ros::Duration>);

  cras::optional<ros::Time> start;
  if (this->params->hasParam("start"))
    start = this->params->getParamVerbose<ros::Time>("start", cras::nullopt, "s", timeOptions);

  cras::optional<ros::Time> end;
  if (this->params->hasParam("end"))
    end = this->params->getParamVerbose<ros::Time>("end", cras::nullopt, "s", timeOptions);

  cras::optional<ros::Duration> duration;
  if (this->params->hasParam("duration"))
    duration = this->params->getParamVerbose<ros::Duration>("duration", cras::nullopt, "s", durationOptions);

  if (start.has_value() && end.has_value() && duration.has_value())
    return cras::make_unexpected("At least one of ~start, ~end and ~duration parameters must remain unset.");

  if (
    (end && start && *end <= *start) ||
    (duration && *duration <= ros::Duration{}) ||
    (duration && *duration > this->getDuration()) ||
    (start && duration && (*start + *duration) > (ros::Time{} + this->getDuration())) ||
    (end && duration && (*end < (ros::Time{} + *duration))))
  {
    return cras::make_unexpected(
      "The provided combination of ~start, ~end and ~duration is not consistent with the movie file.");
  }

  this->start = {0, 0};
  this->end = {0, 0};

  if (start)
    this->start = *start;
  if (end)
    this->end = *end;

  if (duration)
  {
    if (start)
      this->end = *start + *duration;
    else if (end)
      this->start = *end - *duration;
    else
      this->end = this->start + *duration;
  }

  this->timestampOffsetVars["metadata_start"] = this->getMetadataStartTime().toSec();
  auto offsetOptions = cras::GetParamOptions<ros::Duration>{};
  offsetOptions.toParam = cras::bind_front(&parseTimestampOffset, this->timestampOffsetVars);
  const auto stampOffset = this->params->getParamVerbose<ros::Duration>(
    "timestamp_offset", ros::Duration{}, "", offsetOptions);
  this->setTimestampOffset(stampOffset);

  return result;
}

}
