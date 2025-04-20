// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Custom movie_publisher datatypes.
 * \author Martin Pecka
 */

#include <limits>
#include <string>

#include <cras_cpp_common/string_utils.hpp>
#include <movie_publisher/types.h>
#include <ros/duration.h>
#include <ros/time.h>

extern "C" {
#include <libavutil/mathematics.h>
#include <libavutil/avutil.h>
#include <libavutil/rational.h>
}

namespace movie_publisher
{

RationalNumber::operator double() const
{
  return static_cast<double>(numerator) / denominator;
}

RationalNumber::operator float() const
{
  return static_cast<float>(static_cast<double>(*this));
}

AVRational RationalNumber::av_q() const
{
  return {this->numerator, this->denominator};
}

StreamTime::StreamTime() = default;

StreamTime::StreamTime(const uint32_t sec, const uint32_t nsec): StreamTime(ros::Time(sec, nsec))
{
}

StreamTime::StreamTime(const double t): StreamTime(ros::Time(t))
{
}

StreamTime::StreamTime(const ros::Time& t) : StreamTime()
{
  this->sec = t.sec;
  this->nsec = t.nsec;
}

StreamTime::StreamTime(const StreamDuration& duration)
{
  if (duration.sec < 0 || duration.nsec < 0)
    throw std::runtime_error("Cannot convert negative StreamDuration to StreamTime.");
  this->sec = duration.sec;
  this->nsec = duration.nsec;
}

StreamTime::StreamTime(const int64_t streamPTS, const RationalNumber& timeBase) : StreamTime(streamPTS, timeBase.av_q())
{
}

StreamTime::StreamTime(const int64_t streamPTS, const AVRational& timeBase)
{
  if (streamPTS == AV_NOPTS_VALUE)
    *this = {0, 0};
  else
    this->fromNSec(av_rescale_q(streamPTS, timeBase, av_make_q(1, 1000000000)));
}

ros::Time StreamTime::toRosTime() const
{
  return {this->sec, this->nsec};
}

StreamDuration StreamTime::toDuration() const
{
  if (this->sec > std::numeric_limits<decltype(StreamDuration::sec)>::max())
    throw std::runtime_error("Cannot convert StreamTime to StreamDuration. Seconds are too large.");
  if (this->nsec > std::numeric_limits<decltype(StreamDuration::nsec)>::max())
    throw std::runtime_error("Cannot convert StreamTime to StreamDuration. Nanoseconds are too large.");
  return StreamDuration(this->sec, this->nsec);
}

int64_t StreamTime::toStreamPTS(const RationalNumber& timeBase) const
{
  return this->toStreamPTS(timeBase.av_q());
}

int64_t StreamTime::toStreamPTS(const AVRational& timeBase) const
{
  if (timeBase.den == 0)
    return AV_NOPTS_VALUE;
  return av_rescale_q(this->toNSec(), av_make_q(1, 1000000000), timeBase);
}

StreamDuration::StreamDuration() = default;

StreamDuration::StreamDuration(const int32_t sec, const int32_t nsec) : StreamDuration(ros::Duration(sec, nsec))
{
}

StreamDuration::StreamDuration(const double t) : StreamDuration(ros::Duration(t))
{
}

StreamDuration::StreamDuration(const ros::Duration& t) : StreamDuration()
{
  this->sec = t.sec;
  this->nsec = t.nsec;
}

StreamDuration::StreamDuration(const int64_t streamPTS, const RationalNumber& timeBase)
  : StreamDuration(streamPTS, timeBase.av_q())
{
}

StreamDuration::StreamDuration(const int64_t streamPTS, const AVRational& timeBase)
{
  if (streamPTS == AV_NOPTS_VALUE)
    *this = {0, 0};
  else
    this->fromNSec(av_rescale_q(streamPTS, timeBase, av_make_q(1, 1000000000)));
}

ros::Duration StreamDuration::toRosDuration() const
{
  return {this->sec, this->nsec};
}

int64_t StreamDuration::toStreamPTS(const RationalNumber& timeBase) const
{
  return this->toStreamPTS(timeBase.av_q());
}

int64_t StreamDuration::toStreamPTS(const AVRational& timeBase) const
{
  if (timeBase.den == 0)
    return AV_NOPTS_VALUE;
  return av_rescale_q(this->toNSec(), av_make_q(1, 1000000000), timeBase);
}
}

std::string cras::to_string(const movie_publisher::StreamTime& time)
{
  return cras::to_string(time.toRosTime());
}

std::string cras::to_string(const movie_publisher::StreamDuration& duration)
{
  return cras::to_string(duration.toRosDuration());
}
