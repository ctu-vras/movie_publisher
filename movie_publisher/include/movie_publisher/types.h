// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Custom movie_publisher datatypes.
 * \author Martin Pecka
 */

#pragma once

#include <string>

#include <ros/duration.h>
#include <ros/time.h>

struct AVRational;

namespace movie_publisher
{
/**
 * \brief A simple representation of rational numbers of the form numerator/denominator.
 */
struct RationalNumber
{
  int32_t numerator {0};  //!< Numerator
  int32_t denominator {1};  //!< Denominator

  // ReSharper disable once CppNonExplicitConversionOperator
  operator double() const;  // NOLINT(*-explicit-constructor)

  // ReSharper disable once CppNonExplicitConversionOperator
  operator float() const;  // NOLINT(*-explicit-constructor)

  /**
   * \return Corresponding libav representation of rational number.
   */
  AVRational av_q() const;
};

/**
 * \brief Duration type denoting movie stream duration.
 */
class StreamDuration : public ros::DurationBase<StreamDuration>
{
public:
  StreamDuration();
  StreamDuration(int32_t sec, int32_t nsec);
  explicit StreamDuration(double t);
  explicit StreamDuration(const ros::Duration& t);

  /**
   * \brief Convert PTS and timeBase to stream duration.
   * \param streamPTS Number of timeBase duration units.
   * \param timeBase The time base of the stream.
   */
  StreamDuration(int64_t streamPTS, const RationalNumber& timeBase);

  /**
   * \brief Convert PTS and timeBase to stream duration.
   * \param streamPTS Number of timeBase duration units.
   * \param timeBase The time base of the stream.
   */
  StreamDuration(int64_t streamPTS, const AVRational& timeBase);

  /**
   * \return The corresponding ROS duration (the same number of secs and nsecs, but different semantics).
   */
  ros::Duration toRosDuration() const;

  /**
   * \brief Convert this duration to PTS.
   * \param timeBase Time base of the test.
   * \return This duration converted to PTS of a stream with the given time base.
   */
  int64_t toStreamPTS(const RationalNumber& timeBase) const;

  /**
   * \brief Convert this duration to PTS.
   * \param timeBase Time base of the test.
   * \return This duration converted to PTS of a stream with the given time base.
   */
  int64_t toStreamPTS(const AVRational& timeBase) const;

  StreamDuration operator+(const StreamDuration& rhs) const {return StreamDuration(this->d() + rhs.d());}
  StreamDuration operator-(const StreamDuration& rhs) const {return StreamDuration(this->d() - rhs.d());}
  StreamDuration operator-() const  {return StreamDuration(-this->d());}
  StreamDuration operator*(const double scale) const {return StreamDuration(this->d() * scale);}
  StreamDuration& operator+=(const StreamDuration& rhs)
  {
    auto dur = this->d(); dur += rhs.d(); *this = StreamDuration(dur); return *this;
  }
  StreamDuration& operator-=(const StreamDuration& rhs) { return *this += -rhs; }
  StreamDuration& operator*=(const double scale)
  {
    auto dur = this->d(); dur *= scale; *this = StreamDuration(dur); return *this;
  }
  bool operator==(const StreamDuration& rhs) const {return this->d() == rhs.d();}
  bool operator!=(const StreamDuration& rhs) const { return !(*this == rhs); }
  bool operator>(const StreamDuration& rhs) const {return this->d() > rhs.d();}
  bool operator<(const StreamDuration& rhs) const {return this->d() < rhs.d();}
  bool operator>=(const StreamDuration& rhs) const {return this->d() >= rhs.d();}
  bool operator<=(const StreamDuration& rhs) const {return this->d() <= rhs.d();}
  bool isZero() const { return this->d().isZero(); }
  double toSec() const { return this->d().toSec(); }
  StreamDuration& fromSec(const double t) {*this = StreamDuration(ros::Duration().fromSec(t)); return *this;}
  StreamDuration& fromNSec(const int64_t t) {*this = StreamDuration(ros::Duration().fromNSec(t)); return *this;}
private:
  ros::Duration d() const {return this->toRosDuration();}
};

/**
 * \brief Time type denoting movie stream time.
 */
class StreamTime : public ros::TimeBase<StreamTime, StreamDuration>
{
public:
  StreamTime();
  StreamTime(uint32_t sec, uint32_t nsec);
  explicit StreamTime(double t);
  explicit StreamTime(const ros::Time& t);
  /**
   * \brief Convert the given duration to time.
   * \param[in] duration The duration to convert.
   * \note This is normally a wrong operation, but with stream times, it makes sense.
   */
  explicit StreamTime(const StreamDuration& duration);
  /**
   * \brief Convert PTS and timeBase to stream time.
   * \param streamPTS Number of timeBase time units.
   * \param timeBase The time base of the stream.
   */
  StreamTime(int64_t streamPTS, const RationalNumber& timeBase);
  /**
   * \brief Convert PTS and timeBase to stream time.
   * \param streamPTS Number of timeBase time units.
   * \param timeBase The time base of the stream.
   */
  StreamTime(int64_t streamPTS, const AVRational& timeBase);

  /**
   * \return The corresponding ROS time (the same number of secs and nsecs, but different semantics).
   */
  ros::Time toRosTime() const;

  /**
   * \brief Convert this time value to duration.
   * \return The corresponding duration.
   * \note This is normally a wrong operation, but with stream times, it makes sense.
   */
  StreamDuration toDuration() const;

  /**
   * \brief Convert this time to PTS.
   * \param timeBase Time base of the test.
   * \return This time converted to PTS of a stream with the given time base.
   */
  int64_t toStreamPTS(const RationalNumber& timeBase) const;

  /**
   * \brief Convert this time to PTS.
   * \param timeBase Time base of the test.
   * \return This time converted to PTS of a stream with the given time base.
   */
  int64_t toStreamPTS(const AVRational& timeBase) const;

  StreamDuration operator-(const StreamTime& rhs) const {return StreamDuration{this->t() - rhs.t()};}
  StreamTime operator+(const StreamDuration& rhs) const {return StreamTime{this->t() + rhs.toRosDuration()};}
  StreamTime operator-(const StreamDuration& rhs) const {return StreamTime{this->t() - rhs.toRosDuration()};}
  StreamTime& operator+=(const StreamDuration& rhs)
  {
    auto time = this->t(); time += rhs.toRosDuration(); *this = StreamTime(time); return *this;
  }
  StreamTime& operator-=(const StreamDuration& rhs) {return *this += -rhs;}
  bool operator==(const StreamTime& rhs) const {return this->t() == rhs.t();}
  bool operator!=(const StreamTime& rhs) const { return !(*this == rhs); }
  bool operator>(const StreamTime& rhs) const {return this->t() > rhs.t();}
  bool operator<(const StreamTime& rhs) const {return this->t() < rhs.t();}
  bool operator>=(const StreamTime& rhs) const {return this->t() >= rhs.t();}
  bool operator<=(const StreamTime& rhs) const {return this->t() <= rhs.t();}
  bool isZero() const { return this->t().isZero(); }
  double toSec() const { return this->t().toSec(); }
  StreamTime& fromSec(const double t){ *this = StreamTime(ros::Time().fromSec(t)); return *this; }
  StreamTime& fromNSec(const uint64_t t) { *this = StreamTime(ros::Time().fromNSec(t)); return *this; }
private:
  ros::Time t() const {return this->toRosTime();}
};

/**
 * \brief How to compute ROS timestamps from movie frame presentation timestamp (PTS).
 */
enum class TimestampSource
{
  AllZeros,  //!< ROS timestamp is always 0
  AbsoluteVideoTimecode,  //!< Just use the PTS as is.
  RelativeVideoTimecode,  //!< Use PTS, but make it relative to the time set by last seek() call.
  RosTime,  //!< Use current ROS time.
  FromMetadata,  //!< Use PTS and offset it by the start time recorded in movie metadata.
};

}

namespace cras
{

std::string to_string(const movie_publisher::StreamTime& time);
std::string to_string(const movie_publisher::StreamDuration& duration);

}
