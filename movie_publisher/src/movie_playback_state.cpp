// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief State of movie playback.
 * \author Martin Pecka
 */

#include <memory>

#include <movie_publisher/movie_playback_state.h>
#include <movie_publisher/types.h>
#include <ros/time.h>

namespace movie_publisher
{

/**
 * \brief State of movie playback.
 */
struct MoviePlaybackState::Impl
{
  size_t frameNum {0u};  //!< The current frame number in the movie's stream.
  size_t subclipFrameNum {0u};  //!< The current frame number in the selected subclip.
  StreamTime streamTime;  //!< The stream time of the last frame returned by nextFrame().
  StreamTime subclipTime;  //!< The subclip time of the last frame returned by nextFrame().
  ros::Time rosTime;  //!< The ROS time corresponding to the last frame returned by nextFrame().
  bool hasMovieStarted {false};  //!< Whether at least one frame of the movie has been read..
  bool hasMovieEnded {false};  //!< Whether the movie has already ended (becomes true after the last nextFrame()).
};

MoviePlaybackState::MoviePlaybackState() : data(new Impl())
{
}

MoviePlaybackState::~MoviePlaybackState() = default;

MoviePlaybackState::MoviePlaybackState(const MoviePlaybackState& other) : MoviePlaybackState()
{
  *this->data = *other.data;
}

MoviePlaybackState::MoviePlaybackState(MoviePlaybackState&& other) noexcept
{
  std::swap(this->data, other.data);
}

MoviePlaybackState& MoviePlaybackState::operator=(const MoviePlaybackState& other)
{
  *this->data = *other.data;
  return *this;
}

MoviePlaybackState& MoviePlaybackState::operator=(MoviePlaybackState&& other) noexcept
{
  std::swap(this->data, other.data);
  return *this;
}

void MoviePlaybackState::reset()
{
  this->setFrameNum(0);
  this->setSubclipFrameNum(0);
  this->setMovieStarted(false);
  this->setMovieEnded(false);
  this->setStreamTime({});
  this->setSubclipTime({});
  this->setRosTime({});
}

size_t MoviePlaybackState::frameNum() const
{
  return this->data->frameNum;
}

void MoviePlaybackState::setFrameNum(const size_t num)
{
  this->data->frameNum = num;
}

size_t MoviePlaybackState::subclipFrameNum() const
{
  return this->data->subclipFrameNum;
}

void MoviePlaybackState::setSubclipFrameNum(const size_t num)
{
  this->data->subclipFrameNum = num;
}

StreamTime MoviePlaybackState::streamTime() const
{
  return this->data->streamTime;
}

void MoviePlaybackState::setStreamTime(const StreamTime& time)
{
  this->data->streamTime = time;
}

StreamTime MoviePlaybackState::subclipTime() const
{
  return this->data->subclipTime;
}

void MoviePlaybackState::setSubclipTime(const StreamTime& time)
{
  this->data->subclipTime = time;
}

ros::Time MoviePlaybackState::rosTime() const
{
  return this->data->rosTime;
}

void MoviePlaybackState::setRosTime(const ros::Time& time)
{
  this->data->rosTime = time;
}

bool MoviePlaybackState::hasMovieStarted() const
{
  return this->data->hasMovieStarted;
}

void MoviePlaybackState::setMovieStarted(const bool started)
{
  this->data->hasMovieStarted = started;
}

bool MoviePlaybackState::hasMovieEnded() const
{
  return this->data->hasMovieEnded;
}

void MoviePlaybackState::setMovieEnded(const bool ended)
{
  this->data->hasMovieEnded = ended;
}
}
