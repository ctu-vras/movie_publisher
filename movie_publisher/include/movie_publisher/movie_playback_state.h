// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief State of movie playback.
 * \author Martin Pecka
 */

#pragma once

#include <memory>

#include <movie_publisher/types.h>
#include <ros/time.h>

namespace movie_publisher
{

/**
 * \brief State of movie playback.
 */
struct MoviePlaybackState
{
  MoviePlaybackState();
  ~MoviePlaybackState();
  MoviePlaybackState(const MoviePlaybackState& other);
  MoviePlaybackState& operator=(const MoviePlaybackState& other);
  MoviePlaybackState(MoviePlaybackState&& other) noexcept;
  MoviePlaybackState& operator=(MoviePlaybackState&& other) noexcept;

  /**
   * \brief Reset the playback state to uninitialized state.
   */
  void reset();

  /**
   * \return The current frame number in the movie's stream.
   */
  size_t frameNum() const;
  /**
   * \param[in] num The current frame number in the movie's stream.
   */
  void setFrameNum(size_t num);

  /**
   * \return The current frame number in the selected subclip.
   */
  size_t subclipFrameNum() const;
  /**
   * \param[in] num The current frame number in the selected subclip.
   */
  void setSubclipFrameNum(size_t num);

  /**
   * \return The stream time of the last frame returned by MovieReader::nextFrame().
   */
  StreamTime streamTime() const;
  /**
   * \param[in] time The stream time of the last frame returned by MovieReader::nextFrame().
   */
  void setStreamTime(const StreamTime& time);

  /**
   * \return The subclip time of the last frame returned by MovieReader::nextFrame().
   */
  StreamTime subclipTime() const;
  /**
   * \param[in] time The subclip time of the last frame returned by MovieReader::nextFrame().
   */
  void setSubclipTime(const StreamTime& time);

  /**
   * \return The ROS time of the last frame returned by MovieReader::nextFrame().
   */
  ros::Time rosTime() const;
  /**
   * \param[in] time The ROS time of the last frame returned by MovieReader::nextFrame().
   */
  void setRosTime(const ros::Time& time);

  /**
   * \return Whether at least one frame of the movie has been read.
   */
  bool hasMovieStarted() const;
  /**
   * \param[in] started Whether at least one frame of the movie has been read.
   */
  void setMovieStarted(bool started);

  /**
   * \return Whether the movie has already ended (becomes true after the last nextFrame()).
   */
  bool hasMovieEnded() const;
  /**
   * \param[in] ended Whether the movie has already ended (becomes true after the last nextFrame()).
   */
  void setMovieEnded(bool ended);

  using Ptr = std::shared_ptr<MoviePlaybackState>;
  using ConstPtr = std::shared_ptr<const MoviePlaybackState>;

private:
  struct Impl;
  std::unique_ptr<Impl> data;  //!< PIMPL
};

}
