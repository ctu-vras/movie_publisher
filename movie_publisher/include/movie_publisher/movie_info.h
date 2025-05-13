// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Basic information about an open movie.
 * \author Martin Pecka
 */

#pragma once

#include <memory>
#include <string>

#include <movie_publisher/types.h>
#include <ros/time.h>

namespace movie_publisher
{

/**
 * \brief Basic information about an open movie.
 */
struct MovieInfo final
{
  MovieInfo();
  ~MovieInfo();
  MovieInfo(const MovieInfo& other);
  MovieInfo& operator=(const MovieInfo& other);
  MovieInfo(MovieInfo&& other) noexcept;
  MovieInfo& operator=(MovieInfo&& other) noexcept;

  /**
   * \return The name of the file with the movie, or its URL.
   */
  std::string filenameOrURL() const;
  /**
   * \param[in] data The name of the file with the movie, or its URL.
   */
  void setFilenameOrURL(const std::string& data);
  /**
   * \return Index of the selected stream from which the movie will be decoded.
   */
  int movieStreamIndex() const;
  /**
   * \param[in] data Index of the selected stream from which the movie will be decoded.
   */
  void setMovieStreamIndex(int data);

  /**
   * \return Whether the movie can be efficiently seeked.
   */
  bool isSeekable() const;
  /**
   * \param[in] data Whether the movie can be efficiently seeked.
   */
  void setIsSeekable(bool data);
  /**
   * \return Whether the movie is just a single still image.
   */
  bool isStillImage() const;
  /**
   * \param[in] data Whether the movie is just a single still image.
   */
  void setIsStillImage(bool data);

  /**
   * \return Pixel width of the movie.
   */
  size_t width() const;
  /**
   * \param[in] data Pixel width of the movie.
   */
  void setWidth(size_t data);
  /**
   * \return Pixel height of the movie.
   */
  size_t height() const;
  /**
   * \param[in] data Pixel height of the movie.
   */
  void setHeight(size_t data);

  /**
   * \return The framerate of the movie (can be 0 for still images).
   */
  RationalNumber frameRate() const;
  /**
   * \param[in] data The framerate of the movie (can be 0 for still images).
   */
  void setFrameRate(const RationalNumber& data);
  /**
   * \return The duration of the stream (may be 0 for live streams) (either stream or container duration, whichever is
   *         non-zero).
   */
  StreamDuration duration() const;
  /**
   * \param[in] data The duration of the stream (may be 0 for live streams) (either stream or container duration,
   *                 whichever is non-zero).
   */
  void setDuration(const StreamDuration& data);

  /**
   * \return Start time of the selected movie stream (relative to movie start).
   */
  StreamTime streamStart() const;
  /**
   * \param[in] data Start time of the selected movie stream (relative to movie start).
   */
  void setStreamStart(const StreamTime& data);
  /**
   * \return End time of the selected movie stream (relative to movie start) (can be 0 e.g. for live streams).
   */
  StreamTime streamEnd() const;
  /**
   * \param[in] data End time of the selected movie stream (relative to movie start) (can be 0 e.g. for live streams).
   */
  void setStreamEnd(const StreamTime& data);
  /**
   * \return The duration of the stream (may be 0 for live streams).
   */
  StreamDuration streamDuration() const;
  /**
   * \param[in] data The duration of the stream (may be 0 for live streams).
   */
  void setStreamDuration(const StreamDuration& data);
  /**
   * \return The number of frames in the movie (may be 0 for some formats or live streams).
   */
  size_t streamNumFrames() const;
  /**
   * \param[in] data The number of frames in the movie (may be 0 for some formats or live streams).
   */
  void setStreamNumFrames(size_t data);

  /**
   * \return Start time of the subclip (relative to movie start).
   */
  StreamTime subclipStart() const;
  /**
   * \param[in] data Start time of the subclip (relative to movie start).
   */
  void setSubclipStart(const StreamTime& data);
  /**
   * \return End time of the subclip (relative to movie start) (can be 0 e.g. for live streams).
   */
  StreamTime subclipEnd() const;
  /**
   * \param[in] data End time of the subclip (relative to movie start) (can be 0 e.g. for live streams).
   */
  void setSubclipEnd(const StreamTime& data);
  /**
   * \return Duration of the subclip (can be 0 e.g. for live streams).
   */
  StreamDuration subclipDuration() const;
  /**
   * \param[in] data Duration of the subclip (can be 0 e.g. for live streams).
   */
  void setSubclipDuration(const StreamDuration& data);
  /**
   * \return The number of frames in the subclip (may be 0 for some formats or live streams).
   */
  size_t subclipNumFrames() const;
  /**
   * \param[in] data The number of frames in the subclip (may be 0 for some formats or live streams).
   */
  void setSubclipNumFrames(size_t data);

  /**
   * \return The source of the timestamps.
   */
  TimestampSource timestampSource() const;
  /**
   * \param[in] data The source of the timestamps.
   */
  void setTimestampSource(TimestampSource data);
  /**
   * \return The stream start time read from metadata (zero if not found).
   */
  ros::Time metadataStartTime() const;
  /**
   * \param[in] data The stream start time read from metadata (zero if not found).
   */
  void setMetadataStartTime(const ros::Time& data);
  /**
   * \return Rotation of the image (0, 90, 180, 270).
   */
  int metadataRotation() const;
  /**
   * \param[in] data Rotation of the image (0, 90, 180, 270).
   */
  void setMetadataRotation(int data);

  using Ptr = std::shared_ptr<MovieInfo>;
  using ConstPtr = std::shared_ptr<const MovieInfo>;

private:
  struct Impl;
  std::unique_ptr<Impl> data;  //!< PIMPL data
};

}
