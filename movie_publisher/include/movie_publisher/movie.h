// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief An open movie.
 * \author Martin Pecka
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/log_utils.h>
#include <movie_publisher/movie_info.h>
#include <movie_publisher/movie_metadata_processor.h>
#include <movie_publisher/movie_open_config.h>
#include <movie_publisher/movie_playback_state.h>
#include <movie_publisher/types.h>
#include <ros/duration.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

namespace movie_publisher
{

struct MoviePrivate;

/**
 * \brief An open movie that can be read, seeked and closed.
 */
class Movie : public cras::HasLogger
{
public:
  /**
   * \brief Open a movie in the referenced file.
   * \param[in] log cras_cpp_common logging helper.
   * \param[in] config The configuration specifying what movie to open and how.
   * \throws std::runtime_error If the movie cannot be opened.
   */
  Movie(const cras::LogHelperPtr& log, const MovieOpenConfig& config);

  virtual ~Movie();

  /**
   * \return Basic information about the open movie.
   */
  MovieInfo::ConstPtr info() const;

  /**
   * \return The configuration with which the movie has been opened.
   */
  const MovieOpenConfig& config() const;

  /**
   * \return The static metadata of the open movie.
   */
  virtual MetadataExtractor::Ptr staticMetadata() const;

  /**
   * \return The current playback state (after the last call to nextFrame()).
   */
  MoviePlaybackState::ConstPtr playbackState() const;

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
   *  - the movie is non-seekable and `start` or `end-duration` is non-zero
   *  - `start` or `end` are outside the movie timeline
   *  - `duration` is longer than the movie
   *  - `start` + `duration` is outside the movie timeline
   *  - `end` - `duration` is outside the movie timeline
   *  - `end` is before `start`
   *  - seek failed
   *
   * \note This function calls seek() if the current position in the whole movie does not fit in the new subclip.
   *       If you want to seek to the start of the subclip for sure, call `seekInSubClip(0)`.
   * \param[in] start The start time (relative to movie start).
   * \param[in] end The end time (relative to movie start).
   * \param[in] duration The duration.
   * \return Nothing or error message.
   */
  cras::expected<void, std::string> setSubClip(
    const cras::optional<StreamTime>& start, const cras::optional<StreamTime>& end,
    const cras::optional<StreamDuration>& duration);

protected:
  /**
   * \brief Limit the part of the movie returned by nextFrame() calls to the given subclip.
   * \copydoc setSubClip(const cras::optional<StreamTime>&, const cras::optional<StreamTime>&, const cras::optional<StreamDuration>&)
   * \param[in] allowReopen Whether it is allowed to reopen the movie (set to false to break recursion).
   */
  virtual cras::expected<void, std::string> setSubClip(
    const cras::optional<StreamTime>& start, const cras::optional<StreamTime>& end,
    const cras::optional<StreamDuration>& duration, bool allowReopen);

public:
  /**
   * \brief Set the offset of the computed ROS timestamps relative to the reference time.
   * \param [in] offset The offset.
   */
  virtual void setTimestampOffset(const ros::Duration& offset);

  /**
   * \brief Generator returning next frame of the movie on each call.
   *
   * On first call, this returns the first frame of the selected subclip. After seek(), this returns the first frame
   * after the time it was seeked to.
   *
   * \return The playback state and image. header.stamp of the image is the same as the rosTime of the playback state.
   *         It depends on what was set by setTimestampSource() and setTimestampOffset(). On error, an error message is
   *         returned. When a `nullptr` is returned for the image, the movie has reached its end and no more frames will
   *         be generated. This is also reflected in the playback state hasMovieEnded().
   */
  virtual cras::expected<std::pair<MoviePlaybackState, sensor_msgs::ImageConstPtr>, std::string> nextFrame();

  /**
   * \brief Seek the movie to the given time (relative to the start of the movie).
   * \param[in] time The stream time to seek to. It is invalid to seek outside the selected subclip.
   * \return Nothing or an error if seek failed.
   * \see seekInSubclip()
   */
  cras::expected<void, std::string> seek(const StreamTime& time);

protected:
  /**
   * \copydoc seek(const StreamTime&)
   * \param[in] allowReopen Whether it is allowed to reopen the movie (set to false to break recursion).
   */
  virtual cras::expected<void, std::string> seek(const StreamTime& time, bool allowReopen);

public:
  /**
   * \brief Seek the movie to the given time (relative to the start of the subclip).
   * \param[in] time The subclip time to seek to. It is invalid to seek outside the selected subclip.
   * \return Nothing or an error if seek failed.
   * \see seek()
   */
  virtual cras::expected<void, std::string> seekInSubclip(const StreamTime& time);

protected:
  /**
   * \brief Open the file configured in the `config` object passed to constructor.
   * \return Nothing or error.
   */
  virtual cras::expected<void, std::string> open();

  /**
   * \brief Close the movie, free all acquired resources.
   * \note This is automatically called in the destructor.
   */
  virtual void close();

  /**
   * \return Basic information about the open movie.
   */
  MovieInfo::Ptr _info();

  /**
   * \return The configuration with which the movie has been opened.
   */
  MovieOpenConfig& _config();

  /**
   * \return The current playback state (after the last call to nextFrame()).
   */
  MoviePlaybackState::Ptr _playbackState();

private:
  std::unique_ptr<MoviePrivate> data;  //!< PIMPL
};

using MoviePtr = std::shared_ptr<Movie>;
using MovieConstPtr = std::shared_ptr<const Movie>;
}
