// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Basic information about an open movie.
 * \author Martin Pecka
 */

#include <string>

#include <movie_publisher/movie_info.h>
#include <movie_publisher/types.h>
#include <ros/time.h>

namespace movie_publisher
{

/**
 * \brief Basic information about an open movie.
 */
struct MovieInfo::Impl final
{
  std::string filenameOrURL;  //!< The name of the file with the movie, or its URL.
  int movieStreamIndex {0};  //!< Index of the selected stream from which the movie will be decoded.

  bool isSeekable {false};  //!< Whether the movie can be efficiently seeked.
  bool isStillImage {false};  //!< Whether the movie is just a single still image.

  size_t width {0};  //!< Pixel width of the movie.
  size_t height {0};  //!< Pixel height of the movie.

  RationalNumber frameRate {0, 1};  //!< The framerate of the movie (can be 0 for still images).
  //! The duration of the stream (may be 0 for live streams) (either stream or container duration, whichever is
  //! non-zero).
  StreamDuration duration;

  StreamTime streamStart;  //!< Start time of the selected movie stream (relative to movie start).
  //! End time of the selected movie stream (relative to movie start) (can be 0 e.g. for live streams).
  StreamTime streamEnd;
  StreamDuration streamDuration;  //!< The duration of the stream (may be 0 for live streams).
  size_t streamNumFrames {0u};  //!< The number of frames in the movie (may be 0 for some formats or live streams).

  StreamTime subclipStart;  //!< Start time of the subclip (relative to movie start).
  StreamTime subclipEnd;  //!< End time of the subclip (relative to movie start) (can be 0 e.g. for live streams).
  StreamDuration subclipDuration;  //!< Duration of the subclip (can be 0 e.g. for live streams).
  size_t subclipNumFrames {0u};  //!< The number of frames in the subclip (may be 0 for some formats or live streams).

  TimestampSource timestampSource {TimestampSource::RosTime};  //!< The source of the timestamps.
  ros::Time metadataStartTime;  //!< The stream start time read from metadata (zero if not found).
  int metadataRotation {0};  //!< Rotation of the image (0, 90, 180, 270).
};

MovieInfo::MovieInfo() : data(new Impl())
{
}

MovieInfo::~MovieInfo() = default;

MovieInfo::MovieInfo(const MovieInfo& other) : MovieInfo()
{
  *this->data = *other.data;
}

MovieInfo& MovieInfo::operator=(const MovieInfo& other)
{
  if (&other == this)
    return *this;
  *this->data = *other.data;
  return *this;
}

MovieInfo::MovieInfo(MovieInfo&& other) noexcept
{
  this->data = std::move(other.data);
}

MovieInfo& MovieInfo::operator=(MovieInfo&& other) noexcept
{
  this->data = std::move(other.data);
  return *this;
}

std::string MovieInfo::filenameOrURL() const
{
  return this->data->filenameOrURL;
}

void MovieInfo::setFilenameOrURL(const std::string& data)
{
  this->data->filenameOrURL = data;
}

int MovieInfo::movieStreamIndex() const
{
  return this->data->movieStreamIndex;
}

void MovieInfo::setMovieStreamIndex(const int data)
{
  this->data->movieStreamIndex = data;
}

bool MovieInfo::isSeekable() const
{
  return this->data->isSeekable;
}

void MovieInfo::setIsSeekable(const bool data)
{
  this->data->isSeekable = data;
}

bool MovieInfo::isStillImage() const
{
  return this->data->isStillImage;
}

void MovieInfo::setIsStillImage(const bool data)
{
  this->data->isStillImage = data;
}

size_t MovieInfo::width() const
{
  return this->data->width;
}

void MovieInfo::setWidth(const size_t data)
{
  this->data->width = data;
}

size_t MovieInfo::height() const
{
  return this->data->height;
}

void MovieInfo::setHeight(const size_t data)
{
  this->data->height = data;
}

RationalNumber MovieInfo::frameRate() const
{
  return this->data->frameRate;
}

void MovieInfo::setFrameRate(const RationalNumber& data)
{
  this->data->frameRate = data;
}

StreamDuration MovieInfo::duration() const
{
  return this->data->duration;
}

void MovieInfo::setDuration(const StreamDuration& data)
{
  this->data->duration = data;
}

StreamTime MovieInfo::streamStart() const
{
  return this->data->streamStart;
}

void MovieInfo::setStreamStart(const StreamTime& data)
{
  this->data->streamStart = data;
}

StreamTime MovieInfo::streamEnd() const
{
  return this->data->streamEnd;
}

void MovieInfo::setStreamEnd(const StreamTime& data)
{
  this->data->streamEnd = data;
}

StreamDuration MovieInfo::streamDuration() const
{
  return this->data->streamDuration;
}

void MovieInfo::setStreamDuration(const StreamDuration& data)
{
  this->data->streamDuration = data;
}

size_t MovieInfo::streamNumFrames() const
{
  return this->data->streamNumFrames;
}

void MovieInfo::setStreamNumFrames(const size_t data)
{
  this->data->streamNumFrames = data;
}

StreamTime MovieInfo::subclipStart() const
{
  return this->data->subclipStart;
}

void MovieInfo::setSubclipStart(const StreamTime& data)
{
  this->data->subclipStart = data;
}

StreamTime MovieInfo::subclipEnd() const
{
  return this->data->subclipEnd;
}

void MovieInfo::setSubclipEnd(const StreamTime& data)
{
  this->data->subclipEnd = data;
}

StreamDuration MovieInfo::subclipDuration() const
{
  return this->data->subclipDuration;
}

void MovieInfo::setSubclipDuration(const StreamDuration& data)
{
  this->data->subclipDuration = data;
}

size_t MovieInfo::subclipNumFrames() const
{
  return this->data->subclipNumFrames;
}

void MovieInfo::setSubclipNumFrames(const size_t data)
{
  this->data->subclipNumFrames = data;
}

TimestampSource MovieInfo::timestampSource() const
{
  return this->data->timestampSource;
}

void MovieInfo::setTimestampSource(const TimestampSource data)
{
  this->data->timestampSource = data;
}

ros::Time MovieInfo::metadataStartTime() const
{
  return this->data->metadataStartTime;
}

void MovieInfo::setMetadataStartTime(const ros::Time& data)
{
  this->data->metadataStartTime = data;
}

int MovieInfo::metadataRotation() const
{
  return this->data->metadataRotation;
}

void MovieInfo::setMetadataRotation(const int data)
{
  this->data->metadataRotation = data;
}

}
