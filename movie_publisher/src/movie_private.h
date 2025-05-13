// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief PIPML structure for Movie.
 * \author Martin Pecka
 */

#pragma once

#include <string>
#include <unordered_map>

#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <gps_common/GPSFix.h>
#include <movie_publisher/metadata_manager.h>
#include <movie_publisher/movie.h>
#include <movie_publisher/movie_reader.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/image_encodings.h>
#include <vision_msgs/Detection2DArray.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavfilter/avfilter.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

namespace movie_publisher
{

#ifdef av_err2str
#undef av_err2str
av_always_inline char* av_err2str(const int errnum)
{
  thread_local char str[AV_ERROR_MAX_STRING_SIZE] = {0};
  return av_make_error_string(str, AV_ERROR_MAX_STRING_SIZE, errnum);
}
#endif

// RAII helpers for libav

struct AVPacketClose
{
  void operator()(AVPacket* packet) noexcept { av_packet_unref(packet); }
};
using AVPacketPtr = std::unique_ptr<AVPacket, AVPacketClose>;

struct AVFrameClose
{
  void operator()(AVFrame* frame) noexcept { av_frame_unref(frame); }
};
using AVFramePtr = std::unique_ptr<AVFrame, AVFrameClose>;

template <typename M>
M updateHeader(const M& msg, const ros::Time& stamp, const std::string& frameId)
{
  M copy = msg;
  copy.header.stamp = stamp;
  copy.header.frame_id = frameId;
  return copy;
}

class MovieMetadataListener : public TimedMetadataListener
{
public:
  MovieMetadataListener(MovieOpenConfig& config, const std::function<ros::Time(const StreamTime&)>& getTimestamp);
  void processGNSSPosition(const TimedMetadata<GNSSFixAndDetail>& gnss) override;
  void processAzimuth(const TimedMetadata<compass_msgs::Azimuth>& data) override;
  void processMagneticField(const TimedMetadata<sensor_msgs::MagneticField>& data) override;
  void processFaces(const TimedMetadata<vision_msgs::Detection2DArray>& data) override;
  void processCameraInfo(const TimedMetadata<sensor_msgs::CameraInfo>& data) override;
  void processImu(const TimedMetadata<sensor_msgs::Imu>& data) override;
  void processRollPitch(const TimedMetadata<std::pair<double, double>>& data) override;
  void processOpticalFrameTF(const TimedMetadata<geometry_msgs::Transform>& data) override;

private:
  template <typename M>
  M fixHeader(const TimedMetadata<M>& msg, const std::string& frameId)
  {
    return updateHeader(msg.value, this->getTimestamp(msg.stamp), frameId);
  }

  MovieOpenConfig& config;
  std::function<ros::Time(const StreamTime&)> getTimestamp;
};

/**
 * \brief PIMPL structure for Movie.
 */
struct MoviePrivate : public cras::HasLogger
{
  /**
   * \brief Constructor.
   * \param[in] log Logger.
   */
  explicit MoviePrivate(const cras::LogHelperPtr& log);
  ~MoviePrivate();

  MovieOpenConfig::Ptr config;
  MovieInfo::Ptr info;
  MoviePlaybackState::Ptr playbackState;  //!< Playback state of the movie.

  cras::optional<StreamTime> subclipStart;  //!< If nonempty, specifies the start of subclip to process.
  cras::optional<StreamTime> subclipEnd;  //!< If nonempty, specifies the end of subclip to process.
  cras::optional<StreamDuration> subclipDuration;  //!< If nonempty, specifies the duration of subclip to process.

  // State variables
  cras::optional<int64_t> seekRequest;  //!< When set, the next returned frame should be seeked to this position.
  StreamTime lastSeek;  //!< The value of the last seek request (0 before first seek).

  // Extracted metadata
  std::shared_ptr<MetadataManager> metadataManager;  //!< Manager of the extractable metadata.
  std::shared_ptr<MovieMetadataListener> metadataListener;  //!< Listener to the extracted metadata.

  // Libav stuff
  AVPixelFormat targetPixelFormat;  //!< The desired output pixel format.
  int imageBufferSize;  //!< Size of the image buffer.
  AVFilterGraph* filterGraph {};  //!< Filter graph for decoding effects.
  AVFilterContext* filterBuffersrcContext {};  //!< Context for filter inputs.
  AVFilterContext* filterBuffersinkContext {};  //!< Context for filter outputs.
  SwsContext* swscaleContext {};  //!< Scaling context.
  AVCodecContext* codecContext {};  //!< Codec context.
  AVStream* stream {};  //!< The selected stream.
  AVFormatContext* formatContext {};  //!< Format context.

  /**
   * \return Whether the movie is a still image or a multi-frame movie.
   */
  bool isStillImage() const;

  /**
   * \return Whether the movie is seekable.
   */
  bool isSeekable() const;

  /**
   * \return Framerate of the movie [Hz].
   */
  RationalNumber getFrameRate() const;

  /**
   * \return Duration of the container of the movie.
   */
  StreamDuration getContainerDuration() const;

  /**
   * \return Duration of the movie stream.
   */
  StreamDuration getDuration() const;

  /**
   * \return Start time of the selected movie stream (relative to movie start). Zero if unknown.
   */
  StreamTime getStreamStart() const;

  /**
   * \return End time of the selected movie stream (relative to movie start). Zero if unknown.
   */
  StreamTime getStreamEnd() const;

  /**
   * \return Duration of the selected movie stream. Zero if unknown.
   */
  StreamDuration getStreamDuration() const;

  /**
   * \return Total number of frames in the movie (can be unavailable for some movies).
   */
  size_t getNumFrames() const;

  /**
   * \brief Prepare metadata extractors.
   */
  void prepareMetadataExtractors();

  /**
   * \brief Run metadata extractors to parse as much as possible in the *Msg members.
   */
  void extractMetadata();

  /**
   * \brief Update the metadata for another frame (if some metadata are time-dependent).
   * \param[in] ptsTime Stream time of the current frame.
   */
  void updateMetadata(const StreamTime& ptsTime);

  /**
   * \brief Compute ROS timestamp corresponding to the given stream timestamp taking into account the configured
   *        timestamp source.
   * \param[in] ptsTime Stream timestamp.
   * \return The corresponding ROS timestamp.
   */
  ros::Time getTimestamp(const StreamTime& ptsTime) const;

  /**
   * \brief Select the best quality stream and open it for decoding.
   * \return Open stream with the best quality and its index in the list of streams.
   */
  cras::expected<std::pair<AVCodec*, int>, std::string> selectStream();
  /**
   * \brief Open the given codec for decoding.
   * \param[in] codec The codec to open.
   * \return Nothing on success, error message otherwise.
   */
  cras::expected<void, std::string> openCodec(const AVCodec* codec);
  /**
   * \brief Detect the pixel format into which we should extract images.
   */
  void detectTargetPixelFormat();
  /**
   * \brief Add an image rotation filter to the libav graph so that the outputs are upright.
   * \return Nothing on success, error message otherwise.
   */
  cras::expected<void, std::string> addRotationFilter();
  /**
   * \brief Add a scaling filter to the libav graph so that the outputs are properly scaled and color-transformed.
   * \return Nothing on success, error message otherwise.
   */
  cras::expected<void, std::string> configSwscale();
};

}
