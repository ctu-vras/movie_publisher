// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief PIPML structure for MovieReader.
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
#include <movie_publisher/movie_reader.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/image_encodings.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavfilter/avfilter.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

namespace movie_publisher
{

//! \brief Map ROS image encodings to Libav pixel formats.
inline const std::unordered_map<std::string, AVPixelFormat> rosEncodingToAvPixFmt =
{
  {sensor_msgs::image_encodings::YUV422, AV_PIX_FMT_UYVY422},
  {sensor_msgs::image_encodings::BGR8, AV_PIX_FMT_BGR24},
  {sensor_msgs::image_encodings::BGR16, AV_PIX_FMT_BGR48},
  {sensor_msgs::image_encodings::BGRA8, AV_PIX_FMT_BGRA},
  {sensor_msgs::image_encodings::BGRA16, AV_PIX_FMT_BGRA64},
  {sensor_msgs::image_encodings::RGB8, AV_PIX_FMT_RGB24},
  {sensor_msgs::image_encodings::RGB16, AV_PIX_FMT_RGB48},
  {sensor_msgs::image_encodings::RGBA8, AV_PIX_FMT_RGBA},
  {sensor_msgs::image_encodings::RGBA16, AV_PIX_FMT_RGBA64},
  {sensor_msgs::image_encodings::MONO8, AV_PIX_FMT_GRAY8},
  {sensor_msgs::image_encodings::MONO16, AV_PIX_FMT_GRAY16},
};

//! \brief Map Libav pixel formats to ROS image encodings.
inline const std::unordered_map<AVPixelFormat, std::string> avPixFmtToRosEncoding =
{
  {AV_PIX_FMT_UYVY422, sensor_msgs::image_encodings::YUV422},
  {AV_PIX_FMT_BGR24, sensor_msgs::image_encodings::BGR8},
  {AV_PIX_FMT_BGR48, sensor_msgs::image_encodings::BGR16},
  {AV_PIX_FMT_BGRA, sensor_msgs::image_encodings::BGRA8},
  {AV_PIX_FMT_BGRA64, sensor_msgs::image_encodings::BGRA16},
  {AV_PIX_FMT_RGB24, sensor_msgs::image_encodings::RGB8},
  {AV_PIX_FMT_RGB48, sensor_msgs::image_encodings::RGB16},
  {AV_PIX_FMT_RGBA, sensor_msgs::image_encodings::RGBA8},
  {AV_PIX_FMT_RGBA64, sensor_msgs::image_encodings::RGBA16},
  {AV_PIX_FMT_GRAY8, sensor_msgs::image_encodings::MONO8},
  {AV_PIX_FMT_GRAY16, sensor_msgs::image_encodings::MONO16},
};

/**
 * \brief PIMPL structure for MovieReader.
 */
struct MovieReaderPrivate : public cras::HasLogger
{
  /**
   * \brief Constructor.
   * \param[in] log Logger.
   * \param[in] params ROS parameters configuring the movie reader.
   */
  explicit MovieReaderPrivate(const cras::LogHelperPtr& log, const cras::BoundParamHelperPtr& params);
  ~MovieReaderPrivate();

  std::string filename;  //!< Filename of the movie.

  // Parameters
  //! \brief Default encoding to use if the pixel format has no corresponding ROS color encoding.
  std::string defaultEncoding{sensor_msgs::image_encodings::BGR8};
  cras::optional<std::string> forceEncoding;  //!< If set, this encoding will be forced.
  bool allowYUVFallback{true};  //!< Allow falling back to YUV encodings if the pixel format has no ROS encoding.
  //! If set, forces to read the given stream index instead of the automatically selected one.
  cras::optional<int> forceStreamIndex;
  std::string frameId;  //!< ID of the geometrical camera frame (used for position/orientation metadata).
  std::string opticalFrameId;  //!< ID of the optical camera frame (used for images).
  MovieReader::TimestampSource timestampSource;  //!< How to extract timestamps.
  ros::Duration timestampOffset;  //!< Optional offset to add to the extracted timestamps.
  size_t numThreads{1};  //!< Number of video decoding threads.
  cras::BoundParamHelperPtr params;  //!< ROS parameters configuring the movie reader.

  // State variables
  cras::optional<int64_t> seekRequest;  //!< When set, the next returned frame should be seeked to this position.
  ros::Time lastSeek;  //!< The value of the last seek request (0 before first seek).

  // Extracted metadata
  std::shared_ptr<MetadataManager> metadataManager;  //!< Manager of the extractable metadata.

  ros::Time metadataStartTime;  //!< Global start time of the first frame.
  int metadataRotation{0};  //!< Rotation of the image (0, 90, 180, 270).
  cras::optional<sensor_msgs::CameraInfo> cameraInfoMsg;  //!< Extracted CameraInfo message.
  cras::optional<sensor_msgs::NavSatFix> navSatFixMsg;  //!< Extracted NavSatFix message.
  cras::optional<gps_common::GPSFix> gpsMsg;  //!< Extracted GPSFix message.
  cras::optional<compass_msgs::Azimuth> azimuthMsg;  //!< Extracted Azimuth message.
  cras::optional<sensor_msgs::Imu> imuMsg;  //!< Extracted Imu message.
  cras::optional<geometry_msgs::TransformStamped> opticalTfMsg;  //!< Extracted optical->geometrical TF.
  cras::optional<geometry_msgs::TransformStamped> zeroRollPitchTfMsg;  //!< Extracted TF to cancel-out roll and pitch.

  // Libav stuff
  AVPixelFormat targetPixelFormat;  //!< The desired output pixel format.
  int selectedStreamIndex;  //!< Index of the selected stream from which the movie will be decoded.
  int imageBufferSize;  //!< Size of the image buffer.
  AVFilterGraph* filterGraph {};  //!< Filter graph for decoding effects.
  AVFilterContext* filterBuffersrcContext {};  //!< Context for filter inputs.
  AVFilterContext* filterBuffersinkContext {};  //!< Context for filter outputs.
  SwsContext* swscaleContext {};  //!< Scaling context.
  AVCodecContext* codecContext {};  //!< Codec context.
  AVFormatContext* formatContext {};  //!< Format context.

  /**
   * \return Whether the movie is a still image or a multi-frame movie.
   */
  bool isStillImage() const;
  /**
   * \return Framerate of the movie [Hz].
   */
  double getFrameRate() const;
  /**
   * \return Duration of the movie.
   */
  ros::Duration getDuration() const;
  /**
   * \return Total number of frames in the movie (can be unavailable for some movies).
   */
  size_t getNumFrames() const;

  /**
   * \brief Run metadata extractors to parse as much as possible in the *Msg members.
   */
  void extractMetadata();

  /**
   * \brief Update the metadata for another frame (if some metadata are time-dependent).
   * \param[in] ptsTime PTS time of the current frame.
   */
  void updateMetadata(const ros::Time& ptsTime);

  /**
   * \brief Compute timestamp corresponding to the given PTS timecode taking into account the configured timestamp
   *        source.
   * \param[in] ptsTime PTS time.
   * \return The corresponding timestamp.
   */
  ros::Time getTimestamp(const ros::Time& ptsTime) const;

  /**
   * \brief Select the best quality stream and open it for decoding.
   * \return Open stream with the best quality.
   */
  cras::expected<AVCodec*, std::string> selectStream();
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
