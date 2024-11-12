// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief
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

struct MovieReaderPrivate : public cras::HasLogger
{
  explicit MovieReaderPrivate(const cras::LogHelperPtr& log, const cras::BoundParamHelperPtr& params);
  ~MovieReaderPrivate();

  std::string filename;

  // Parameters
  std::string defaultEncoding{sensor_msgs::image_encodings::BGR8};
  cras::optional<std::string> forceEncoding;
  bool allowYUVFallback{true};
  cras::optional<int> forceStreamIndex;
  std::string frameId;
  std::string opticalFrameId;
  MovieReader::TimestampSource timestampSource;
  ros::Duration timestampOffset;
  size_t numThreads{1};
  cras::BoundParamHelperPtr params;

  // State variables
  cras::optional<int64_t> seekRequest;
  ros::Time lastSeek;

  // Extracted metadata
  std::shared_ptr<MetadataManager> metadataManager;

  ros::Time metadataStartTime;
  int metadataRotation{0};
  cras::optional<sensor_msgs::CameraInfo> cameraInfoMsg;
  cras::optional<sensor_msgs::NavSatFix> navSatFixMsg;
  cras::optional<gps_common::GPSFix> gpsMsg;
  cras::optional<compass_msgs::Azimuth> azimuthMsg;
  cras::optional<sensor_msgs::Imu> imuMsg;
  cras::optional<geometry_msgs::TransformStamped> opticalTfMsg;
  cras::optional<geometry_msgs::TransformStamped> zeroRollPitchTfMsg;

  // Libav stuff
  AVPixelFormat targetPixelFormat;
  int selectedStreamIndex;
  int imageBufferSize;
  AVFilterGraph* filterGraph {};
  AVFilterContext* filterBuffersrcContext {};
  AVFilterContext* filterBuffersinkContext {};
  SwsContext* swscaleContext {};
  AVCodecContext* codecContext {};
  AVFormatContext* formatContext {};

  bool isStillImage() const;
  double getFrameRate() const;
  ros::Duration getDuration() const;
  size_t getNumFrames() const;

  void extractMetadata();
  void updateMetadata(const ros::Time& headerTime);

  cras::expected<AVCodec*, std::string> selectStream();
  cras::expected<void, std::string> openCodec(const AVCodec* codec);
  void detectTargetPixelFormat();
  cras::expected<void, std::string> addRotationFilter();
  cras::expected<void, std::string> configSwscale();
};

}
