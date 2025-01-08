// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief PIPML structure for MovieReader.
 * \author Martin Pecka
 */

#include "movie_reader_private.h"

#include <clocale>
#include <string>
#include <regex>
#include <unordered_map>

#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <gps_common/GPSFix.h>
#include <movie_publisher/metadata_extractor.h>
#include <movie_publisher/metadata_manager.h>
#include <movie_publisher/movie_reader.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavfilter/avfilter.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>
#include <libavformat/avformat.h>
#include <libavutil/display.h>
#include <libavutil/imgutils.h>
#include <libavutil/pixdesc.h>
#include <libavutil/parseutils.h>
#include <libswscale/swscale.h>
}

#ifdef av_err2str
#undef av_err2str
av_always_inline char* av_err2str(const int errnum)
{
  thread_local char str[AV_ERROR_MAX_STRING_SIZE] = {0};
  return av_make_error_string(str, AV_ERROR_MAX_STRING_SIZE, errnum);
}
#endif

namespace movie_publisher
{

int64_t av_q_to_nsec(const AVRational& duration)
{
  return av_rescale_q(1L, duration, av_make_q(1, 1000000000));
}

ros::Duration av_q_to_ros_duration(const AVRational& duration)
{
  return ros::Duration().fromNSec(av_q_to_nsec(duration));
}

ros::Time av_q_to_ros_time(const AVRational& time)
{
  return ros::Time().fromNSec(av_q_to_nsec(time));
}

AVRational nsec_to_av_q(const int64_t nsec)
{
  AVRational res;
  av_reduce(&res.num, &res.den, nsec, 1000000000L, 10000000000L);
  return res;
}

AVRational ros_duration_to_av_q(const ros::Duration& duration)
{
  return nsec_to_av_q(duration.toNSec());
}

AVRational ros_time_to_av_q(const ros::Time& time)
{
  return nsec_to_av_q(time.toNSec());
}

MovieReaderPrivate::MovieReaderPrivate(const cras::LogHelperPtr& log, const cras::BoundParamHelperPtr& params) :
  cras::HasLogger(log), params(params)
{
#if LIBAVFORMAT_VERSION_INT < AV_VERSION_INT(58, 9, 100)
  av_register_all();
#endif
#if LIBAVFILTER_VERSION_INT < AV_VERSION_INT(7, 14, 100)
  avfilter_register_all();
#endif
}

MovieReaderPrivate::~MovieReaderPrivate()
{
  this->metadataManager.reset();
}

bool MovieReaderPrivate::isStillImage() const
{
  if (this->formatContext == nullptr || this->codecContext == nullptr)
    return false;

  if (this->formatContext->streams[this->selectedStreamIndex]->nb_frames == 1)
    return true;

  if (this->formatContext->streams[this->selectedStreamIndex]->duration == 1)
    return true;

  const std::string demuxerName = this->formatContext->iformat->name;
  if (demuxerName == "image2" || demuxerName == "png_pipe")
    return true;

  return false;
}

double MovieReaderPrivate::getFrameRate() const
{
  const auto& stream = this->formatContext->streams[this->selectedStreamIndex];

  // Static images
  if (stream->nb_frames == 1)
    return 0.0;

  const auto eps = av_make_q(1, 1000);

  auto fps = stream->avg_frame_rate;
  if (av_cmp_q(fps, eps) < 0)
    fps = stream->r_frame_rate;
  if (av_cmp_q(fps, eps) < 0)
    fps = av_guess_frame_rate(this->formatContext, stream, nullptr);

  return fps.den != 0 ? av_q2d(fps) : 0.0;
}

ros::Duration MovieReaderPrivate::getDuration() const
{
  const auto& stream = this->formatContext->streams[this->selectedStreamIndex];
  return av_q_to_ros_duration(av_mul_q(av_make_q(stream->duration, 1), stream->time_base));
}

size_t MovieReaderPrivate::getNumFrames() const
{
  if (this->isStillImage())
    return 1;
  const auto& stream = this->formatContext->streams[this->selectedStreamIndex];
  return static_cast<size_t>(stream->nb_frames);
}

void MovieReaderPrivate::extractMetadata()
{
  const auto& stream = this->formatContext->streams[this->selectedStreamIndex];
  const auto& codecParams = stream->codecpar;
  const size_t width = codecParams->width;
  const size_t height = codecParams->height;

  this->metadataManager = std::make_shared<MetadataManager>(this->log, width, height);

  const MetadataExtractorParams params = {
    this->log, this->metadataManager, this->params,
    this->filename, width, height,
    this->formatContext, this->selectedStreamIndex, this->isStillImage()
  };
  this->metadataManager->loadExtractorPlugins(params);

  this->metadataStartTime = this->metadataManager->getCreationTime().value_or(ros::Time{});
  this->metadataRotation = this->metadataManager->getRotation().value_or(0);

  const auto cameraInfoMsg = this->metadataManager->getCameraInfo();
  if (cameraInfoMsg.has_value())
  {
    this->cameraInfoMsg = *cameraInfoMsg;
    this->cameraInfoMsg->header.frame_id = this->opticalFrameId;
    this->cameraInfoMsg->header.stamp = this->getTimestamp(this->lastSeek);
  }

  const auto [navMsg, gpsMsg] = this->metadataManager->getGNSSPosition();
  if (navMsg.has_value())
  {
    this->navSatFixMsg = *navMsg;
    this->navSatFixMsg->header.frame_id = this->frameId;
    this->navSatFixMsg->header.stamp = this->getTimestamp(this->lastSeek);
  }
  if (gpsMsg.has_value())
  {
    this->gpsMsg = *gpsMsg;
    this->gpsMsg->header.frame_id = this->frameId;
    this->gpsMsg->header.stamp = this->getTimestamp(this->lastSeek);
  }

  const auto azimuth = this->metadataManager->getAzimuth();
  if (azimuth.has_value())
  {
    this->azimuthMsg = *azimuth;
    this->azimuthMsg->header.frame_id = this->frameId;
    this->azimuthMsg->header.stamp = this->getTimestamp(this->lastSeek);
  }

  const auto imuMsg = this->metadataManager->getImu();
  if (imuMsg.has_value())
  {
    this->imuMsg = *imuMsg;
    this->imuMsg->header.frame_id = this->frameId;
    this->imuMsg->header.stamp = this->getTimestamp(this->lastSeek);
  }

  const auto rollPitchOrientation = this->metadataManager->getRollPitchOrientation();
  if (rollPitchOrientation.has_value())
  {
    auto& tfMsg = this->zeroRollPitchTfMsg.emplace();
    tfMsg.header.frame_id = this->frameId;
    tfMsg.child_frame_id = this->frameId + "_zero_roll_pitch";
    tfMsg.header.stamp = this->getTimestamp(this->lastSeek);
    tf2::Quaternion quat;
    tf2::fromMsg(*rollPitchOrientation, quat);
    tfMsg.transform.rotation = tf2::toMsg(quat.inverse());
  }

  const auto opticalTf = this->metadataManager->getOpticalFrameTF();
  if (opticalTf.has_value())
  {
    auto& msg = this->opticalTfMsg.emplace();
    msg.header.stamp = this->getTimestamp(this->lastSeek);
    msg.header.frame_id = this->frameId;
    msg.transform = *opticalTf;
    msg.child_frame_id = this->opticalFrameId;
  }

  // Temporarily change to a UTF-8 locale so that we can print the ° characters.
  cras::TempLocale l(LC_CTYPE, "en_US.UTF-8");

  const auto uniqueCamName = this->metadataManager->getCameraUniqueName();
  if (uniqueCamName.has_value())
  {
    CRAS_INFO("Camera: %s", uniqueCamName.value().c_str());
  }
  else
  {
    const auto camName = this->metadataManager->getCameraGeneralName();
    if (camName.has_value())
      CRAS_INFO("Camera: %s", camName.value().c_str());
  }
  CRAS_INFO("Creation time: %s", cras::to_pretty_string(this->metadataStartTime).c_str());
  CRAS_INFO("Rotation is %d°.", this->metadataRotation);
  if (navMsg.has_value() || gpsMsg.has_value())
  {
    const auto lat = navMsg.has_value() ? navMsg->latitude : gpsMsg->latitude;
    const auto lon = navMsg.has_value() ? navMsg->longitude : gpsMsg->longitude;
    const auto alt = navMsg.has_value() ? navMsg->altitude : gpsMsg->altitude;
    CRAS_INFO("GPS coordinates are %0.8f° %s, %0.8f° %s, %0.2f m.a.s.l.",
      std::fabs(lat), lat >= 0 ? "N" : "S", std::fabs(lon), lon >= 0 ? "E" : "W", alt);
  }
  if (this->azimuthMsg.has_value())
    CRAS_INFO("Azimuth is %0.3f° from %s North.", this->azimuthMsg->azimuth,
      this->azimuthMsg->reference == compass_msgs::Azimuth::REFERENCE_GEOGRAPHIC ? "true" : "magnetic");
  if (this->imuMsg.has_value())
  {
    const auto& orientation = this->imuMsg->orientation;
    tf2::Quaternion quat;
    tf2::convert(orientation, quat);
    tf2::Matrix3x3 m(quat);
    double r, p, y;
    m.getRPY(r, p, y);
    r *= 180.0 / M_PI;
    p *= 180.0 / M_PI;
    y *= 180.0 / M_PI;
    CRAS_INFO("Roll is %.1f°, pitch is %.1f°, yaw is %.1f°.", r, p, y);
    const auto& a = this->imuMsg->linear_acceleration;
    CRAS_INFO("Acceleration is %.2f %.2f %.2f .", a.x, a.y, a.z);
  }
  if (this->cameraInfoMsg.has_value())
  {
    if (this->cameraInfoMsg->K[0] != 0)
      CRAS_INFO("Camera projection is calibrated [fx=%0.1f, fy=%0.1f, cx=%0.1f, cy=%0.1f].",
        this->cameraInfoMsg->K[0], this->cameraInfoMsg->K[4], this->cameraInfoMsg->K[2], this->cameraInfoMsg->K[5]);
    if (!this->cameraInfoMsg->D.empty() && this->cameraInfoMsg->D[0] != 0)
      CRAS_INFO("Camera distortion is calibrated: %s@%s.",
        this->cameraInfoMsg->distortion_model.c_str(), cras::to_string(this->cameraInfoMsg->D).c_str());
  }
}

void MovieReaderPrivate::updateMetadata(const ros::Time& ptsTime)
{
  // TODO handle metadata that can be updated
  const auto time = this->getTimestamp(ptsTime);
  if (this->azimuthMsg.has_value())
    this->azimuthMsg->header.stamp = time;
  if (this->cameraInfoMsg.has_value())
    this->cameraInfoMsg->header.stamp = time;
  if (this->navSatFixMsg.has_value())
    this->navSatFixMsg->header.stamp = time;
  if (this->gpsMsg.has_value())
    this->gpsMsg->header.stamp = time;
  if (this->imuMsg.has_value())
    this->imuMsg->header.stamp = time;
  if (this->zeroRollPitchTfMsg.has_value())
    this->zeroRollPitchTfMsg->header.stamp = time;

  // opticalTfMsg is static TF, so update its stamp only at the beginning
  if (ptsTime == ros::Time{} || ptsTime == this->lastSeek)
    this->opticalTfMsg->header.stamp = this->getTimestamp(this->lastSeek);
}

ros::Time MovieReaderPrivate::getTimestamp(const ros::Time& ptsTime) const
{
  ros::Time result;
  switch (this->timestampSource)
  {
    case MovieReader::TimestampSource::AbsoluteVideoTimecode:
      result = ptsTime;
    break;
    case MovieReader::TimestampSource::RelativeVideoTimecode:
      result.fromNSec((ptsTime - this->lastSeek).toNSec());
    break;
    case MovieReader::TimestampSource::AllZeros:
      result = {0, 0};
    break;
    case MovieReader::TimestampSource::RosTime:
      result = ros::Time::now();
    break;
    case MovieReader::TimestampSource::FromMetadata:
      result = this->metadataStartTime + (ptsTime - ros::Time{});
    break;
  }
  result += this->timestampOffset;
  return result;
}

cras::expected<AVCodec*, std::string> MovieReaderPrivate::selectStream()
{
  int res = avformat_find_stream_info(this->formatContext, nullptr);
  if (res < 0)
    return cras::make_unexpected(cras::format("Could not get the stream info: %s", av_err2str(res)));

  AVCodec* codec {nullptr};

  if (this->forceStreamIndex.has_value())
  {
    if (*this->forceStreamIndex >= this->formatContext->nb_streams)
    {
      return cras::make_unexpected(cras::format(
        "Requested stream number %i, but file %s has only %i streams.",
        *this->forceStreamIndex, this->filename.c_str(), this->formatContext->nb_streams));
    }

    const auto& stream = this->formatContext->streams[*this->forceStreamIndex];
    if (stream->codecpar->codec_type != AVMEDIA_TYPE_VIDEO)
    {
      return cras::make_unexpected(cras::format(
        "Stream %i in file %s is not a video stream.", *this->forceStreamIndex, this->filename.c_str()));
    }

    if (this->formatContext->video_codec != nullptr)
    {
      codec = this->formatContext->video_codec;
    }
    else
    {
      codec = avcodec_find_decoder(stream->codecpar->codec_id);
      if (codec == nullptr)
      {
        return cras::make_unexpected(cras::format(
          "Stream %i of file %s is a video stream, but there is no codec for it (codec ID is %i)!",
          *this->forceStreamIndex, this->filename.c_str(), stream->codecpar->codec_id));
      }
    }
    this->selectedStreamIndex = *this->forceStreamIndex;
  }
  else
  {
    this->selectedStreamIndex = av_find_best_stream(
      this->formatContext, AVMEDIA_TYPE_VIDEO, -1, -1, &codec, 0);
    if (this->selectedStreamIndex < 0)
    {
      if (this->selectedStreamIndex == AVERROR_STREAM_NOT_FOUND)
        return cras::make_unexpected(cras::format("File %s does not contain a video stream!", this->filename.c_str()));
      else if (this->selectedStreamIndex == AVERROR_DECODER_NOT_FOUND)
        cras::make_unexpected(cras::format(
          "File %s contains a video stream, but there is no codec for it!", this->filename.c_str()));
      else
        return cras::make_unexpected(cras::format(
          "Error finding a suitable video stream in file %s!", this->filename.c_str()));
    }
  }

  const auto& stream = this->formatContext->streams[this->selectedStreamIndex];
  const auto& codecParams = stream->codecpar;
  const auto inputPixFmt = static_cast<AVPixelFormat>(codecParams->format);
  const std::string inputPixFmtName = av_get_pix_fmt_name(inputPixFmt);

  CRAS_DEBUG("Stream properties:");
  CRAS_DEBUG("\ttime_base: %d/%d", stream->time_base.num, stream->time_base.den);
  CRAS_DEBUG("\tavg_frame_rate: %d/%d", stream->avg_frame_rate.num, stream->avg_frame_rate.den);
  CRAS_DEBUG("\tr_frame_rate: %d/%d", stream->r_frame_rate.num, stream->r_frame_rate.den);
  CRAS_DEBUG("\tnb_frames: %" PRId64, stream->nb_frames);
  CRAS_DEBUG("\tstart_time: %" PRId64, stream->start_time);
  CRAS_DEBUG("\tduration: %.3f s", av_q2d(av_mul_q(av_make_q(stream->duration, 1), stream->time_base)));
  CRAS_DEBUG("\tduration: %s s", cras::to_string(this->getDuration()).c_str());
  CRAS_DEBUG("\tpixfmt: %s", inputPixFmtName.c_str());

  CRAS_INFO("Video Codec: resolution %d x %d, codec %s, ID %d, bit_rate %ld Bps, FPS %.2f",
    codecParams->width, codecParams->height, codec->name, codec->id, codecParams->bit_rate, this->getFrameRate());

  return codec;
}

cras::expected<void, std::string> MovieReaderPrivate::openCodec(const AVCodec* codec)
{
  const auto& codecParams = this->formatContext->streams[this->selectedStreamIndex]->codecpar;

  this->codecContext = avcodec_alloc_context3(codec);

  int res = avcodec_parameters_to_context(this->codecContext, codecParams);
  if (res < 0)
    cras::make_unexpected(cras::format("Failed to copy codec params to codec context: %s", av_err2str(res)));

  AVDictionary* codecOpts = nullptr;
  av_dict_set(&codecOpts, "threads", cras::to_string(this->numThreads).c_str(), 0);

  res = avcodec_open2(this->codecContext, codec, &codecOpts);
  if (res < 0)
  {
    av_dict_free(&codecOpts);
    cras::make_unexpected(cras::format("Failed to open codec through avcodec_open2: %s", av_err2str(res)));
  }
  av_dict_free(&codecOpts);

  return {};
}

void MovieReaderPrivate::detectTargetPixelFormat()
{
  const AVStream* stream = this->formatContext->streams[this->selectedStreamIndex];
  const AVCodecParameters* codecParams = stream->codecpar;

  const auto inputPixFmt = static_cast<AVPixelFormat>(codecParams->format);
  const std::string inputPixFmtName = av_get_pix_fmt_name(inputPixFmt);

  // Detect which encoding and pixel format will be used for the conversion to ROS image.
  if (this->forceEncoding.has_value())
  {
    if (rosEncodingToAvPixFmt.find(this->forceEncoding.value()) != rosEncodingToAvPixFmt.end())
    {
      this->targetPixelFormat = rosEncodingToAvPixFmt.at(this->forceEncoding.value());
    }
    else
    {
      this->targetPixelFormat = rosEncodingToAvPixFmt.at(this->defaultEncoding);
      CRAS_WARN(
        "ROS encoding '%s' is not supported. Converting to default encoding '%s' instead.",
        this->forceEncoding->c_str(), this->defaultEncoding.c_str());
    }
  }
  else
  {
    // There is a direct pixfmt->ros mapping
    if (avPixFmtToRosEncoding.find(inputPixFmt) != avPixFmtToRosEncoding.end())
    {
      this->targetPixelFormat = inputPixFmt;
    }
    // There is no direct mapping, use a fallback
    else
    {
      if (this->allowYUVFallback && cras::startsWith(cras::toLower(inputPixFmtName), "yuv"))
      {
        this->targetPixelFormat = rosEncodingToAvPixFmt.at(sensor_msgs::image_encodings::YUV422);
        CRAS_DEBUG(
          "Pixel format '%s' has no corresponding ROS encoding. Converting to default YUV encoding '%s'.",
          inputPixFmtName.c_str(), avPixFmtToRosEncoding.at(this->targetPixelFormat).c_str());
      }
      else
      {
        this->targetPixelFormat = rosEncodingToAvPixFmt.at(this->defaultEncoding);
        CRAS_DEBUG(
          "Pixel format '%s' has no corresponding ROS encoding. Converting to default encoding '%s'.",
          inputPixFmtName.c_str(), this->defaultEncoding.c_str());
      }
    }
  }

  CRAS_DEBUG("Converting input pixel format '%s' to ROS encoding '%s' using pixel format '%s'.",
             inputPixFmtName.c_str(), avPixFmtToRosEncoding.at(this->targetPixelFormat).c_str(),
             av_get_pix_fmt_name(this->targetPixelFormat));
}

cras::expected<void, std::string> MovieReaderPrivate::addRotationFilter()
{
  auto buffersrc = avfilter_get_by_name("buffer");
  auto buffersink = avfilter_get_by_name("buffersink");
  if (!buffersink)
    buffersink = avfilter_get_by_name("ffbuffersink");

  if (buffersrc == nullptr || buffersink == nullptr)
    return cras::make_unexpected("Error finding buffer or buffersink filters. That should not happen.");

  this->filterGraph = avfilter_graph_alloc();
  this->filterGraph->nb_threads = 1;

  const auto& stream = this->formatContext->streams[this->selectedStreamIndex];
  const auto& codecParams = stream->codecpar;
  const auto args = cras::format(
    "video_size=%dx%d:pix_fmt=%d:time_base=%d/%d:pixel_aspect=%d/%d",
    codecParams->width, codecParams->height, codecParams->format, stream->time_base.num, stream->time_base.den,
    stream->sample_aspect_ratio.num, stream->sample_aspect_ratio.den);
  if (avfilter_graph_create_filter(
    &this->filterBuffersrcContext, buffersrc, "in", args.c_str(), nullptr, this->filterGraph) < 0)
  {
    return cras::make_unexpected("Error creating filter source buffer");
  }

  AVPixelFormat pix_fmts[] = {this->targetPixelFormat, AV_PIX_FMT_NONE};
  auto buffersinkParams = av_buffersink_params_alloc();
  buffersinkParams->pixel_fmts = pix_fmts;
  if (avfilter_graph_create_filter(
    &this->filterBuffersinkContext, buffersink, "out", nullptr, buffersinkParams, this->filterGraph) < 0)
  {
    av_free(buffersinkParams);
    return cras::make_unexpected("Error creating filter sink buffer");
  }
  av_free(buffersinkParams);

  AVFilterInOut* filterOutputs = avfilter_inout_alloc();
  AVFilterInOut* filterInputs = avfilter_inout_alloc();

  filterOutputs->name = av_strdup("in");
  filterOutputs->filter_ctx = this->filterBuffersrcContext;
  filterOutputs->pad_idx = 0;
  filterOutputs->next = nullptr;

  filterInputs->name = av_strdup("out");
  filterInputs->filter_ctx = this->filterBuffersinkContext;
  filterInputs->pad_idx = 0;
  filterInputs->next = nullptr;

  std::string filterDesc;
  if (this->metadataRotation == 90.0)
    filterDesc = "transpose=1";
  else if (this->metadataRotation == 180.0)
    filterDesc = "transpose=1,transpose=1";
  else
    filterDesc = "transpose=2";

  int ret = avfilter_graph_parse_ptr(
    this->filterGraph, filterDesc.c_str(), &filterInputs, &filterOutputs, nullptr);
  if (ret < 0)
  {
    avfilter_inout_free(&filterInputs);
    avfilter_inout_free(&filterOutputs);
    return cras::make_unexpected(cras::format("Failed to parse filters description: %s", av_err2str(ret)));
  }

  avfilter_inout_free(&filterInputs);
  avfilter_inout_free(&filterOutputs);

  ret = avfilter_graph_config(this->filterGraph, nullptr);
  if (ret < 0)
    return cras::make_unexpected(cras::format("Failed to config filter graph: %s", av_err2str(ret)));;

  return {};
}

cras::expected<void, std::string> MovieReaderPrivate::configSwscale()
{
  const auto& codecParams = this->formatContext->streams[this->selectedStreamIndex]->codecpar;
  const auto swapDims = this->metadataRotation == 90 || this->metadataRotation == 270;
  const auto outWidth = swapDims ? codecParams->height : codecParams->width;
  const auto outHeight = swapDims ? codecParams->width : codecParams->height;

  this->swscaleContext = sws_getCachedContext(
    nullptr,
    outWidth, outHeight, static_cast<AVPixelFormat>(codecParams->format),
    outWidth, outHeight, this->targetPixelFormat,
    SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);

  if (!this->swscaleContext)
    cras::make_unexpected("failed to get swscale context");

  this->imageBufferSize = av_image_get_buffer_size(this->targetPixelFormat, outWidth, outHeight, av_cpu_max_align());
  if (this->imageBufferSize < 0)
    cras::make_unexpected("failed to get image buffer size");

  return {};
}

}
