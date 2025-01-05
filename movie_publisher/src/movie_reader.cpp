// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Read a movie or image file.
 * \author Martin Pecka
 */

#include "movie_reader_private.h"

#include <memory>
#include <string>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <exiv2/exiv2.hpp>
#include <sys/stat.h>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <movie_publisher/movie_reader.h>
#include <ros/duration.h>
#include <ros/time.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavfilter/avfilter.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>
#include <libavformat/avformat.h>
#include <libavutil/display.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include CXX_FILESYSTEM_INCLUDE
namespace fs = CXX_FILESYSTEM_NAMESPACE;

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

namespace movie_publisher
{

MovieReader::MovieReader(const cras::LogHelperPtr& log, const cras::BoundParamHelperPtr& params) :
  HasLogger(log), data(new MovieReaderPrivate(log, params))
{
  av_log_set_level(AV_LOG_WARNING);
}

MovieReader::~MovieReader() = default;

void MovieReader::setAllowYUVFallback(const bool allowYUVFallback)
{
  this->data->allowYUVFallback = allowYUVFallback;
}

void MovieReader::forceEncoding(const std::string& encoding)
{
  if (!encoding.empty())
  {
    if (rosEncodingToAvPixFmt.find(encoding) == rosEncodingToAvPixFmt.cend())
    {
      CRAS_ERROR(
        "Forced encoding has to be either a color encoding, mono encoding or yuv422, but %s was given. "
        "Not forcing the given encoding.", encoding.c_str());
      return;
    }

    this->data->forceEncoding = encoding;
  }
  else
    this->data->forceEncoding.reset();
}

void MovieReader::setDefaultEncoding(const std::string& encoding)
{
  if (rosEncodingToAvPixFmt.find(encoding) == rosEncodingToAvPixFmt.cend())
  {
    CRAS_ERROR(
      "Default encoding has to be either a color encoding, mono encoding or yuv422, but %s was given. "
      "The default encoding %s is not changing.", encoding.c_str(),
      this->data->defaultEncoding.c_str());
    return;
  }

  this->data->defaultEncoding = encoding;
}

void MovieReader::setStreamIndex(const int index)
{
  if (index >= 0)
    this->data->forceStreamIndex = index;
  else
    this->data->forceStreamIndex.reset();
}

void MovieReader::setFrameId(const std::string& frameId, const std::string& opticalFrameId)
{
  const auto optFrame = opticalFrameId.empty() ? frameId : opticalFrameId;
  this->data->frameId = frameId;
  this->data->opticalFrameId = optFrame;
  if (this->data->cameraInfoMsg.has_value())
    this->data->cameraInfoMsg->header.frame_id = optFrame;
  if (this->data->navSatFixMsg.has_value())
    this->data->navSatFixMsg->header.frame_id = frameId;
  if (this->data->gpsMsg.has_value())
    this->data->gpsMsg->header.frame_id = frameId;
  if (this->data->azimuthMsg.has_value())
    this->data->azimuthMsg->header.frame_id = frameId;
  if (this->data->imuMsg.has_value())
    this->data->imuMsg->header.frame_id = frameId;
  if (this->data->opticalTfMsg.has_value() && !opticalFrameId.empty())
  {
    this->data->opticalTfMsg->header.frame_id = frameId;
    this->data->opticalTfMsg->child_frame_id = optFrame;
  }
  if (this->data->zeroRollPitchTfMsg.has_value())
  {
    this->data->zeroRollPitchTfMsg->header.frame_id = frameId;
    this->data->zeroRollPitchTfMsg->child_frame_id = frameId + "_zero_roll_pitch";
  }
}

void MovieReader::setNumThreads(size_t numThreads)
{
  this->data->numThreads = numThreads;
}

void MovieReader::setTimestampSource(const TimestampSource& source)
{
  this->data->timestampSource = source;
  this->data->updateMetadata(this->data->lastSeek);
}

void MovieReader::setTimestampOffset(const ros::Duration& offset)
{
  this->data->timestampOffset = offset;
  this->data->updateMetadata(this->data->lastSeek);
}

double MovieReader::getFrameRate() const
{
  return this->data->getFrameRate();
}

ros::Duration MovieReader::getDuration() const
{
  return this->data->getDuration();
}

ros::Time MovieReader::getMetadataStartTime() const
{
  return this->data->metadataStartTime;
}

size_t MovieReader::getNumFrames() const
{
  return this->data->getNumFrames();
}

const cras::optional<compass_msgs::Azimuth>& MovieReader::getAzimuthMsg() const
{
  return this->data->azimuthMsg;
}

const cras::optional<sensor_msgs::CameraInfo>& MovieReader::getCameraInfoMsg() const
{
  return this->data->cameraInfoMsg;
}

const cras::optional<sensor_msgs::NavSatFix>& MovieReader::getNavSatFixMsg() const
{
  return this->data->navSatFixMsg;
}

const cras::optional<gps_common::GPSFix>& MovieReader::getGpsMsg() const
{
  return this->data->gpsMsg;
}

const cras::optional<sensor_msgs::Imu>& MovieReader::getImuMsg() const
{
  return this->data->imuMsg;
}

const cras::optional<geometry_msgs::TransformStamped>& MovieReader::getZeroRollPitchTF() const
{
  return this->data->zeroRollPitchTfMsg;
}

const cras::optional<geometry_msgs::TransformStamped>& MovieReader::getOpticalFrameTF() const
{
  return this->data->opticalTfMsg;
}

bool MovieReader::isSeekable() const
{
  if (this->data->formatContext == nullptr || this->data->codecContext == nullptr)
    return false;

#if LIBAVFORMAT_VERSION_INT >= AV_VERSION_INT(58, 6, 100)
  if (this->data->formatContext->ctx_flags & AVFMTCTX_UNSEEKABLE)
    return false;
#endif

  // Due to bug https://trac.ffmpeg.org/ticket/6113, JPEG "streams" cannot be seeked to the beginning
  if (std::string(this->data->formatContext->iformat->name) == "image2" &&
    this->data->codecContext->codec_id == AV_CODEC_ID_MJPEG)
    return false;

  return true;
}

bool MovieReader::isStillImage() const
{
  return this->data->isStillImage();
}

cras::expected<void, std::string> MovieReader::open(const std::string& filename)
{
  return this->open(filename, TimestampSource::RosTime);
}

cras::expected<void, std::string> MovieReader::open(const std::string& filename, const TimestampSource timestampSource)
{
  this->close();

  this->data->filename = filename;
  this->data->timestampSource = timestampSource;

  this->data->formatContext = avformat_alloc_context();

  cras::TempLocale l(LC_ALL, "en_US.UTF-8");
  CRAS_DEBUG("Opening the input file (%s) and loading format (container) header", filename.c_str());

  if (avformat_open_input(&this->data->formatContext, filename.c_str(), nullptr, nullptr) != 0)
  {
    this->close();
    return cras::make_unexpected(cras::format("could not open file %s", filename.c_str()));
  }

  CRAS_DEBUG("Format %s, container duration %ld us, bit_rate %ld",
    this->data->formatContext->iformat->name, this->data->formatContext->duration, this->data->formatContext->bit_rate);

  const auto streamSelectResult = this->data->selectStream();
  if (!streamSelectResult.has_value())
  {
    this->close();
    return cras::make_unexpected(streamSelectResult.error());
  }

  const AVCodec* codec = *streamSelectResult;

  const auto openCodecResult = this->data->openCodec(codec);
  if (!openCodecResult.has_value())
  {
    this->close();
    return cras::make_unexpected(openCodecResult.error());
  }

  this->data->detectTargetPixelFormat();

  this->data->extractMetadata();

  if (this->data->metadataRotation != 0)
  {
    const auto filterAddResult = this->data->addRotationFilter();
    if (!filterAddResult.has_value())
    {
      this->close();
      return cras::make_unexpected(filterAddResult.error());
    }
  }

  const auto swscaleConfigResult = this->data->configSwscale();
  if (!swscaleConfigResult.has_value())
  {
    this->close();
    cras::make_unexpected(swscaleConfigResult.error());
  }

  return {};
}

cras::expected<void, std::string> MovieReader::seek(const ros::Time& time)
{
  const auto& stream = this->data->formatContext->streams[this->data->selectedStreamIndex];

  const auto ts = av_rescale_q(time.toNSec(), av_make_q(1, 1000000000), stream->time_base);

  if (this->isSeekable())
  {
    const int res = avformat_seek_file(
      this->data->formatContext, this->data->selectedStreamIndex, INT64_MIN, ts, ts, 0);
    if (res < 0)
      return cras::make_unexpected(cras::format("Error seeking: %s", av_err2str(res)));

    avcodec_flush_buffers(this->data->codecContext);
  }
  else
  {
    CRAS_WARN_ONCE("Seeking a non-seekable file by close/open. This is very inefficient.");
    // We have to copy the arguments because open() will call close() internally which clears these values.
    const auto filename = this->data->filename;
    const auto timestampSource = this->data->timestampSource;
    const auto res = this->open(filename, timestampSource);
    if (!res.has_value())
      return res;
  }

  this->data->lastSeek = time;
  this->data->seekRequest = ts;

  return {};
}

cras::expected<std::pair<ros::Time, sensor_msgs::ImageConstPtr>, std::string> MovieReader::nextFrame()
{
  while (true)
  {
    AVPacketPtr packet(av_packet_alloc());
    int res = av_read_frame(this->data->formatContext, packet.get());
    if (res < 0)
    {
      if (res != AVERROR_EOF)
        return cras::make_unexpected(cras::format("av_read_frame failure: %s", av_err2str(res)));
      return std::make_pair(ros::Time{}, nullptr);
    }

    if (packet->stream_index != this->data->selectedStreamIndex)
      continue;

    // Send Packet for decoding
    res = avcodec_send_packet(this->data->codecContext, packet.get());

    if (res < 0)
      return cras::make_unexpected(cras::format("Error while sending a packet to the decoder: %s", av_err2str(res)));

    while (true)
    {
      AVFramePtr tmpFrame(av_frame_alloc());
      res = avcodec_receive_frame(this->data->codecContext, tmpFrame.get());
      if (res == AVERROR(EAGAIN) || res == AVERROR_EOF)
      {
        break;
      }
      else if (res < 0)
      {
        return cras::make_unexpected(cras::format(
          "Error while receiving a frame from the decoder: %s", av_err2str(res)));
      }

      // Fetch better timestamp if available
      if (tmpFrame->best_effort_timestamp != AV_NOPTS_VALUE)
        tmpFrame->pts = tmpFrame->best_effort_timestamp;

      if (this->data->seekRequest.has_value())
      {
        if (tmpFrame->pts >= *this->data->seekRequest)
          this->data->seekRequest.reset();
        else
          // If we seeked and have not yet arrived at the requested seek time, just decode and skip frames
          break;
      }

      const auto& stream = this->data->formatContext->streams[this->data->selectedStreamIndex];

      const auto rosPtsTime =
        ros::Time().fromNSec(av_rescale_q(tmpFrame->pts, stream->time_base, av_make_q(1, 1000000000)));

      AVFramePtr frame(av_frame_alloc());
      sensor_msgs::ImagePtr msg(new sensor_msgs::Image);
      msg->header.frame_id = this->data->opticalFrameId.empty() ? this->data->frameId : this->data->opticalFrameId;
      msg->header.stamp = this->data->getTimestamp(rosPtsTime);
      msg->encoding = avPixFmtToRosEncoding.at(this->data->targetPixelFormat);

      if (this->data->metadataRotation != 0.0)
      {
        res = av_buffersrc_add_frame(this->data->filterBuffersrcContext, tmpFrame.get());
        if (res < 0)
          return cras::make_unexpected(cras::format(
            "Error while feeding a frame into filter graph: %s", av_err2str(res)));

        while (true)
        {
          res = av_buffersink_get_frame(this->data->filterBuffersinkContext, tmpFrame.get());
          if (res == AVERROR(EAGAIN) || res == AVERROR_EOF)
            break;
          if (res < 0)
            return cras::make_unexpected(cras::format(
              "Error while reading a frame from filter graph: %s", av_err2str(res)));
          break;
        }
      }

      // allocate frame buffer for output
      msg->data.resize(this->data->imageBufferSize);
      av_image_fill_arrays(
        frame->data, frame->linesize, msg->data.data(), this->data->targetPixelFormat,
        tmpFrame->width, tmpFrame->height, av_cpu_max_align());

      sws_scale(this->data->swscaleContext,
        tmpFrame->data, tmpFrame->linesize, 0, tmpFrame->height, frame->data, frame->linesize);

      msg->width = tmpFrame->width;
      msg->height = tmpFrame->height;
      for (const int size : frame->linesize)
        msg->step += size;

      this->data->updateMetadata(rosPtsTime);

      return std::make_pair(rosPtsTime, msg);
    }
  }
}

void MovieReader::close()
{
  CRAS_DEBUG("Closing movie.");

  this->data->filename = "";
  this->data->seekRequest.reset();
  this->data->imageBufferSize = 0;
  this->data->filterBuffersrcContext = nullptr;
  this->data->filterBuffersinkContext = nullptr;
  avfilter_graph_free(&this->data->filterGraph);
  sws_freeContext(this->data->swscaleContext);
  this->data->swscaleContext = nullptr;
  avcodec_free_context(&this->data->codecContext);
  avformat_close_input(&this->data->formatContext);
}

}
