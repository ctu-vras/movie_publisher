// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief An open movie.
 * \author Martin Pecka
 */

#include "movie_private.h"

#include <memory>
#include <string>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <sys/stat.h>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <movie_publisher/movie.h>
#include <movie_publisher/parsing_utils.h>
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

namespace movie_publisher
{

Movie::Movie(const cras::LogHelperPtr& log, const MovieOpenConfig& config) :
  HasLogger(log), data(new MoviePrivate(log))
{
  this->data->config = std::make_shared<MovieOpenConfig>(config);
  this->open().or_else([](const auto& error) {throw std::runtime_error(error);});
}

cras::expected<void, std::string> Movie::open()
{
  av_log_set_level(AV_LOG_WARNING);

  const auto& config = *this->data->config;
  auto& info = this->data->info;

  info->setFilenameOrURL(config.filenameOrURL());
  info->setTimestampSource(config.timestampSource());

  this->data->formatContext = avformat_alloc_context();

  cras::TempLocale l(LC_ALL, "en_US.UTF-8");
  CRAS_DEBUG("Opening the input file (%s) and loading format (container) header", config.filenameOrURL().c_str());

  if (avformat_open_input(&this->data->formatContext, config.filenameOrURL().c_str(), nullptr, nullptr) != 0)
  {
    this->close();
    return cras::make_unexpected(cras::format("could not open file %s", config.filenameOrURL().c_str()));
  }

  CRAS_DEBUG("Format %s, container duration %s s, bit_rate %ld bps",
    this->data->formatContext->iformat->name, cras::to_string(this->data->getContainerDuration()).c_str(),
    this->data->formatContext->bit_rate);

  const auto streamSelectResult = this->data->selectStream();
  if (!streamSelectResult.has_value())
  {
    this->close();
    return cras::make_unexpected(streamSelectResult.error());
  }

  const auto [codec, selectedStreamIndex] = *streamSelectResult;
  info->setMovieStreamIndex(selectedStreamIndex);
  info->setWidth(this->data->stream->codecpar->width);
  info->setHeight(this->data->stream->codecpar->height);
  info->setStreamStart(this->data->getStreamStart());
  info->setStreamEnd(this->data->getStreamEnd());
  info->setStreamDuration(this->data->getStreamDuration());
  info->setDuration(this->data->getDuration());
  // Initialize subclip to the whole stream
  info->setSubclipStart(info->streamStart());
  info->setSubclipEnd(info->streamEnd());
  info->setSubclipDuration(info->streamDuration());
  *this->data->playbackState = {};

  if (auto result = this->data->openCodec(codec); !result.has_value())
  {
    this->close();
    return result;
  }

  info->setIsSeekable(this->data->isSeekable());
  info->setIsStillImage(this->data->isStillImage());
  info->setFrameRate(this->data->getFrameRate());
  info->setStreamNumFrames(this->data->getNumFrames());

  this->data->detectTargetPixelFormat();

  for (const auto& processor : config.metadataProcessors())
  {
    if (auto result = processor->onOpen(info, config); !result.has_value())
      CRAS_ERROR("Error in movie metadata processor onOpen() function: %s", result.error().c_str());
  }

  this->data->prepareMetadataExtractors();

  for (const auto& processor : config.metadataProcessors())
  {
    if (auto result = processor->onMetadataReady(this->data->metadataManager); !result.has_value())
      CRAS_ERROR("Error in movie metadata processor onMetadataReady() function: %s", result.error().c_str());
  }

  info->setMetadataStartTime(this->data->metadataManager->getCreationTime().value_or(ros::Time{}));
  info->setMetadataRotation(this->data->metadataManager->getRotation().value_or(0));

  this->data->extractMetadata();

  if (info->metadataRotation() != 0)
  {
    if (auto result = this->data->addRotationFilter(); !result.has_value())
    {
      this->close();
      return result;
    }
  }

  if (auto result = this->data->configSwscale(); !result.has_value())
  {
    this->close();
    return result;
  }

  // When restarting playback by close/open, we already may have a specification of a subclip, so apply it again
  if (this->data->subclipStart.has_value() || this->data->subclipEnd.has_value() ||
      this->data->subclipDuration.has_value())
  {
    const auto result = this->setSubClip(
      this->data->subclipStart, this->data->subclipEnd, this->data->subclipDuration, false);
    if (!result.has_value())
      return result;
  }

  return {};
}

Movie::~Movie()
{
  Movie::close();
}

cras::expected<void, std::string> Movie::setSubClip(
  const cras::optional<StreamTime>& start, const cras::optional<StreamTime>& end,
  const cras::optional<StreamDuration>& duration)
{
  return this->setSubClip(start, end, duration, true);
}

cras::expected<void, std::string> Movie::setSubClip(
  const cras::optional<StreamTime>& start, const cras::optional<StreamTime>& end,
  const cras::optional<StreamDuration>& duration, const bool allowReopen)
{
  if (start.has_value() && end.has_value() && duration.has_value())
    return cras::make_unexpected("At least one of start, end and duration must be unset.");

  if (
    (end && start && *end <= *start) ||
    (duration && *duration <= StreamDuration{}) ||
    (duration && !this->data->info->duration().isZero() && *duration > this->data->info->duration()) ||
    (start && duration && !this->data->info->duration().isZero() &&
      (*start + *duration) > (StreamTime(this->data->info->duration()))) ||
    (end && duration && (*end < (StreamTime(*duration)))))
  {
    return cras::make_unexpected(
      "The provided combination of subclip start, end and duration is not consistent with the movie file.");
  }

  if (!this->data->info->isSeekable() && start.has_value() && !start->isZero())
    CRAS_WARN("Requested non-zero subclip start time, but the movie is not seekable. The performance will be bad.");

  StreamTime subclipStart {0, 0};
  StreamTime subclipEnd {0, 0};
  if (!this->data->info->duration().isZero())
    subclipEnd = subclipStart + this->data->info->duration();

  if (start)
    subclipStart = *start;
  if (end)
    subclipEnd = *end;

  if (duration)
  {
    if (start)
      subclipEnd = *start + *duration;
    else if (end)
      subclipStart = *end - *duration;
    else
      subclipEnd = subclipStart + *duration;
  }

  if (!subclipStart.isZero() && !this->data->info->streamStart().isZero() &&
      subclipStart < this->data->info->streamStart())
    return cras::make_unexpected(cras::format("The requested subclip start time %s is before the stream start time %s",
      cras::to_string(subclipStart).c_str(), cras::to_string(this->data->info->streamStart()).c_str()));

  if (!subclipEnd.isZero() && !this->data->info->streamEnd().isZero() && subclipEnd > this->data->info->streamEnd())
    return cras::make_unexpected(cras::format("The requested subclip end time %s is after the stream end time %s",
      cras::to_string(subclipEnd).c_str(), cras::to_string(this->data->info->streamEnd()).c_str()));

  this->data->info->setSubclipStart(subclipStart);
  this->data->info->setSubclipEnd(subclipEnd);
  this->data->info->setSubclipDuration(subclipEnd - subclipStart);

  this->data->subclipStart = start;
  this->data->subclipEnd = end;
  this->data->subclipDuration = duration;

  this->data->info->setSubclipNumFrames(static_cast<size_t>(
    this->data->info->subclipDuration().toSec() * static_cast<double>(this->data->info->frameRate())));

  const auto streamTime = this->data->playbackState->streamTime();
  if (streamTime >= this->data->info->subclipEnd() || streamTime < this->data->info->subclipStart())
  {
    return this->seek(this->data->info->subclipStart(), allowReopen);
  }
  else
  {
    this->data->playbackState->setSubclipTime(StreamTime(streamTime - subclipStart));
    const auto timeBase = this->data->stream->time_base;
    const auto streamPTS = streamTime.toStreamPTS(timeBase);
    const auto subclipStartPTS = subclipStart.toStreamPTS(timeBase);
    this->data->playbackState->setSubclipFrameNum(
      av_rescale_q(streamPTS - subclipStartPTS, timeBase, this->data->info->frameRate().av_q()));
  }

  return {};
}

void Movie::setTimestampOffset(const ros::Duration& offset)
{
  this->data->config->setTimestampOffset(offset);
}

cras::expected<void, std::string> Movie::seek(const StreamTime& time)
{
  return this->seek(time, true);
}

cras::expected<void, std::string> Movie::seek(const StreamTime& time, const bool allowReopen)
{
  const auto ts = time.toStreamPTS(this->data->stream->time_base);

  if (this->data->info->isSeekable())
  {
    const int res = avformat_seek_file(
      this->data->formatContext, this->data->info->movieStreamIndex(), INT64_MIN, ts, ts, 0);
    if (res < 0)
      return cras::make_unexpected(cras::format("Error seeking: %s", av_err2str(res)));

    avcodec_flush_buffers(this->data->codecContext);
  }
  else if (allowReopen)
  {
    CRAS_WARN_ONCE("Seeking a non-seekable file by close/open. This is very inefficient.");
    this->close();
    const auto res = this->open();
    if (!res.has_value())
      return res;
  }

  this->data->lastSeek = time;
  this->data->seekRequest = ts;

  this->data->metadataManager->seekTimedMetadata(time);

  for (const auto& processor : this->data->config->metadataProcessors())
  {
    if (auto result = processor->onSeek(time); !result.has_value())
      CRAS_ERROR_THROTTLE(1.0, "Error calling MovieMetadataProcessor onSeek() callback: %s", result.error().c_str());
  }

  return {};
}

cras::expected<void, std::string> Movie::seekInSubclip(const StreamTime& time)
{
  return this->seek(time + this->data->info->subclipStart().toDuration());
}

cras::expected<std::pair<MoviePlaybackState, sensor_msgs::ImageConstPtr>, std::string> Movie::nextFrame()
{
  while (true)
  {
    AVPacketPtr packet(av_packet_alloc());
    int res = av_read_frame(this->data->formatContext, packet.get());
    if (res < 0)
    {
      if (res != AVERROR_EOF)
        return cras::make_unexpected(cras::format("av_read_frame failure: %s", av_err2str(res)));
      this->data->playbackState->setMovieEnded(true);
      return std::make_pair(*this->data->playbackState, nullptr);
    }

    this->data->metadataManager->processPacket(packet.get());
    if (packet->stream_index != this->data->info->movieStreamIndex())
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
        // Jump to the outer loop to decode another packet (if EOF, it will go to the EOF branch)
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

      // Check if we should finish a seek by reading an throwing away frames
      if (this->data->seekRequest.has_value())
      {
        if (tmpFrame->pts >= *this->data->seekRequest)
          this->data->seekRequest.reset();
        else
          // If we seeked and have not yet arrived at the requested seek time, go to the outer loop and
          // just decode and skip frames.
          break;
      }

      const auto timeBase = this->data->stream->time_base;

      this->data->playbackState->setMovieStarted(true);
      this->data->playbackState->setStreamTime({tmpFrame->pts, timeBase});
      this->data->playbackState->setSubclipTime(StreamTime(std::max(
        this->data->playbackState->streamTime() - this->data->info->subclipStart(),
        StreamDuration(0, 0))));

      const AVRational framerateInv = av_inv_q(this->data->info->frameRate().av_q());
      this->data->playbackState->setFrameNum(av_rescale_q(tmpFrame->pts, timeBase, framerateInv));
      const auto ts = this->data->info->subclipStart().toStreamPTS(timeBase);
      this->data->playbackState->setSubclipFrameNum(av_rescale_q(tmpFrame->pts - ts, timeBase, framerateInv));

      this->data->updateMetadata(this->data->playbackState->streamTime());

      AVFramePtr frame(av_frame_alloc());
      sensor_msgs::ImagePtr msg(new sensor_msgs::Image);
      msg->header.frame_id = this->data->config->opticalFrameId();
      msg->header.stamp = this->data->getTimestamp(this->data->playbackState->streamTime());
      this->data->playbackState->setRosTime(msg->header.stamp);

      const auto maybeEncoding = avPixFmtToRosEncoding(this->data->targetPixelFormat);
      if (maybeEncoding.has_value())
        msg->encoding = *maybeEncoding;
      else
      {
        ROS_WARN_ONCE("Invalid image encoding. Setting BGR8 instead. %s", maybeEncoding.error().c_str());
        msg->encoding = sensor_msgs::image_encodings::BGR8;
      }

      if (this->data->info->metadataRotation() != 0)
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

      for (const auto& processor : this->data->config->metadataProcessors())
      {
        if (auto result = processor->processFrame(msg, *this->data->playbackState); !result.has_value())
          CRAS_ERROR_THROTTLE(1.0, "Error running MovieMetadataProcessor processFrame(): %s", result.error().c_str());
      }

      return std::make_pair(*this->data->playbackState, msg);
    }
  }
}

void Movie::close()
{
  CRAS_DEBUG("Closing movie.");

  for (const auto& processor : this->data->config->metadataProcessors())
  {
    if (auto result = processor->onClose(); !result.has_value())
      CRAS_ERROR_THROTTLE(1.0, "Error running MovieMetadataProcessor onClose(): %s", result.error().c_str());
  }

  *this->data->info = {};
  *this->data->playbackState = {};
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

MovieInfo::ConstPtr Movie::info() const
{
  return this->data->info;
}

MovieInfo::Ptr Movie::_info()
{
  return this->data->info;
}

const MovieOpenConfig& Movie::config() const
{
  return *this->data->config;
}

MovieOpenConfig& Movie::_config()
{
  return *this->data->config;
}

MetadataExtractor::Ptr Movie::staticMetadata() const
{
  return this->data->metadataManager;
}

MoviePlaybackState::ConstPtr Movie::playbackState() const
{
  return this->data->playbackState;
}

MoviePlaybackState::Ptr Movie::_playbackState()
{
  return this->data->playbackState;
}

}
