// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief PIPML structure for MovieReader.
 * \author Martin Pecka
 */

#include "movie_private.h"

#include <clocale>
#include <string>
#include <regex>
#include <unordered_map>

#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/functional.hpp>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <gps_common/GPSFix.h>
#include <movie_publisher/metadata_extractor.h>
#include <movie_publisher/metadata_manager.h>
#include <movie_publisher/movie.h>
#include <movie_publisher/movie_reader.h>
#include <movie_publisher/parsing_utils.h>
#include <movie_publisher/types.h>
#include <ros/assert.h>
#include <ros/common.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/FrameGraph.h>
#include <tf2_msgs/TFMessage.h>
#include <vision_msgs/Detection2DArray.h>

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

#if ROS_VERSION_MINIMUM(1, 17, 0)
#define MAGIC_ENUM_USING_ALIAS_OPTIONAL template <typename T> using optional = cras::optional<T>;
#include <magic_enum.hpp>

std::string enum_name(const movie_publisher::MetadataType type)
{
  return std::string(magic_enum::enum_name(type));
}
#else
std::string enum_name(const movie_publisher::MetadataType type)
{
  switch (type)
  {
    case movie_publisher::MetadataType::CAMERA_GENERAL_NAME: return "CAMERA_GENERAL_NAME";
    case movie_publisher::MetadataType::CAMERA_UNIQUE_NAME: return "CAMERA_UNIQUE_NAME";
    case movie_publisher::MetadataType::CAMERA_SERIAL_NUMBER: return "CAMERA_SERIAL_NUMBER";
    case movie_publisher::MetadataType::CAMERA_MAKE: return "CAMERA_MAKE";
    case movie_publisher::MetadataType::CAMERA_MODEL: return "CAMERA_MODEL";
    case movie_publisher::MetadataType::LENS_MAKE: return "LENS_MAKE";
    case movie_publisher::MetadataType::LENS_MODEL: return "LENS_MODEL";
    case movie_publisher::MetadataType::CREATION_TIME: return "CREATION_TIME";
    case movie_publisher::MetadataType::ROTATION: return "ROTATION";
    case movie_publisher::MetadataType::CROP_FACTOR: return "CROP_FACTOR";
    case movie_publisher::MetadataType::SENSOR_SIZE_MM: return "SENSOR_SIZE_MM";
    case movie_publisher::MetadataType::FOCAL_LENGTH_35MM: return "FOCAL_LENGTH_35MM";
    case movie_publisher::MetadataType::FOCAL_LENGTH_MM: return "FOCAL_LENGTH_MM";
    case movie_publisher::MetadataType::FOCAL_LENGTH_PX: return "FOCAL_LENGTH_PX";
    case movie_publisher::MetadataType::INTRINSIC_MATRIX: return "INTRINSIC_MATRIX";
    case movie_publisher::MetadataType::DISTORTION: return "DISTORTION";
    case movie_publisher::MetadataType::GNSS_POSITION: return "GNSS_POSITION";
    case movie_publisher::MetadataType::AZIMUTH: return "AZIMUTH";
    case movie_publisher::MetadataType::MAGNETIC_FIELD: return "MAGNETIC_FIELD";
    case movie_publisher::MetadataType::ROLL_PITCH: return "ROLL_PITCH";
    case movie_publisher::MetadataType::ACCELERATION: return "ACCELERATION";
    case movie_publisher::MetadataType::ANGULAR_VELOCITY: return "ANGULAR_VELOCITY";
    case movie_publisher::MetadataType::FACES: return "FACES";
    case movie_publisher::MetadataType::CAMERA_INFO: return "CAMERA_INFO";
    case movie_publisher::MetadataType::IMU: return "IMU";
    case movie_publisher::MetadataType::OPTICAL_FRAME_TF: return "OPTICAL_FRAME_TF";
    case movie_publisher::MetadataType::ZERO_ROLL_PITCH_TF: return "ZERO_ROLL_PITCH_TF";
    default: return "UNKNOWN";
  }
}
#endif

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

MovieMetadataListener::MovieMetadataListener(
  MovieOpenConfig& config, const std::function<ros::Time(const StreamTime&)>& getTimestamp)
  : config(config), getTimestamp(getTimestamp)
{
}

void MovieMetadataListener::processGNSSPosition(
  const TimedMetadata<std::pair<cras::optional<sensor_msgs::NavSatFix>, cras::optional<gps_common::GPSFix>>>& gnss)
{
  for (const auto& processor : this->config.metadataProcessors())
  {
    if (gnss.value.first.has_value())
    {
      auto copy = *gnss.value.first;
      copy.header.stamp = this->getTimestamp(gnss.stamp);
      copy.header.frame_id = this->config.frameId();
      processor->processNavSatFix(copy);
    }
    if (gnss.value.second.has_value())
    {
      auto copy = *gnss.value.second;
      copy.header.stamp = this->getTimestamp(gnss.stamp);
      copy.header.frame_id = this->config.frameId();
      copy.status.header = copy.header;
      processor->processGps(copy);
    }
  }
}
void MovieMetadataListener::processAzimuth(const TimedMetadata<compass_msgs::Azimuth>& data)
{
  for (const auto& processor : this->config.metadataProcessors())
    processor->processAzimuth(this->fixHeader(data, this->config.frameId()));
}
void MovieMetadataListener::processMagneticField(const TimedMetadata<sensor_msgs::MagneticField>& data)
{
  for (const auto& processor : this->config.metadataProcessors())
    processor->processMagneticField(this->fixHeader(data, this->config.frameId()));
}
void MovieMetadataListener::processFaces(const TimedMetadata<vision_msgs::Detection2DArray>& data)
{
  for (const auto& processor : this->config.metadataProcessors())
    processor->processFaces(this->fixHeader(data, this->config.frameId()));
}
void MovieMetadataListener::processCameraInfo(const TimedMetadata<sensor_msgs::CameraInfo>& data)
{
  for (const auto& processor : this->config.metadataProcessors())
    processor->processCameraInfo(this->fixHeader(data, this->config.opticalFrameId()));
}
void MovieMetadataListener::processImu(const TimedMetadata<sensor_msgs::Imu>& data)
{
  for (const auto& processor : this->config.metadataProcessors())
    processor->processImu(this->fixHeader(data, this->config.frameId()));
}
void MovieMetadataListener::processRollPitch(const TimedMetadata<std::pair<double, double>>& data)
{
  geometry_msgs::TransformStamped tf;
  tf.header.stamp = this->getTimestamp(data.stamp);
  tf.header.frame_id = this->config.frameId();
  tf.child_frame_id = this->config.frameId() + "_zero_roll_pitch";
  tf2::Quaternion quat;
  quat.setRPY(data.value.first, data.value.second, 0.0);
  tf.transform.rotation = tf2::toMsg(quat.inverse());
  for (const auto& processor : this->config.metadataProcessors())
    processor->processZeroRollPitchTf(tf);
}
void MovieMetadataListener::processOpticalFrameTF(const TimedMetadata<geometry_msgs::Transform>& data)
{
  geometry_msgs::TransformStamped tf;
  tf.header.stamp = this->getTimestamp(data.stamp);
  tf.header.frame_id = this->config.frameId();
  tf.child_frame_id = this->config.opticalFrameId();
  tf.transform = data.value;
  for (const auto& processor : this->config.metadataProcessors())
    processor->processOpticalTf(tf);
}

MoviePrivate::MoviePrivate(const cras::LogHelperPtr& log) : cras::HasLogger(log)
{
  this->info = std::make_shared<MovieInfo>();
  this->playbackState = std::make_shared<MoviePlaybackState>();

#if LIBAVFORMAT_VERSION_INT < AV_VERSION_INT(58, 9, 100)
  av_register_all();
#endif
#if LIBAVFILTER_VERSION_INT < AV_VERSION_INT(7, 14, 100)
  avfilter_register_all();
#endif
}

MoviePrivate::~MoviePrivate()
{
  this->metadataManager.reset();
}

bool MoviePrivate::isStillImage() const
{
  if (this->formatContext == nullptr || this->stream == nullptr)
    return false;

  if (this->stream->nb_frames == 1)
    return true;

  if (this->stream->duration == 1)
    return true;

  const std::string demuxerName = this->formatContext->iformat->name;
  if (demuxerName == "image2" || demuxerName == "png_pipe")
    return true;

  return false;
}

bool MoviePrivate::isSeekable() const
{
  if (this->formatContext == nullptr || this->codecContext == nullptr)
    return false;

#if LIBAVFORMAT_VERSION_INT >= AV_VERSION_INT(58, 6, 100)
  if (this->formatContext->ctx_flags & AVFMTCTX_UNSEEKABLE)
    return false;
#endif
  // Due to bug https://trac.ffmpeg.org/ticket/6113, JPEG "streams" cannot be seeked to the beginning
  if (std::string(this->formatContext->iformat->name) == "image2" && this->codecContext->codec_id == AV_CODEC_ID_MJPEG)
    return false;

  return true;
}

RationalNumber MoviePrivate::getFrameRate() const
{
  if (this->formatContext == nullptr || this->stream == nullptr)
    return RationalNumber{0, 1};

  // Static images
  if (this->stream->nb_frames == 1)
    return RationalNumber{0, 1};

  const auto eps = av_make_q(1, 1000);

  auto fps = this->stream->avg_frame_rate;
  if (av_cmp_q(fps, eps) < 0)
    fps = this->stream->r_frame_rate;
  if (av_cmp_q(fps, eps) < 0)
    fps = av_guess_frame_rate(this->formatContext, this->stream, nullptr);

  return fps.den != 0 ? RationalNumber{fps.num, fps.den} : RationalNumber{0, 1};
}

StreamDuration MoviePrivate::getContainerDuration() const
{
  if (this->formatContext == nullptr)
    return {0, 0};

  return {this->formatContext->duration, AV_TIME_BASE_Q};
}

StreamDuration MoviePrivate::getDuration() const
{
  if (this->stream == nullptr)
    return {0, 0};

  const auto streamDuration = this->getStreamDuration();
  if (!streamDuration.isZero())
    return streamDuration;

  return this->getContainerDuration();
}

StreamTime MoviePrivate::getStreamStart() const
{
  if (this->stream == nullptr)
    return {0, 0};
  return {this->stream->start_time, this->stream->time_base};
}

StreamTime MoviePrivate::getStreamEnd() const
{
  if (this->stream == nullptr || this->stream->start_time == AV_NOPTS_VALUE ||
      this->stream->duration == AV_NOPTS_VALUE || this->stream->duration == 0)
    return {0, 0};
  return {this->stream->start_time + this->stream->duration, this->stream->time_base};
}

StreamDuration MoviePrivate::getStreamDuration() const
{
  if (this->stream == nullptr)
    return {0, 0};
  return {this->stream->duration, this->stream->time_base};
}

size_t MoviePrivate::getNumFrames() const
{
  if (this->stream == nullptr)
    return 0.0;
  if (this->isStillImage())
    return 1;
  return static_cast<size_t>(this->stream->nb_frames);
}

void MoviePrivate::prepareMetadataExtractors()
{
  this->metadataManager = std::make_shared<MetadataManager>(this->log, *this->config, this->info);

  const MetadataExtractorParams params = {
    this->log, this->metadataManager, *this->config, this->info, this->formatContext, this->metadataManager->getCache(),
  };
  this->metadataManager->loadExtractorPlugins(params);

  this->metadataListener = std::make_shared<MovieMetadataListener>(
    *this->config, cras::bind_front(&MoviePrivate::getTimestamp, this));
  this->metadataManager->addTimedMetadataListener(this->metadataListener);

  this->metadataManager->prepareTimedMetadata(this->config->metadataTypes());
}

void MoviePrivate::extractMetadata()
{
  const auto rosStamp = this->getTimestamp(this->lastSeek);
  const auto& metadataTypes = this->config->metadataTypes();

  if (metadataTypes.find(MetadataType::GNSS_POSITION) != metadataTypes.end())
  {
    const auto [navMsg, gpsMsg] = this->metadataManager->getGNSSPosition();
    if (navMsg.has_value())
    {
      const auto rosNavMsg = updateHeader(*navMsg, rosStamp, this->config->frameId());
      for (const auto& processor : this->config->metadataProcessors())
        processor->processNavSatFix(rosNavMsg);
    }
    if (gpsMsg.has_value())
    {
      const auto rosGpsMsg = updateHeader(*gpsMsg, rosStamp, this->config->frameId());
      for (const auto& processor : this->config->metadataProcessors())
        processor->processGps(rosGpsMsg);
    }
  }

  if (metadataTypes.find(MetadataType::AZIMUTH) != metadataTypes.end())
  {
    const auto azimuth = this->metadataManager->getAzimuth();
    if (azimuth.has_value())
    {
      const auto azimuthMsg = updateHeader(*azimuth, rosStamp, this->config->frameId());
      for (const auto& processor : this->config->metadataProcessors())
        processor->processAzimuth(azimuthMsg);
    }
  }

  if (metadataTypes.find(MetadataType::MAGNETIC_FIELD) != metadataTypes.end())
  {
    const auto magField = this->metadataManager->getMagneticField();
    if (magField.has_value())
    {
      const auto magneticFieldMsg = updateHeader(*magField, rosStamp, this->config->frameId());
      for (const auto& processor : this->config->metadataProcessors())
        processor->processMagneticField(magneticFieldMsg);
    }
  }

  if (metadataTypes.find(MetadataType::IMU) != metadataTypes.end())
  {
    const auto imuMsg = this->metadataManager->getImu();
    if (imuMsg.has_value())
    {
      const auto rosImuMsg = updateHeader(*imuMsg, rosStamp, this->config->frameId());
      for (const auto& processor : this->config->metadataProcessors())
        processor->processImu(rosImuMsg);
    }
  }

  if (metadataTypes.find(MetadataType::ZERO_ROLL_PITCH_TF) != metadataTypes.end())
  {
    const auto zeroRollPitchTf = this->metadataManager->getZeroRollPitchTF();
    if (zeroRollPitchTf.has_value())
    {
      geometry_msgs::TransformStamped zeroRollPitchTfMsg;
      zeroRollPitchTfMsg.header.frame_id = this->config->frameId();
      zeroRollPitchTfMsg.child_frame_id = this->config->frameId() + "_zero_roll_pitch";
      zeroRollPitchTfMsg.header.stamp = rosStamp;
      zeroRollPitchTfMsg.transform = *zeroRollPitchTf;
      for (const auto& processor : this->config->metadataProcessors())
        processor->processZeroRollPitchTf(zeroRollPitchTfMsg);
    }
  }

  if (metadataTypes.find(MetadataType::OPTICAL_FRAME_TF) != metadataTypes.end())
  {
    const auto opticalTf = this->metadataManager->getOpticalFrameTF();
    if (opticalTf.has_value())
    {
      geometry_msgs::TransformStamped opticalTfMsg;
      opticalTfMsg.header.stamp = rosStamp;
      opticalTfMsg.header.frame_id = this->config->frameId();
      opticalTfMsg.transform = *opticalTf;
      opticalTfMsg.child_frame_id = this->config->opticalFrameId();
      for (const auto& processor : this->config->metadataProcessors())
        processor->processOpticalTf(opticalTfMsg);
    }
  }

  if (metadataTypes.find(MetadataType::FACES) != metadataTypes.end())
  {
    const auto facesMsg = this->metadataManager->getFaces();
    if (facesMsg.has_value())
    {
      const auto rosFacesMsg = updateHeader(*facesMsg, rosStamp, this->config->opticalFrameId());
      for (const auto& processor : this->config->metadataProcessors())
        processor->processFaces(rosFacesMsg);
    }
  }

  // Temporarily change to a UTF-8 locale so that we can print the ° characters.
  cras::TempLocale l(LC_CTYPE, "en_US.UTF-8");

  if (metadataTypes.find(MetadataType::CAMERA_UNIQUE_NAME) != metadataTypes.end())
  {
    const auto uniqueCamName = this->metadataManager->getCameraUniqueName();
    if (uniqueCamName.has_value())
    {
      CRAS_INFO("Camera: %s", uniqueCamName.value().c_str());
    }
    else if (metadataTypes.find(MetadataType::CAMERA_GENERAL_NAME) != metadataTypes.end())
    {
      const auto camName = this->metadataManager->getCameraGeneralName();
      if (camName.has_value())
        CRAS_INFO("Camera: %s", camName.value().c_str());
    }
  }
  if (metadataTypes.find(MetadataType::CREATION_TIME) != metadataTypes.end())
  {
    CRAS_INFO("Creation time: %s",
      cras::to_pretty_string(this->metadataManager->getCreationTime().value_or(ros::Time{})).c_str());
  }
  if (metadataTypes.find(MetadataType::ROTATION) != metadataTypes.end())
  {
    CRAS_INFO("Rotation is %d°.", this->metadataManager->getRotation().value_or(0));
  }
  if (metadataTypes.find(MetadataType::GNSS_POSITION) != metadataTypes.end())
  {
    const auto [navMsg, gpsMsg] = this->metadataManager->getGNSSPosition();
    if (navMsg.has_value() || gpsMsg.has_value())
    {
      const auto lat = navMsg.has_value() ? navMsg->latitude : gpsMsg->latitude;
      const auto lon = navMsg.has_value() ? navMsg->longitude : gpsMsg->longitude;
      const auto alt = navMsg.has_value() ? navMsg->altitude : gpsMsg->altitude;
      CRAS_INFO("GPS coordinates are %0.8f° %s, %0.8f° %s, %0.2f m.a.s.l.",
        std::fabs(lat), lat >= 0 ? "N" : "S", std::fabs(lon), lon >= 0 ? "E" : "W", alt);
    }
  }
  if (metadataTypes.find(MetadataType::AZIMUTH) != metadataTypes.end() &&
    this->metadataManager->getAzimuth().has_value())
  {
    const auto azimuthMsg = *this->metadataManager->getAzimuth();
    CRAS_INFO("Azimuth is %0.3f° from %s North.", azimuthMsg.azimuth,
      azimuthMsg.reference == compass_msgs::Azimuth::REFERENCE_GEOGRAPHIC ? "true" : "magnetic");
  }
  if (metadataTypes.find(MetadataType::IMU) != metadataTypes.end() && this->metadataManager->getImu().has_value())
  {
    const auto imuMsg = *this->metadataManager->getImu();
    const auto& orientation = imuMsg.orientation;
    tf2::Quaternion quat;
    tf2::convert(orientation, quat);
    tf2::Matrix3x3 m(quat);
    double r, p, y;
    m.getRPY(r, p, y);
    r *= 180.0 / M_PI;
    p *= 180.0 / M_PI;
    y *= 180.0 / M_PI;
    CRAS_INFO("Roll is %.1f°, pitch is %.1f°, yaw is %.1f°.", r, p, y);
    const auto& rates = imuMsg.angular_velocity;
    CRAS_INFO("Angular rate is %.2f %.2f %.2f rad/s.", rates.x, rates.y, rates.z);
    const auto& a = imuMsg.linear_acceleration;
    CRAS_INFO("Acceleration is %.2f %.2f %.2f m/s^2.", a.x, a.y, a.z);
  }
  if (metadataTypes.find(MetadataType::CAMERA_INFO) != metadataTypes.end() &&
    this->metadataManager->getCameraInfo().has_value())
  {
    const auto cameraInfoMsg = *this->metadataManager->getCameraInfo();
    if (cameraInfoMsg.K[0] != 0)
      CRAS_INFO("Camera projection is calibrated [fx=%0.1f, fy=%0.1f, cx=%0.1f, cy=%0.1f].",
        cameraInfoMsg.K[0], cameraInfoMsg.K[4], cameraInfoMsg.K[2], cameraInfoMsg.K[5]);
    if (!cameraInfoMsg.D.empty() && cameraInfoMsg.D[0] != 0)
      CRAS_INFO("Camera distortion is calibrated: %s@%s.",
        cameraInfoMsg.distortion_model.c_str(), cras::to_string(cameraInfoMsg.D).c_str());
  }
}

void MoviePrivate::updateMetadata(const StreamTime& ptsTime)
{
  auto metadataToUpdate = this->config->metadataTypes();
  decltype(metadataToUpdate) changed;
  // This should never be higher than 1; but we count it just in case
  size_t noUpdateIterations = 0;

  // Iterate over timed metadata extractors as long as new metadata are extracted (some extractors may depend on
  // metadata produced by other extractors; that is why we need to loop).
  do
  {
    changed.clear();
    for (const auto type : metadataToUpdate)
    {
      const auto numProcessed = this->metadataManager->processTimedMetadata(type, ptsTime, true);
      if (numProcessed > 0)
      {
        changed.insert(type);
        CRAS_DEBUG_THROTTLE_NAMED(1.0, std::string("timed_metadata.") + enum_name(type),
          "Produced %zu timed messages.", numProcessed);
      }
    }
    if (!changed.empty())
    {
      for (const auto& m : changed)
        metadataToUpdate.erase(m);
      continue;
    }
    // If no new metadata were produced, relax the requirement for optional metadata and try without them
    for (const auto type : metadataToUpdate)
    {
      const auto numProcessed = this->metadataManager->processTimedMetadata(type, ptsTime, false);
      if (numProcessed > 0)
      {
        changed.insert(type);
        CRAS_DEBUG_THROTTLE_NAMED(1.0, std::string("timed_metadata.") + enum_name(type),
          "Produced %zu timed messages.", numProcessed);
      }
    }
    if (!changed.empty())
    {
      for (const auto& m : changed)
        metadataToUpdate.erase(m);
      continue;
    }
    noUpdateIterations++;
    ROS_ASSERT_MSG(noUpdateIterations < 10, "Too many iterations of updateMetadata()");
  }
  while (!metadataToUpdate.empty() && !changed.empty());

  this->metadataManager->clearTimedMetadataCache();
}

ros::Time MoviePrivate::getTimestamp(const StreamTime& ptsTime) const
{
  ros::Time result;
  switch (this->config->timestampSource())
  {
    case TimestampSource::AbsoluteVideoTimecode:
      result = ptsTime.toRosTime();
    break;
    case TimestampSource::RelativeVideoTimecode:
      result = (ptsTime - (this->lastSeek - StreamTime{})).toRosTime();
    break;
    case TimestampSource::AllZeros:
      result = {0, 0};
    break;
    case TimestampSource::RosTime:
      result = ros::Time::now();
    break;
    case TimestampSource::FromMetadata:
      result = this->info->metadataStartTime() + (ptsTime - StreamTime{}).toRosDuration();
    break;
  }
  result += this->config->timestampOffset();
  return result;
}

cras::expected<std::pair<AVCodec*, int>, std::string> MoviePrivate::selectStream()
{
  int res = avformat_find_stream_info(this->formatContext, nullptr);
  if (res < 0)
    return cras::make_unexpected(cras::format("Could not get the stream info: %s", av_err2str(res)));

  AVCodec* codec {nullptr};

  int selectedStreamIndex {0};
  if (this->config->forceStreamIndex().has_value())
  {
    if (*this->config->forceStreamIndex() >= this->formatContext->nb_streams)
    {
      return cras::make_unexpected(cras::format(
        "Requested stream number %i, but file %s has only %i streams.",
        *this->config->forceStreamIndex(), this->config->filenameOrURL().c_str(), this->formatContext->nb_streams));
    }

    const auto& stream = this->formatContext->streams[*this->config->forceStreamIndex()];
    if (stream->codecpar->codec_type != AVMEDIA_TYPE_VIDEO)
    {
      return cras::make_unexpected(cras::format(
        "Stream %i in file %s is not a video stream.",
        *this->config->forceStreamIndex(), this->config->filenameOrURL().c_str()));
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
          *this->config->forceStreamIndex(), this->config->filenameOrURL().c_str(), stream->codecpar->codec_id));
      }
    }
    selectedStreamIndex = *this->config->forceStreamIndex();
  }
  else
  {
    selectedStreamIndex = av_find_best_stream(
      this->formatContext, AVMEDIA_TYPE_VIDEO, -1, -1, &codec, 0);
    if (selectedStreamIndex < 0)
    {
      if (selectedStreamIndex == AVERROR_STREAM_NOT_FOUND)
        return cras::make_unexpected(cras::format(
          "File %s does not contain a video stream!", this->config->filenameOrURL().c_str()));
      else if (selectedStreamIndex == AVERROR_DECODER_NOT_FOUND)
        cras::make_unexpected(cras::format(
          "File %s contains a video stream, but there is no codec for it!", this->config->filenameOrURL().c_str()));
      else
        return cras::make_unexpected(cras::format(
          "Error finding a suitable video stream in file %s!", this->config->filenameOrURL().c_str()));
    }
  }

  this->stream = this->formatContext->streams[selectedStreamIndex];
  const auto& codecParams = stream->codecpar;
  const auto inputPixFmt = static_cast<AVPixelFormat>(codecParams->format);
  const std::string inputPixFmtName = av_get_pix_fmt_name(inputPixFmt);

  CRAS_DEBUG("Stream properties:");
  CRAS_DEBUG("\ttime_base: %d/%d", this->stream->time_base.num, this->stream->time_base.den);
  CRAS_DEBUG("\tavg_frame_rate: %d/%d", this->stream->avg_frame_rate.num, this->stream->avg_frame_rate.den);
  CRAS_DEBUG("\tr_frame_rate: %d/%d", this->stream->r_frame_rate.num, this->stream->r_frame_rate.den);
  CRAS_DEBUG("\tnb_frames: %" PRId64, this->stream->nb_frames);
  CRAS_DEBUG("\tstream start_time: %" PRId64, this->stream->start_time);
  CRAS_DEBUG("\tstream duration: %s s",
    cras::to_string(StreamDuration(this->stream->duration, this->stream->time_base)).c_str());
  CRAS_DEBUG("\tduration: %s s", cras::to_string(this->getDuration()).c_str());
  CRAS_DEBUG("\tpixfmt: %s", inputPixFmtName.c_str());

  CRAS_INFO("Video Codec: resolution %d x %d, codec %s, ID %d, bit_rate %ld Bps, FPS %.2f",
    codecParams->width, codecParams->height, codec->name, codec->id, codecParams->bit_rate,
    static_cast<double>(this->getFrameRate()));

  return std::make_pair(codec, selectedStreamIndex);
}

cras::expected<void, std::string> MoviePrivate::openCodec(const AVCodec* codec)
{
  this->codecContext = avcodec_alloc_context3(codec);
  int res = avcodec_parameters_to_context(this->codecContext, this->stream->codecpar);
  if (res < 0)
    cras::make_unexpected(cras::format("Failed to copy codec params to codec context: %s", av_err2str(res)));

  AVDictionary* codecOpts = nullptr;
  av_dict_set(&codecOpts, "threads", cras::to_string(this->config->numThreads()).c_str(), 0);

  res = avcodec_open2(this->codecContext, codec, &codecOpts);
  if (res < 0)
  {
    av_dict_free(&codecOpts);
    cras::make_unexpected(cras::format("Failed to open codec through avcodec_open2: %s", av_err2str(res)));
  }
  av_dict_free(&codecOpts);

  return {};
}

void MoviePrivate::detectTargetPixelFormat()
{
  const auto inputPixFmt = static_cast<AVPixelFormat>(this->stream->codecpar->format);
  const std::string inputPixFmtName = av_get_pix_fmt_name(inputPixFmt);

  // Detect which encoding and pixel format will be used for the conversion to ROS image.
  if (this->config->forceEncoding().has_value())
  {
    if (rosEncodingToAvPixFmt(this->config->forceEncoding().value()).has_value())
    {
      this->targetPixelFormat = *rosEncodingToAvPixFmt(this->config->forceEncoding().value());
    }
    else
    {
      this->targetPixelFormat = *rosEncodingToAvPixFmt(this->config->defaultEncoding());
      CRAS_WARN(
        "ROS encoding '%s' is not supported. Converting to default encoding '%s' instead.",
        this->config->forceEncoding()->c_str(), this->config->defaultEncoding().c_str());
    }
  }
  else
  {
    // There is a direct pixfmt->ros mapping
    if (avPixFmtToRosEncoding(inputPixFmt).has_value())
    {
      this->targetPixelFormat = inputPixFmt;
    }
    // There is no direct mapping, use a fallback
    else
    {
      if (this->config->allowYUVFallback() && cras::startsWith(cras::toLower(inputPixFmtName), "yuv"))
      {
        this->targetPixelFormat = *rosEncodingToAvPixFmt(sensor_msgs::image_encodings::YUV422);
        CRAS_DEBUG(
          "Pixel format '%s' has no corresponding ROS encoding. Converting to default YUV encoding '%s'.",
          inputPixFmtName.c_str(), (*avPixFmtToRosEncoding(this->targetPixelFormat)).c_str());
      }
      else
      {
        this->targetPixelFormat = *rosEncodingToAvPixFmt(this->config->defaultEncoding());
        CRAS_DEBUG(
          "Pixel format '%s' has no corresponding ROS encoding. Converting to default encoding '%s'.",
          inputPixFmtName.c_str(), this->config->defaultEncoding().c_str());
      }
    }
  }

  CRAS_DEBUG("Converting input pixel format '%s' to ROS encoding '%s' using pixel format '%s'.",
             inputPixFmtName.c_str(), (*avPixFmtToRosEncoding(this->targetPixelFormat)).c_str(),
             av_get_pix_fmt_name(this->targetPixelFormat));
}

cras::expected<void, std::string> MoviePrivate::addRotationFilter()
{
  auto buffersrc = avfilter_get_by_name("buffer");
  auto buffersink = avfilter_get_by_name("buffersink");
  if (!buffersink)
    buffersink = avfilter_get_by_name("ffbuffersink");

  if (buffersrc == nullptr || buffersink == nullptr)
    return cras::make_unexpected("Error finding buffer or buffersink filters. That should not happen.");

  this->filterGraph = avfilter_graph_alloc();
  this->filterGraph->nb_threads = 1;

  const auto& codecParams = this->stream->codecpar;
  const auto args = cras::format(
    "video_size=%dx%d:pix_fmt=%d:time_base=%d/%d:pixel_aspect=%d/%d",
    codecParams->width, codecParams->height, codecParams->format,
    this->stream->time_base.num, this->stream->time_base.den,
    this->stream->sample_aspect_ratio.num, this->stream->sample_aspect_ratio.den);
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
  if (this->info->metadataRotation() == 90.0)
    filterDesc = "transpose=1";
  else if (this->info->metadataRotation() == 180.0)
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

cras::expected<void, std::string> MoviePrivate::configSwscale()
{
  const auto& codecParams = this->stream->codecpar;
  const auto swapDims = this->info->metadataRotation() == 90 || this->info->metadataRotation() == 270;
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
