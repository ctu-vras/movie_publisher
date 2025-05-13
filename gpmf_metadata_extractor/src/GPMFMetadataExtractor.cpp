// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Metadata extractor from GPMF streams.
 * \author Martin Pecka
 */

#include "GPMFMetadataExtractor.h"

#include <queue>

extern "C"
{
#include <libavformat/avformat.h>
}

#include <cras_cpp_common/type_utils.hpp>
#include <movie_publisher/metadata_cache.h>
#include <pluginlib/class_list_macros.h>

#include "gpmf_parser/GPMF_parser.h"
#include "gpmf_parser/GPMF_utils.h"

namespace movie_publisher
{

struct GPMFMetadataPrivate : public cras::HasLogger
{
  explicit GPMFMetadataPrivate(const cras::LogHelperPtr& log) : cras::HasLogger(log) {}

  std::weak_ptr<MetadataManager> manager;  //!< Metadata manager
  int priority {15};  //!< Priority of this extractor. Keep it rather low because this extractor is actually cheap.

  const AVFormatContext* avFormatContext {nullptr};  //!< Context of the open movie file.
  cras::optional<int> gpmdStreamIndex;  //!< Index of the gpmd stream in movie container.
  cras::optional<int> tmcdStreamIndex;  //!< Index of the tmcd stream in movie container.

  size_t width {0u};  //!< Movie width in px.
  size_t height {0u};  //!< Movie height in px.

  StreamTime lastTime;  //!< Stream timestamp of the last call to processTimedMetadata().
  MetadataCache cache;  //!< Buffer for messages decoded from libav packets.

  size_t numGPMDPackets {0u};  //!< The number of processed GPMD packets.

  //! The set of timed metadata requested to be produced by this extractor. Empty means all.
  std::unordered_set<MetadataType> requestedTimedMetadata;
  //! Metadata supported by the currently loaded movie.
  std::unordered_set<MetadataType> supportedTimedMetadata;

  /**
   * \brief Process a packet that was identified to belong to the gpmd stream.
   * \param[in] packet The decoded libav packet. Either do not store it, or av_packet_ref() it.
   */
  void processGpmdPacket(const AVPacket* packet);

  /**
   * \brief Process a packet that was identified to belong to the tmcd stream.
   * \param[in] packet The decoded libav packet. Either do not store it, or av_packet_ref() it.
   */
  void processTmcdPacket(const AVPacket* packet);
};

GPMFMetadataExtractor::GPMFMetadataExtractor(
  const cras::LogHelperPtr& log, const std::weak_ptr<MetadataManager>& manager,
  const size_t width, const size_t height, const AVFormatContext* avFormatContext, const int priority)
  : TimedMetadataExtractor(log), data(new GPMFMetadataPrivate(log))
{
  this->data->manager = manager;
  this->data->width = width;
  this->data->height = height;
  this->data->avFormatContext = avFormatContext;
  this->data->priority = priority;

  // Find gpmd and tmcd streams
  for (size_t i = 0; i < avFormatContext->nb_streams; i++)
  {
    if (avFormatContext->streams[i]->codecpar->codec_tag == MKTAG('g', 'p', 'm', 'd'))
      this->data->gpmdStreamIndex = i;
    else if (avFormatContext->streams[i]->codecpar->codec_tag == MKTAG('t', 'm', 'c', 'd'))
      this->data->tmcdStreamIndex = i;
  }

  if (this->data->gpmdStreamIndex.has_value())
  {
    CRAS_INFO_NAMED("gpmf", "Found timed metadata track.");

    // TODO check if this list is complete, but only list types that can be read directly without combining lower-level
    //      data. It is not important that all these data are in the stream, but if we have implementation for
    //      extracting data that are there.
    this->data->supportedTimedMetadata = {
      // MetadataType::CROP_FACTOR,
      // MetadataType::SENSOR_SIZE_MM,
      // MetadataType::DISTORTION,
      // MetadataType::ROTATION,
      // MetadataType::FOCAL_LENGTH_MM,
      // MetadataType::FOCAL_LENGTH_35MM,
      // MetadataType::FOCAL_LENGTH_PX,
      // MetadataType::INTRINSIC_MATRIX,
      // MetadataType::AZIMUTH,
      MetadataType::ROLL_PITCH,
      MetadataType::GNSS_POSITION,
      MetadataType::ACCELERATION,
      // MetadataType::MAGNETIC_FIELD,
      MetadataType::ANGULAR_VELOCITY,
      MetadataType::FACES,
    };
  }
}

GPMFMetadataExtractor::~GPMFMetadataExtractor() = default;

std::string GPMFMetadataExtractor::getName() const
{
  return cras::getTypeName<std::remove_cv_t<std::remove_reference_t<decltype(*this)>>>();
}

int GPMFMetadataExtractor::getPriority() const
{
  return this->data->priority;
}

cras::optional<double> GPMFMetadataExtractor::getCropFactor()
{
  if (this->data->cache.latest.getCropFactor().has_value())
    return this->data->cache.latest.getCropFactor()->value();
  return cras::nullopt;
}

cras::optional<std::pair<double, double>> GPMFMetadataExtractor::getSensorSizeMM()
{
  if (this->data->cache.latest.getSensorSizeMM().has_value())
    return this->data->cache.latest.getSensorSizeMM()->value();
  return cras::nullopt;
}

cras::optional<double> GPMFMetadataExtractor::getFocalLengthMM()
{
  if (this->data->cache.latest.getFocalLengthMM().has_value())
    return this->data->cache.latest.getFocalLengthMM()->value();
  return cras::nullopt;
}

cras::optional<std::pair<DistortionType, Distortion>> GPMFMetadataExtractor::getDistortion()
{
  if (this->data->cache.latest.getDistortion().has_value())
    return this->data->cache.latest.getDistortion()->value();
  return cras::nullopt;
}

cras::optional<std::string> GPMFMetadataExtractor::getCameraSerialNumber()
{
  if (this->data->cache.latest.getCameraSerialNumber().has_value())
    return this->data->cache.latest.getCameraSerialNumber()->value();
  return cras::nullopt;
}

cras::optional<std::string> GPMFMetadataExtractor::getCameraMake()
{
  if (this->data->cache.latest.getCameraMake().has_value())
    return this->data->cache.latest.getCameraMake()->value();
  return cras::nullopt;
}

cras::optional<std::string> GPMFMetadataExtractor::getCameraModel()
{
  if (this->data->cache.latest.getCameraModel().has_value())
    return this->data->cache.latest.getCameraModel()->value();
  return cras::nullopt;
}

cras::optional<std::string> GPMFMetadataExtractor::getLensMake()
{
  if (this->data->cache.latest.getLensMake().has_value())
    return this->data->cache.latest.getLensMake()->value();
  return cras::nullopt;
}

cras::optional<std::string> GPMFMetadataExtractor::getLensModel()
{
  if (this->data->cache.latest.getLensModel().has_value())
    return this->data->cache.latest.getLensModel()->value();
  return cras::nullopt;
}

cras::optional<int> GPMFMetadataExtractor::getRotation()
{
  if (this->data->cache.latest.getRotation().has_value())
    return this->data->cache.latest.getRotation()->value();
  return cras::nullopt;
}

cras::optional<ros::Time>GPMFMetadataExtractor::getCreationTime()
{
  if (this->data->cache.latest.getCreationTime().has_value())
    return this->data->cache.latest.getCreationTime()->value();
  return cras::nullopt;
}

cras::optional<double> GPMFMetadataExtractor::getFocalLength35MM()
{
  if (this->data->cache.latest.getFocalLength35MM().has_value())
    return this->data->cache.latest.getFocalLength35MM()->value();
  return cras::nullopt;
}

cras::optional<double> GPMFMetadataExtractor::getFocalLengthPx()
{
  if (this->data->cache.latest.getFocalLengthPx().has_value())
    return this->data->cache.latest.getFocalLengthPx()->value();
  return cras::nullopt;
}

cras::optional<IntrinsicMatrix> GPMFMetadataExtractor::getIntrinsicMatrix()
{
  if (this->data->cache.latest.getIntrinsicMatrix().has_value())
    return this->data->cache.latest.getIntrinsicMatrix()->value();
  return cras::nullopt;
}

GNSSFixAndDetail GPMFMetadataExtractor::getGNSSPosition()
{
  if (this->data->cache.latest.getGNSSPosition().has_value())
    return this->data->cache.latest.getGNSSPosition().value();
  return {cras::nullopt, cras::nullopt};
}

cras::optional<compass_msgs::Azimuth> GPMFMetadataExtractor::getAzimuth()
{
  if (this->data->cache.latest.getAzimuth().has_value())
    return this->data->cache.latest.getAzimuth()->value();
  return cras::nullopt;
}

cras::optional<std::pair<double, double>> GPMFMetadataExtractor::getRollPitch()
{
  if (this->data->cache.latest.getRollPitch().has_value())
    return this->data->cache.latest.getRollPitch()->value();
  return cras::nullopt;
}

cras::optional<geometry_msgs::Vector3> GPMFMetadataExtractor::getAcceleration()
{
  if (this->data->cache.latest.getAcceleration().has_value())
    return this->data->cache.latest.getAcceleration()->value();
  return cras::nullopt;
}

cras::optional<sensor_msgs::MagneticField> GPMFMetadataExtractor::getMagneticField()
{
  if (this->data->cache.latest.getMagneticField().has_value())
    return this->data->cache.latest.getMagneticField()->value();
  return cras::nullopt;
}

cras::optional<geometry_msgs::Vector3> GPMFMetadataExtractor::getAngularVelocity()
{
  if (this->data->cache.latest.getAngularVelocity().has_value())
    return this->data->cache.latest.getAngularVelocity()->value();
  return cras::nullopt;
}

cras::optional<vision_msgs::Detection2DArray> GPMFMetadataExtractor::getFaces()
{
  if (this->data->cache.latest.getFaces().has_value())
    return this->data->cache.latest.getFaces()->value();
  return cras::nullopt;
}

void GPMFMetadataExtractor::prepareTimedMetadata(const std::unordered_set<MetadataType>& types)
{
  // TODO some stuff that needs to be done before starting to extract timed metadata from the video
  //      Called with a list of metadata that should be extracted (if empty, extract all).
  //      This can e.g. read the first second of the video to get all the static metadata like camera model etc.
  this->data->requestedTimedMetadata = types;
}

void GPMFMetadataExtractor::processPacket(const AVPacket* packet)
{
  if (this->data->gpmdStreamIndex.has_value() && packet->stream_index == *this->data->gpmdStreamIndex)
    this->data->processGpmdPacket(packet);
  if (this->data->tmcdStreamIndex.has_value() && packet->stream_index == *this->data->tmcdStreamIndex)
    this->data->processTmcdPacket(packet);
}

void GPMFMetadataPrivate::processGpmdPacket(const AVPacket* packet)
{
  this->numGPMDPackets++;

  // Figure the stream time of the packet
  const auto& timeBase = this->avFormatContext->streams[packet->stream_index]->time_base;
  StreamTime packetTime(packet->pts, RationalNumber{timeBase.num, timeBase.den});

  CRAS_DEBUG_STREAM_NAMED("gpmf", "gpmd packet " << this->numGPMDPackets << " stamp " << packetTime.toRosTime());

  this->cache.latest.getCameraMake().emplace().emplace() = "GoPro";  // TODO hard-code to "GoPro" ?
  // this->cache.latest.getCameraModel().emplace().emplace() = "Hero 13 Black";  // TODO MINF
  // this->cache.latest.getCameraSerialNumber().emplace().emplace() = "1234";  // TODO CASN
  this->cache.latest.getLensMake().emplace().emplace() = "GoPro";  // TODO hard-code to "GoPro" ?
  // this->cache.latest.getLensModel().emplace().emplace() = "lens";  // TODO LINF???

  // Initialize packet in parser
  const auto time = packetTime + StreamDuration(0.01);

  uint32_t* payload = reinterpret_cast<uint32_t*>(packet->data);
  uint32_t payloadsize = packet->size;
  GPMF_stream g_stream;
  uint32_t samples, elements, buffersize;
  uint64_t lastTimestamp_us(0);
  if (GPMF_OK != GPMF_Init(&g_stream, payload, payloadsize))
  {
    CRAS_WARN_THROTTLE_NAMED(1.0, "gpmf", "Could not initialize GPMD packet.");
    return;
  }

  do
  {
    switch (GPMF_Key(&g_stream))
    {
      case STR2FOURCC("STMP"):
      {
        if (GPMF_OK != GPMF_FormattedData(&g_stream, &lastTimestamp_us, sizeof(uint64_t), 0, 1)) {
          CRAS_WARN_THROTTLE_NAMED(1.0, "gpmf", "Failed parsing STMP.");
          break;
        }
      }
      case STR2FOURCC("ACCL"):
      {
        samples = GPMF_Repeat(&g_stream);
        elements = GPMF_ElementsInStruct(&g_stream);
        buffersize = samples * elements;
        std::vector<double> tmp_buf(buffersize);
        if (GPMF_OK != GPMF_ScaledData(&g_stream, tmp_buf.data(), buffersize * sizeof(double),
          0, samples, GPMF_TYPE_DOUBLE)) {
          CRAS_WARN_THROTTLE_NAMED(1.0, "gpmf", "Failed parsing ACCL.");
          break;
        }

        // calculate offset between accl entries based on the frequency, in microseconds
        const uint64_t entry_offset_us = 1'000'000 / 200;
        for (size_t i = 0; i < samples; i++)
        {
          TimedMetadata<geometry_msgs::Vector3> msg;
          msg.stamp = StreamTime((lastTimestamp_us + i * entry_offset_us)/ 1e6);
          msg.value.z = tmp_buf[i];
          msg.value.x = tmp_buf[i + 1];
          msg.value.y = tmp_buf[i + 2];
          // Sanity check, the values are sometimes crazy
          if (std::abs(msg.value.x) > 100 || std::abs(msg.value.y) > 100 || std::abs(msg.value.z) > 100)
            continue;
          this->cache.timed.acceleration().push_back(msg);
        }
        break;
      }

      case STR2FOURCC("GYRO"):
      {
        samples = GPMF_Repeat(&g_stream);
        elements = GPMF_ElementsInStruct(&g_stream);
        buffersize = samples * elements;
        std::vector<double> tmp_buf(buffersize);
        if (GPMF_OK != GPMF_ScaledData(&g_stream, tmp_buf.data(), buffersize * sizeof(double),
          0, samples, GPMF_TYPE_DOUBLE)) {
          CRAS_WARN_THROTTLE_NAMED(1.0, "gpmf", "Failed parsing GYRO.");
          break;
        }

        // calculate offset between accl entries based on the frequency, in microseconds
        const uint64_t entry_offset_us = 1'000'000 / 200;
        for (size_t i = 0; i < samples; i++)
        {
          TimedMetadata<geometry_msgs::Vector3> msg;
          msg.stamp = StreamTime((lastTimestamp_us + i * entry_offset_us)/ 1e6);
          msg.value.z = tmp_buf[i];
          msg.value.x = tmp_buf[i + 1];
          msg.value.y = tmp_buf[i + 2];
          // Sanity check, the values are sometimes crazy
          if (std::abs(msg.value.x) > 10 || std::abs(msg.value.y) > 10 || std::abs(msg.value.z) > 10)
            continue;
          this->cache.timed.angularVelocity().push_back(msg);
        }
        break;
      }

      case STR2FOURCC("GPS9"):
      {
        samples = GPMF_Repeat(&g_stream);
        elements = GPMF_ElementsInStruct(&g_stream);
        buffersize = samples * elements;
        std::vector<double> tmp_buf(buffersize);

        std::cout.flush();
        if (GPMF_OK != GPMF_ScaledData(&g_stream, tmp_buf.data(), buffersize * sizeof(double),
          0, samples, GPMF_TYPE_DOUBLE)) {
          CRAS_WARN_THROTTLE_NAMED(1.0, "gpmf", "Failed parsing GPS9.");
          break;
        }

        // calculate offset between accl entries based on the frequency, in microseconds
        const uint64_t entry_offset_us = 1'000'000 / 10;
        for (size_t i = 0; i < samples; i++) {
          double latitude = tmp_buf[i * elements];
          double longitude = tmp_buf[i * elements + 1];
          double altitude = tmp_buf[i * elements + 2];
          double speed_2d = tmp_buf[i * elements + 3];
          double speed_3d = tmp_buf[i * elements + 4];
          uint64_t days_since_2000 = tmp_buf[i * elements + 5];
          double seconds_since_midnight = tmp_buf[i * elements + 6];
          double dop = tmp_buf[i * elements + 7];
          uint64_t fix_type = tmp_buf[i * elements + 8];

          cras::optional<sensor_msgs::NavSatFix> navSatFix;
          navSatFix.emplace();
          navSatFix->latitude = latitude;
          navSatFix->longitude = longitude;
          navSatFix->altitude = altitude;

          cras::optional<gps_common::GPSFix> gpsFix;
          gpsFix.emplace();
          gpsFix->latitude = latitude;
          gpsFix->longitude = longitude;
          gpsFix->altitude = altitude;
          gpsFix->speed = speed_2d;  // or 3d speed?
          gpsFix->gdop = dop;

          StreamTime time = StreamTime((lastTimestamp_us + i * entry_offset_us)/ 1e6);
          TimedMetadata<GNSSFixAndDetail> msg = {time, std::make_pair(navSatFix, gpsFix)};
          this->cache.timed.gnssPosition().push_back(msg);
        }
      }
      case STR2FOURCC("GRAV"):
      {
        samples = GPMF_Repeat(&g_stream);
        elements = GPMF_ElementsInStruct(&g_stream);
        buffersize = samples * elements;
        std::vector<double> tmp_buf(buffersize);
        if (GPMF_OK != GPMF_ScaledData(&g_stream, tmp_buf.data(), buffersize * sizeof(double),
          0, samples, GPMF_TYPE_DOUBLE)) {
          CRAS_WARN_THROTTLE_NAMED(1.0, "gpmf", "Failed parsing GRAV.");
          break;
        }

        // frequency GRAV = framerate
        // calculate offset between accl entries based on the frequency, in microseconds
        const uint64_t entry_offset_us = 1'000'000 / 24;
        for (size_t i = 0; i < samples; i++)
        {
          double gx = tmp_buf[i * elements];
          double gy = tmp_buf[i * elements + 1];
          double gz = tmp_buf[i * elements + 2];

          TimedMetadata<std::pair<double, double>> msg;
          msg.stamp = StreamTime((lastTimestamp_us + i * entry_offset_us)/ 1e6);
          // in radians
          msg.value.first = atan2(gy, gz);  // roll
          msg.value.second = atan2(-gx, sqrt(gy * gy + gz * gz));  // pitch
          this->cache.timed.rollPitch().push_back(msg);
        }
        break;
      }
      case STR2FOURCC("FACE"):
      {
        samples = GPMF_Repeat(&g_stream);
        elements = GPMF_ElementsInStruct(&g_stream);
        buffersize = samples * elements;
        std::vector<double> tmp_buf(buffersize);

        if (!samples)
          break;

        if (GPMF_OK != GPMF_ScaledData(&g_stream, tmp_buf.data(), buffersize * sizeof(double),
          0, samples, GPMF_TYPE_DOUBLE))
        {
          CRAS_WARN_THROTTLE_NAMED(1.0, "gpmf", "Failed parsing FACE.");
          break;
        }

        // frequency FACE ~ 10 / 12 depending on framerate on samples its 10
        // calculate offset between accl entries based on the frequency, in microseconds
        const uint64_t entry_offset_us = 1'000'000 / 10;
        for (size_t i = 0; i < samples; i++)
        {
          double x = tmp_buf[i * elements + 3] * this->width;
          double y = tmp_buf[i * elements + 4] * this->height;
          double w = tmp_buf[i * elements + 5] * this->width;
          double h = tmp_buf[i * elements + 6] * this->height;

          double center_x = x + w / 2.0f;
          double center_y = y + h / 2.0f;

          TimedMetadata<vision_msgs::Detection2DArray> msg;
          msg.stamp = StreamTime((lastTimestamp_us + i * entry_offset_us)/ 1e6);
          // msg.value.header.stamp whats this ?
          msg.value.detections.emplace_back();
          msg.value.detections.back().bbox.center.x = center_x;
          msg.value.detections.back().bbox.center.y = center_y;
          msg.value.detections.back().bbox.size_x = w;
          msg.value.detections.back().bbox.size_y = h;

          this->cache.timed.faces().push_back(msg);
        }
        break;
      }
      // both CORI and IORI are items with 4 elements
      case STR2FOURCC("CORI"):
      {
        samples = GPMF_Repeat(&g_stream);
        elements = GPMF_ElementsInStruct(&g_stream);
      }
      case STR2FOURCC("IORI"):
      {
        samples = GPMF_Repeat(&g_stream);
        elements = GPMF_ElementsInStruct(&g_stream);
      }
      default:  // if you don't know the Key you can skip to the next
      break;
    }
  } while (GPMF_OK == GPMF_Next(&g_stream, GPMF_RECURSE_LEVELS));
}

void GPMFMetadataPrivate::processTmcdPacket(const AVPacket* packet)
{
  static size_t num = 0;
  ++num;

  const auto& timeBase = this->avFormatContext->streams[packet->stream_index]->time_base;
  StreamTime packetTime(packet->pts, RationalNumber{timeBase.num, timeBase.den});

  CRAS_DEBUG_STREAM_NAMED("gpmf", "tmcd packet " << num << " stamp " << packetTime.toRosTime());

  // this->cache.latest.getCreationTime().emplace().emplace() = ros::Time::now();  // TODO read from the tmcd content?
  // TODO or CDAT, maybe TZON
}

/**
 * \brief Helper function to pass queued metadata to listeners for all messages in the queue up to maxTime.
 * \tparam T Type of metadata.
 * \param[in] bufferedData The queue of buffered metadata to be processed.
 * \param[in] lastData The cache of last data that should be updated.
 * \param[in] processFn The listener function that should be called on each listener.
 * \param[in] listeners The list of listeners.
 * \param[in] maxTime Max time until which the metadata should be taken ouf the the buffer.
 */
template<typename T>
size_t proc(std::vector<TimedMetadata<T>>& bufferedData, cras::optional<cras::optional<T>>& lastData,
  void(TimedMetadataListener::*processFn)(const TimedMetadata<T>&),
  const std::vector<std::shared_ptr<TimedMetadataListener>>& listeners, const StreamTime& maxTime)
{
  size_t numProcessed {0u};
  for (; !bufferedData.empty(); bufferedData.erase(bufferedData.begin()))
  {
    const auto& msg = bufferedData.front();
    if (msg.stamp > maxTime)
      break;
    lastData.emplace().emplace() = msg.value;
    numProcessed++;
    for (const auto& listener : listeners)
      (listener.get()->*processFn)(msg);
  }
  return numProcessed;
}

size_t proc(std::vector<TimedMetadata<GNSSFixAndDetail>>& bufferedData, cras::optional<GNSSFixAndDetail>& lastData,
  void(TimedMetadataListener::*processFn)(const TimedMetadata<GNSSFixAndDetail>&),
  const std::vector<std::shared_ptr<TimedMetadataListener>>& listeners, const StreamTime& maxTime)
{
  size_t numProcessed {0u};
  for (; !bufferedData.empty(); bufferedData.erase(bufferedData.begin()))
  {
    const auto& msg = bufferedData.front();
    if (msg.stamp > maxTime)
      break;
    lastData.emplace() = msg.value;
    numProcessed++;
    for (const auto& listener : listeners)
      (listener.get()->*processFn)(msg);
  }
  return numProcessed;
}

size_t GPMFMetadataExtractor::processTimedMetadata(
  const MetadataType type, const StreamTime& maxTime, const bool requireOptional)
{
  if (this->data->requestedTimedMetadata.count(type) == 0)
    return 0;

  // Shorthands to keep the following code short.
  auto& buffer = this->data->cache.timed;
  auto& last = this->data->cache.latest;
  auto& l = this->listeners;
  const auto& t = maxTime;
  size_t n {0u};

  switch (type)
  {
  case MetadataType::CROP_FACTOR:
    n += proc(buffer.cropFactor(), last.getCropFactor(), &TimedMetadataListener::processCropFactor, l, t);
    break;
  case MetadataType::SENSOR_SIZE_MM:
    n += proc(buffer.sensorSizeMM(), last.getSensorSizeMM(), &TimedMetadataListener::processSensorSizeMM, l, t);
    break;
  case MetadataType::DISTORTION:
    n += proc(buffer.distortion(), last.getDistortion(), &TimedMetadataListener::processDistortion, l, t);
    break;
  case MetadataType::ROTATION:
    n += proc(buffer.rotation(), last.getRotation(), &TimedMetadataListener::processRotation, l, t);
    break;
  case MetadataType::FOCAL_LENGTH_MM:
    n += proc(buffer.focalLengthMM(), last.getFocalLengthMM(), &TimedMetadataListener::processFocalLengthMM, l, t);
    break;
  case MetadataType::FOCAL_LENGTH_35MM:
    n += proc(buffer.focalLength35MM(), last.getFocalLength35MM(),
      &TimedMetadataListener::processFocalLength35MM, l, t);
    break;
  case MetadataType::FOCAL_LENGTH_PX:
    n += proc(buffer.focalLengthPx(), last.getFocalLengthPx(), &TimedMetadataListener::processFocalLengthPx, l, t);
    break;
  case MetadataType::INTRINSIC_MATRIX:
    n += proc(buffer.intrinsicMatrix(), last.getIntrinsicMatrix(),
      &TimedMetadataListener::processIntrinsicMatrix, l, t);
    break;
  case MetadataType::AZIMUTH:
    n += proc(buffer.azimuth(), last.getAzimuth(), &TimedMetadataListener::processAzimuth, l, t);
    break;
  case MetadataType::ROLL_PITCH:
    n += proc(buffer.rollPitch(), last.getRollPitch(), &TimedMetadataListener::processRollPitch, l, t);
    break;
  case MetadataType::GNSS_POSITION:
    n += proc(buffer.gnssPosition(), last.getGNSSPosition(), &TimedMetadataListener::processGNSSPosition, l, t);
    break;
  case MetadataType::ACCELERATION:
    n += proc(buffer.acceleration(), last.getAcceleration(), &TimedMetadataListener::processAcceleration, l, t);
    break;
  case MetadataType::MAGNETIC_FIELD:
    n += proc(buffer.magneticField(), last.getMagneticField(), &TimedMetadataListener::processMagneticField, l, t);
    break;
  case MetadataType::ANGULAR_VELOCITY:
    n += proc(buffer.angularVelocity(), last.getAngularVelocity(), &TimedMetadataListener::processAngularVelocity,
      l, t);
    break;
  case MetadataType::FACES:
    n += proc(buffer.faces(), last.getFaces(), &TimedMetadataListener::processFaces, l, t);
    break;
  default:
    return 0;
  }

  this->data->lastTime = maxTime;
  return n;
}

void GPMFMetadataExtractor::seekTimedMetadata(const StreamTime& seekTime)
{
  TimedMetadataExtractor::seekTimedMetadata(seekTime);
  CRAS_DEBUG_NAMED("gpmf", "seek to %s", cras::to_string(seekTime).c_str());

  // When the movie is seeked, we need to flush everything we buffered and wait for reading new data.
  this->data->lastTime = seekTime;
  this->data->cache.latest.clear();
  this->data->cache.timed.clear();
}

bool GPMFMetadataExtractor::hasTimedMetadata() const
{
  return this->data->gpmdStreamIndex.has_value();
}

std::unordered_set<MetadataType> GPMFMetadataExtractor::supportedTimedMetadata(
  const std::unordered_set<MetadataType>& availableMetadata) const
{
  return this->data->supportedTimedMetadata;
}

MetadataExtractor::Ptr GPMFMetadataExtractorPlugin::getExtractor(const MetadataExtractorParams& params)
{
  if (params.log == nullptr || params.manager.lock() == nullptr)
    return nullptr;
  if (params.info->width() == 0 || params.info->height() == 0)
    return nullptr;

  const int priority = 15;
  return std::make_shared<GPMFMetadataExtractor>(
    params.log, params.manager, params.info->width(), params.info->height(), params.avFormatContext, priority);
}

}

PLUGINLIB_EXPORT_CLASS(movie_publisher::GPMFMetadataExtractorPlugin, movie_publisher::MetadataExtractorPlugin)
