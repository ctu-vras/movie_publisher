// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Metadata extractor from GPMF streams.
 * \author Martin Pecka
 */

#include "GPMFMetadataExtractor.h"

#include <queue>
#include <unordered_map>
#include <unordered_set>

#include <Eigen/Geometry>

extern "C"
{
#include <libavformat/avformat.h>
}

#include <cras_cpp_common/tf2_utils.hpp>
#include <cras_cpp_common/type_utils.hpp>
#include <movie_publisher/metadata_cache.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>

#include "gpmf_parser/GPMF_parser.h"
#include "gpmf_parser/GPMF_utils.h"

namespace movie_publisher
{

class RateEstimator
{
public:
  void addSample(const size_t numNewSamples, const StreamDuration& duration)
  {
    if (numNewSamples == 0 || duration.isZero())
      return;
    this->numSamples += numNewSamples;
    this->totalDuration += duration;
  }

  StreamDuration entryOffset() const
  {
    if (this->numSamples == 0)
      return {};
    return this->totalDuration * (1.0 / static_cast<double>(this->numSamples));
  }

  void reset()
  {
    this->numSamples = 0;
    this->totalDuration = {0, 0};
  }

private:
  size_t numSamples {0u};
  StreamDuration totalDuration;
};

struct GPMFMetadataPrivate : public cras::HasLogger
{
  explicit GPMFMetadataPrivate(const cras::LogHelperPtr& log) : cras::HasLogger(log) {}

  std::weak_ptr<MetadataManager> manager;  //!< Metadata manager
  int priority {15};  //!< Priority of this extractor. Keep it rather low because this extractor is actually cheap.

  const AVFormatContext* avFormatContext {nullptr};  //!< Context of the open movie file.
  cras::optional<int> gpmdStreamIndex;  //!< Index of the gpmd stream in movie container.
  cras::optional<int> tmcdStreamIndex;  //!< Index of the tmcd stream in movie container.

  MovieInfo::ConstPtr info;  //!< Movie info.
  cras::optional<MovieOpenConfig> config;  //!< Movie config.

  StreamTime lastTime;  //!< Stream timestamp of the last call to processTimedMetadata().
  MetadataCache cache;  //!< Buffer for messages decoded from libav packets.

  size_t numGPMDPackets {0u};  //!< The number of processed GPMD packets.

  //! The set of timed metadata requested to be produced by this extractor. Empty means all.
  std::unordered_set<MetadataType> requestedTimedMetadata;
  //! Metadata supported by the currently loaded movie.
  std::unordered_set<MetadataType> supportedTimedMetadata;

  std::unordered_map<uint32_t, RateEstimator> rateEstimators;  //!< Rate estimators (keys are FourCC codes).
  bool hasGPS9 {false};  //!< Whether GPS9 stream is available.
  bool hasNonZeroIORI {false};  //!< Whether IORI with non-zero angle has been seen.
  cras::optional<StreamDuration> stmpOffset;  //!< Offset at which we have seen the first STMP message.

  void dumpGPMFStream(GPMF_stream* g_stream);
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

std::string fourcc2str(const uint32_t fourcc)
{
  char keyStr[5];
  memcpy(keyStr, &fourcc, 4);
  keyStr[4] = 0;
  return keyStr;
}

GPMFMetadataExtractor::GPMFMetadataExtractor(
  const cras::LogHelperPtr& log, const std::weak_ptr<MetadataManager>& manager, const MovieInfo::ConstPtr& info,
  const MovieOpenConfig& config, const AVFormatContext* avFormatContext, const int priority)
  : TimedMetadataExtractor(log), data(new GPMFMetadataPrivate(log))
{
  this->data->manager = manager;
  this->data->info = info;
  this->data->config = config;
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
      MetadataType::ROTATION,
      // MetadataType::FOCAL_LENGTH_MM,
      // MetadataType::FOCAL_LENGTH_35MM,
      // MetadataType::FOCAL_LENGTH_PX,
      // MetadataType::INTRINSIC_MATRIX,
      // MetadataType::AZIMUTH,
      MetadataType::ROLL_PITCH,
      MetadataType::GNSS_POSITION,
      MetadataType::ACCELERATION,
      MetadataType::MAGNETIC_FIELD,
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

void GPMFMetadataPrivate::dumpGPMFStream(GPMF_stream* g_stream)
{
#if (ROSCONSOLE_MIN_SEVERITY >= ROSCONSOLE_SEVERITY_DEBUG)
  const std::string loggerName = "gpmf.dump";

  // Return early if DEBUG-level printing is not enabled
  ROSCONSOLE_DEFINE_LOCATION(true, ros::console::Level::Debug, std::string(ROSCONSOLE_DEFAULT_NAME) + "." + loggerName);
  if (ROS_LIKELY(!__rosconsole_define_location__enabled))
    return;

  const auto key = GPMF_Key(g_stream);
  const auto samples = GPMF_Repeat(g_stream);
  const auto elements = GPMF_ElementsInStruct(g_stream);
  const auto type = GPMF_Type(g_stream);
  const auto buffersize = samples * elements;
  const size_t ARR_MAX_SIZE = 9;
  std::string comment;

  switch (type)
  {
  case GPMF_TYPE_FOURCC:
    {
      uint32_t fourcc;
      GPMF_FormattedData(g_stream, &fourcc, sizeof(uint32_t), 0, 1);
      comment = fourcc2str(fourcc);
      break;
    }
  case GPMF_TYPE_STRING_ASCII:
    {
      std::vector<uint8_t> buffer(buffersize);
      GPMF_FormattedData(g_stream, buffer.data(), buffersize * sizeof(uint8_t), 0, samples);
      comment = std::string(buffer.begin(), buffer.end());
      break;
    }
  case GPMF_TYPE_SIGNED_BYTE:
    {
      std::vector<int8_t> buffer(buffersize);
      GPMF_FormattedData(g_stream, buffer.data(), buffersize * sizeof(int8_t), 0, samples);
      if (buffer.size() > ARR_MAX_SIZE) {comment = cras::format("%zu: ", buffer.size()); buffer.resize(ARR_MAX_SIZE);}
      comment += cras::to_string(buffer);
      break;
    }
  case GPMF_TYPE_UNSIGNED_BYTE:
    {
      std::vector<uint8_t> buffer(buffersize);
      GPMF_FormattedData(g_stream, buffer.data(), buffersize * sizeof(uint8_t), 0, samples);
      if (buffer.size() > ARR_MAX_SIZE) {comment = cras::format("%zu: ", buffer.size()); buffer.resize(ARR_MAX_SIZE);}
      comment += cras::to_string(buffer);
      break;
    }
  case GPMF_TYPE_SIGNED_SHORT:
    {
      std::vector<int16_t> buffer(buffersize);
      GPMF_FormattedData(g_stream, buffer.data(), buffersize * sizeof(int16_t), 0, samples);
      if (buffer.size() > ARR_MAX_SIZE) {comment = cras::format("%zu: ", buffer.size()); buffer.resize(ARR_MAX_SIZE);}
      comment += cras::to_string(buffer);
      break;
    }
  case GPMF_TYPE_UNSIGNED_SHORT:
    {
      std::vector<uint16_t> buffer(buffersize);
      GPMF_FormattedData(g_stream, buffer.data(), buffersize * sizeof(uint16_t), 0, samples);
      if (buffer.size() > ARR_MAX_SIZE) {comment = cras::format("%zu: ", buffer.size()); buffer.resize(ARR_MAX_SIZE);}
      comment += cras::to_string(buffer);
      break;
    }
  case GPMF_TYPE_SIGNED_LONG:
    {
      std::vector<int32_t> buffer(buffersize);
      GPMF_FormattedData(g_stream, buffer.data(), buffersize * sizeof(int32_t), 0, samples);
      if (buffer.size() > ARR_MAX_SIZE) {comment = cras::format("%zu: ", buffer.size()); buffer.resize(ARR_MAX_SIZE);}
      comment += cras::to_string(buffer);
      break;
    }
  case GPMF_TYPE_UNSIGNED_LONG:
    {
      std::vector<uint32_t> buffer(buffersize);
      GPMF_FormattedData(g_stream, buffer.data(), buffersize * sizeof(uint32_t), 0, samples);
      if (buffer.size() > ARR_MAX_SIZE) {comment = cras::format("%zu: ", buffer.size()); buffer.resize(ARR_MAX_SIZE);}
      comment += cras::to_string(buffer);
      break;
    }
  case GPMF_TYPE_SIGNED_64BIT_INT:
    {
      std::vector<int64_t> buffer(buffersize);
      GPMF_FormattedData(g_stream, buffer.data(), buffersize * sizeof(int64_t), 0, samples);
      if (buffer.size() > ARR_MAX_SIZE) {comment = cras::format("%zu: ", buffer.size()); buffer.resize(ARR_MAX_SIZE);}
      comment += cras::to_string(buffer);
      break;
    }
  case GPMF_TYPE_UNSIGNED_64BIT_INT:
    {
      std::vector<uint64_t> buffer(buffersize);
      GPMF_FormattedData(g_stream, buffer.data(), buffersize * sizeof(uint64_t), 0, samples);
      if (buffer.size() > ARR_MAX_SIZE) {comment = cras::format("%zu: ", buffer.size()); buffer.resize(ARR_MAX_SIZE);}
      comment += cras::to_string(buffer);
      break;
    }
  case GPMF_TYPE_FLOAT:
    {
      std::vector<float> buffer(buffersize);
      GPMF_FormattedData(g_stream, buffer.data(), buffersize * sizeof(float), 0, samples);
      if (buffer.size() > ARR_MAX_SIZE) {comment = cras::format("%zu: ", buffer.size()); buffer.resize(ARR_MAX_SIZE);}
      comment += cras::to_string(buffer);
      break;
    }
  case GPMF_TYPE_DOUBLE:
    {
      std::vector<double> buffer(buffersize);
      GPMF_FormattedData(g_stream, buffer.data(), buffersize * sizeof(double), 0, samples);
      if (buffer.size() > ARR_MAX_SIZE) {comment = cras::format("%zu: ", buffer.size()); buffer.resize(ARR_MAX_SIZE);}
      comment += cras::to_string(buffer);
      break;
    }
  case GPMF_TYPE_Q15_16_FIXED_POINT:
    {
      std::vector<int32_t> buffer(buffersize);
      GPMF_FormattedData(g_stream, buffer.data(), buffersize * sizeof(int32_t), 0, samples);
      if (buffer.size() > ARR_MAX_SIZE) {comment = cras::format("%zu: ", buffer.size()); buffer.resize(ARR_MAX_SIZE);}
      std::vector<double> bufferD;
      std::transform(buffer.begin(), buffer.end(), bufferD.begin(), [](int32_t x)
      {
        return static_cast<double>(x) / 65536.0;
      });
      comment += cras::to_string(bufferD);
      break;
    }
  case GPMF_TYPE_Q31_32_FIXED_POINT:
    {
      std::vector<int64_t> buffer(buffersize);
      GPMF_FormattedData(g_stream, buffer.data(), buffersize * sizeof(int64_t), 0, samples);
      if (buffer.size() > ARR_MAX_SIZE) {comment = cras::format("%zu: ", buffer.size()); buffer.resize(ARR_MAX_SIZE);}
      std::vector<double> bufferD;
      std::transform(buffer.begin(), buffer.end(), bufferD.begin(), [](int64_t x)
      {
        const auto xx = *reinterpret_cast<uint64_t*>(&x);
        auto xxx = static_cast<double>(xx >> static_cast<uint64_t>(32));
        xxx += static_cast<double>(xx & static_cast<uint64_t>(0xffffffff)) / static_cast<double>(0x100000000);
        return xxx;
      });
      comment += cras::to_string(bufferD);
      break;
    }
  case GPMF_TYPE_UTC_DATE_TIME:
  case GPMF_TYPE_GUID:
    {
      std::vector<std::array<uint8_t, 16>> buffer(samples);
      std::vector<std::string> bufferS(samples);
      GPMF_FormattedData(g_stream, buffer.data(), buffersize * sizeof(uint8_t), 0, samples);
      std::transform(buffer.begin(), buffer.end(), bufferS.begin(), [](const std::array<uint8_t, 16>& x)
      {
        return std::string(x.begin(), x.end());
      });
      comment = cras::to_string(bufferS);
      break;
    }
  case GPMF_TYPE_COMPLEX:
    {
      std::vector<double> buffer(buffersize);
      if (samples > 0)
        GPMF_ScaledData(g_stream, buffer.data(), buffersize * sizeof(double), 0, samples, GPMF_TYPE_DOUBLE);
      if (buffer.size() > ARR_MAX_SIZE) {comment = cras::format("%zu: ", buffer.size()); buffer.resize(ARR_MAX_SIZE);}
      comment += cras::to_string(buffer);
      break;
    }
  default:
    break;
  }
  CRAS_DEBUG_NAMED(loggerName, "%s %c %u %u: %s", fourcc2str(key).c_str(), type, samples, elements, comment.c_str());
#endif
}

void GPMFMetadataPrivate::processGpmdPacket(const AVPacket* packet)
{
  this->numGPMDPackets++;

  // Figure the stream time of the packet
  const auto& timeBase_q = this->avFormatContext->streams[packet->stream_index]->time_base;
  const RationalNumber timeBase{timeBase_q.num, timeBase_q.den};
  const StreamTime packetTime(packet->pts, timeBase);
  const StreamDuration packetDuration(packet->duration, timeBase);

  CRAS_DEBUG_STREAM_NAMED("gpmf",
    "gpmd packet " << this->numGPMDPackets << " stamp " << packetTime.toRosTime() <<
    " duration " << packetDuration.toRosDuration());

  this->cache.latest.getCameraMake().emplace().emplace() = "GoPro";  // TODO hard-code to "GoPro" ?
  // this->cache.latest.getCameraModel().emplace().emplace() = "Hero 13 Black";  // TODO MINF
  // this->cache.latest.getCameraSerialNumber().emplace().emplace() = "1234";  // TODO CASN
  this->cache.latest.getLensMake().emplace().emplace() = "GoPro";  // TODO hard-code to "GoPro" ?
  // this->cache.latest.getLensModel().emplace().emplace() = "lens";  // TODO LINF???

  uint32_t* payload = reinterpret_cast<uint32_t*>(packet->data);
  uint32_t payloadsize = packet->size;
  GPMF_stream g_stream;
  uint32_t samples, elements, buffersize, key;
  std::string lastType;
  uint64_t lastTimestamp_us(0);
  StreamTime lastTimestamp = packetTime;
  std::vector<uint32_t> lastGPSF;
  std::vector<uint16_t> lastGPSP;
  if (GPMF_OK != GPMF_Init(&g_stream, payload, payloadsize))
  {
    CRAS_WARN_THROTTLE_NAMED(1.0, "gpmf", "Could not initialize GPMD packet.");
    return;
  }

  std::vector<TimedMetadata<vision_msgs::Detection2DArray>> faces;

  do
  {
    this->dumpGPMFStream(&g_stream);

    samples = GPMF_Repeat(&g_stream);
    elements = GPMF_ElementsInStruct(&g_stream);
    buffersize = samples * elements;
    key = GPMF_Key(&g_stream);

    auto& rate = this->rateEstimators[key];

    switch (key)
    {
      case STR2FOURCC("STMP"):
      {
        if (GPMF_OK != GPMF_FormattedData(&g_stream, &lastTimestamp_us, sizeof(uint64_t), 0, 1)) {
          CRAS_WARN_THROTTLE_NAMED(1.0, "gpmf", "Failed parsing STMP.");
          break;
        }
        if (!this->stmpOffset.has_value())
        {
          this->stmpOffset.emplace();
          this->stmpOffset->fromNSec(-lastTimestamp_us * 1000 + packetTime.toNSec());
        }
        lastTimestamp.fromNSec(lastTimestamp_us * 1000);
        try
        {
          lastTimestamp += *this->stmpOffset;
        }
        catch (const std::runtime_error&)
        {
          // Underflow below 0
          lastTimestamp = {};
        }
        break;
      }
      case STR2FOURCC("TYPE"):
      {
        std::vector<uint8_t> tmp_buf(buffersize);
        if (GPMF_OK != GPMF_FormattedData(&g_stream, tmp_buf.data(), buffersize * sizeof(uint8_t), 0, samples)) {
          CRAS_WARN_THROTTLE_NAMED(1.0, "gpmf", "Failed parsing TYPE.");
          break;
        }
        // Older GoPros have the string null-terminated. The double passing to std::string constructor strips that.
        lastType = std::string(std::string(tmp_buf.begin(), tmp_buf.end()).c_str());
        break;
      }
      case STR2FOURCC("ACCL"):
      {
        std::vector<double> tmp_buf(buffersize);
        if (GPMF_OK != GPMF_ScaledData(&g_stream, tmp_buf.data(), buffersize * sizeof(double),
          0, samples, GPMF_TYPE_DOUBLE)) {
          CRAS_WARN_THROTTLE_NAMED(1.0, "gpmf", "Failed parsing ACCL.");
          break;
        }

        rate.addSample(samples, packetDuration);
        const auto entryOffset = rate.entryOffset();

        for (size_t i = 0; i < samples; i++)
        {
          TimedMetadata<geometry_msgs::Vector3> msg;
          msg.stamp = lastTimestamp + entryOffset * static_cast<double>(i);
          msg.value.z = tmp_buf[i * elements];
          msg.value.x = -tmp_buf[i * elements + 2];
          msg.value.y = tmp_buf[i * elements + 1];
          // Sanity check, the values are sometimes crazy
          if (std::abs(msg.value.x) > 100 || std::abs(msg.value.y) > 100 || std::abs(msg.value.z) > 100)
            continue;
          this->cache.timed.acceleration().push_back(msg);
        }
        break;
      }

      case STR2FOURCC("GYRO"):
      {
        std::vector<double> tmp_buf(buffersize);
        if (GPMF_OK != GPMF_ScaledData(&g_stream, tmp_buf.data(), buffersize * sizeof(double),
          0, samples, GPMF_TYPE_DOUBLE)) {
          CRAS_WARN_THROTTLE_NAMED(1.0, "gpmf", "Failed parsing GYRO.");
          break;
        }

        rate.addSample(samples, packetDuration);
        const auto entryOffset = rate.entryOffset();

        for (size_t i = 0; i < samples; i++)
        {
          TimedMetadata<geometry_msgs::Vector3> msg;
          msg.stamp = lastTimestamp + entryOffset * static_cast<double>(i);
          msg.value.z = tmp_buf[i * elements];
          msg.value.x = -tmp_buf[i * elements + 2];
          msg.value.y = tmp_buf[i * elements + 1];
          // Sanity check, the values are sometimes crazy
          if (std::abs(msg.value.x) > 10 || std::abs(msg.value.y) > 10 || std::abs(msg.value.z) > 10)
            continue;
          this->cache.timed.angularVelocity().push_back(msg);
        }
        break;
      }

      case STR2FOURCC("MAGN"):
      {
        std::vector<double> tmp_buf(buffersize);
        if (GPMF_OK != GPMF_ScaledData(&g_stream, tmp_buf.data(), buffersize * sizeof(double),
          0, samples, GPMF_TYPE_DOUBLE)) {
          CRAS_WARN_THROTTLE_NAMED(1.0, "gpmf", "Failed parsing MAGN.");
          break;
        }

        rate.addSample(samples, packetDuration);
        const auto entryOffset = rate.entryOffset();

        for (size_t i = 0; i < samples; i++)
        {
          TimedMetadata<sensor_msgs::MagneticField> msg;
          msg.stamp = lastTimestamp + entryOffset * static_cast<double>(i);
          msg.value.magnetic_field.z = tmp_buf[i * elements] * 1e-6;
          msg.value.magnetic_field.x = -tmp_buf[i * elements + 2] * 1e-6;
          msg.value.magnetic_field.y = tmp_buf[i * elements + 1] * 1e-6;
          this->cache.timed.magneticField().push_back(msg);
        }
        break;
      }

      case STR2FOURCC("GPS9"):
      {
        this->hasGPS9 = true;
        std::vector<double> tmp_buf(buffersize);

        if (GPMF_OK != GPMF_ScaledData(&g_stream, tmp_buf.data(), buffersize * sizeof(double),
          0, samples, GPMF_TYPE_DOUBLE)) {
          CRAS_WARN_THROTTLE_NAMED(1.0, "gpmf", "Failed parsing GPS9.");
          break;
        }

        rate.addSample(samples, packetDuration);
        const auto entryOffset = rate.entryOffset();

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
          if (fix_type == 0)
            continue;

          cras::optional<sensor_msgs::NavSatFix> navSatFix;
          navSatFix.emplace();
          navSatFix->header.frame_id = this->config->frameId();
          navSatFix->latitude = latitude;
          navSatFix->longitude = longitude;
          navSatFix->altitude = altitude;
          navSatFix->status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
          navSatFix->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

          cras::optional<gps_common::GPSFix> gpsFix;
          gpsFix.emplace();
          gpsFix->header.frame_id = this->config->frameId();
          gpsFix->status.header.frame_id = this->config->frameId();
          gpsFix->latitude = latitude;
          gpsFix->longitude = longitude;
          gpsFix->altitude = altitude;
          gpsFix->speed = speed_2d;
          gpsFix->gdop = dop;
          gpsFix->status.status = gps_common::GPSStatus::STATUS_FIX;
          gpsFix->status.position_source = gps_common::GPSStatus::SOURCE_GPS;

          const auto time = lastTimestamp + entryOffset * static_cast<double>(i);
          TimedMetadata<GNSSFixAndDetail> msg = {time, std::make_pair(navSatFix, gpsFix)};
          this->cache.timed.gnssPosition().push_back(msg);
        }
        break;
      }
      case STR2FOURCC("GPSP"):
      {
        if (this->hasGPS9)
          break;

        lastGPSP.resize(buffersize);
        if (GPMF_OK != GPMF_ScaledData(&g_stream, lastGPSP.data(), buffersize * sizeof(uint16_t),
          0, samples, GPMF_TYPE_UNSIGNED_SHORT)) {
          CRAS_WARN_THROTTLE_NAMED(1.0, "gpmf", "Failed parsing GPSP.");
          lastGPSP.clear();
          break;
        }
        break;
      }
      case STR2FOURCC("GPSF"):
      {
        if (this->hasGPS9)
          break;

        lastGPSF.resize(buffersize);
        if (GPMF_OK != GPMF_ScaledData(&g_stream, lastGPSF.data(), buffersize * sizeof(uint32_t),
          0, samples, GPMF_TYPE_UNSIGNED_LONG)) {
          CRAS_WARN_THROTTLE_NAMED(1.0, "gpmf", "Failed parsing GPSF.");
          lastGPSF.clear();
          break;
        }
        break;
      }
      case STR2FOURCC("GPS5"):
      {
        if (this->hasGPS9)
          break;

        std::vector<double> tmp_buf(buffersize);

        if (GPMF_OK != GPMF_ScaledData(&g_stream, tmp_buf.data(), buffersize * sizeof(double),
          0, samples, GPMF_TYPE_DOUBLE)) {
          CRAS_WARN_THROTTLE_NAMED(1.0, "gpmf", "Failed parsing GPS5.");
          break;
        }

        rate.addSample(samples, packetDuration);
        const auto entryOffset = rate.entryOffset();

        for (size_t i = 0; i < samples; i++) {
          double latitude = tmp_buf[i * elements];
          double longitude = tmp_buf[i * elements + 1];
          double altitude = tmp_buf[i * elements + 2];
          double speed_2d = tmp_buf[i * elements + 3];
          double speed_3d = tmp_buf[i * elements + 4];
          double dop = 100;

          uint64_t fix_type = 1;

          if (lastGPSF.size() >= i + 1)
            fix_type = lastGPSF[i];

          if (fix_type == 0)
            continue;

          if (lastGPSP.size() >= i + 1)
            dop = lastGPSP[i] / 100.0;

          cras::optional<sensor_msgs::NavSatFix> navSatFix;
          navSatFix.emplace();
          navSatFix->header.frame_id = this->config->frameId();
          navSatFix->latitude = latitude;
          navSatFix->longitude = longitude;
          navSatFix->altitude = altitude;
          navSatFix->status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
          navSatFix->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

          cras::optional<gps_common::GPSFix> gpsFix;
          gpsFix.emplace();
          gpsFix->header.frame_id = this->config->frameId();
          gpsFix->status.header.frame_id = this->config->frameId();
          gpsFix->latitude = latitude;
          gpsFix->longitude = longitude;
          gpsFix->altitude = altitude;
          gpsFix->speed = speed_2d;
          gpsFix->gdop = dop;
          gpsFix->status.status = gps_common::GPSStatus::STATUS_FIX;
          gpsFix->status.position_source = gps_common::GPSStatus::SOURCE_GPS;

          const auto time = lastTimestamp + entryOffset * static_cast<double>(i);
          TimedMetadata<GNSSFixAndDetail> msg = {time, std::make_pair(navSatFix, gpsFix)};
          this->cache.timed.gnssPosition().push_back(msg);
        }
        break;
      }
      case STR2FOURCC("GRAV"):
      {
        std::vector<double> tmp_buf(buffersize);
        if (GPMF_OK != GPMF_ScaledData(&g_stream, tmp_buf.data(), buffersize * sizeof(double),
          0, samples, GPMF_TYPE_DOUBLE)) {
          CRAS_WARN_THROTTLE_NAMED(1.0, "gpmf", "Failed parsing GRAV.");
          break;
        }

        rate.addSample(samples, packetDuration);
        const auto entryOffset = rate.entryOffset();

        for (size_t i = 0; i < samples; i++)
        {
          // https://github.com/gopro/gpmf-parser/issues/170#issue-1458191867
          // Data are stored as Xzy + we convert them from GoPro IMU frame to ROS frame (yXZ) -> ZXy
          // For some reason, we also have to negate gy, don't know why.
          double gx =  tmp_buf[i * elements + 2];
          double gy = -tmp_buf[i * elements + 0];
          double gz = -tmp_buf[i * elements + 1];

          // Some cameras fill GRAV with zeros
          if (std::abs(gx) + std::abs(gy) + std::abs(gz) < 0.1)
            continue;

          // Compute the rotation between measured gravity vector and steady-state gravity vector
          Eigen::Vector3d v1(gx, gy, gz);
          Eigen::Vector3d v2(0, 0, -1);

          const auto q = Eigen::Quaterniond::FromTwoVectors(v1, v2);
          geometry_msgs::Quaternion quat;
          quat.x = q.x(); quat.y = q.y(); quat.z = q.z(); quat.w = q.w();

          TimedMetadata<std::pair<double, double>> msg;
          msg.stamp = lastTimestamp + entryOffset * static_cast<double>(i);

          double yaw;
          cras::getRPY(quat, msg.value.first, msg.value.second, yaw);

          this->cache.timed.rollPitch().push_back(msg);
        }
        break;
      }
      case STR2FOURCC("FACE"):
      {
        std::vector<double> tmp_buf(buffersize);
        if (samples > 0)
        {
          if (GPMF_OK != GPMF_ScaledData(&g_stream, tmp_buf.data(), buffersize * sizeof(double),
            0, samples, GPMF_TYPE_DOUBLE))
          {
            CRAS_WARN_THROTTLE_NAMED(1.0, "gpmf", "Failed parsing FACE.");
            break;
          }
        }

        int version = 0;
        if (lastType == "BBSSSSSBB")
          version = (samples > 0) ? static_cast<int>(tmp_buf[0]) : 4;
        else if (lastType == "Lffffff")
          version = 3;
        else if (lastType == "Lffffffffffffffffffffff")
          version = 2;
        else if (lastType == "Lffff")
          version = 1;

        cras::optional<size_t> x_idx, y_idx, w_idx, h_idx, conf_idx, id_idx;
        switch (version)
        {
        case 1:
        case 2:
          x_idx = 1; y_idx = 2; w_idx = 3; h_idx = 4; id_idx = 0;
          break;
        case 3:
          x_idx = 1; y_idx = 2; w_idx = 3; h_idx = 4; conf_idx = 5; id_idx = 0;
          break;
        case 4:
          x_idx = 3; y_idx = 4; w_idx = 5; h_idx = 6; conf_idx = 1; id_idx = 2;
          break;
        default:
          CRAS_ERROR_ONCE_NAMED("gpmf", "Unknown FACE entry structure. Faces will not be parsed.");
        }

        TimedMetadata<vision_msgs::Detection2DArray> msg;
        // We only put the base stamp here. The entry offset stamps will be added after finishing this loop, when we
        // know how many faces per payload there are.
        msg.stamp = lastTimestamp;
        msg.value.header.frame_id = this->config->opticalFrameId();

        for (size_t i = 0; x_idx.has_value() && i < samples; i++)
        {
          const auto entry = &tmp_buf[i * elements];
          double x = entry[*x_idx] * this->info->width();
          double y = entry[*y_idx] * this->info->height();
          double w = entry[*w_idx] * this->info->width();
          double h = entry[*h_idx] * this->info->height();

          double center_x = x + w / 2.0f;
          double center_y = y + h / 2.0f;

          msg.value.detections.emplace_back();
          msg.value.detections.back().header.frame_id = this->config->opticalFrameId();
          msg.value.detections.back().bbox.center.x = center_x;
          msg.value.detections.back().bbox.center.y = center_y;
          msg.value.detections.back().bbox.size_x = w;
          msg.value.detections.back().bbox.size_y = h;

          if (conf_idx.has_value() || id_idx.has_value())
          {
            msg.value.detections.back().results.emplace_back();
            msg.value.detections.back().results.back().pose.pose.orientation.w = 1;
            if (id_idx.has_value())
              msg.value.detections.back().results.back().id = static_cast<int64_t>(entry[*id_idx]);
            if (conf_idx.has_value())
              msg.value.detections.back().results.back().score = entry[*conf_idx] / 100.0;
          }
        }
        faces.push_back(msg);
        break;
      }
      case STR2FOURCC("IORI"):
      {
        std::vector<double> tmp_buf(buffersize);
        if (GPMF_OK != GPMF_ScaledData(&g_stream, tmp_buf.data(), buffersize * sizeof(double),
          0, samples, GPMF_TYPE_DOUBLE)) {
          CRAS_WARN_THROTTLE_NAMED(1.0, "gpmf", "Failed parsing IORI.");
          break;
        }

        rate.addSample(samples, packetDuration);
        const auto entryOffset = rate.entryOffset();

        for (size_t i = 0; i < samples; i++)
        {
          double w = tmp_buf[i * elements];
          double x = tmp_buf[i * elements + 1];
          double y = tmp_buf[i * elements + 2];
          double z = tmp_buf[i * elements + 3];
          tf2::Quaternion q(x, y, z, w);
          double roll, pitch, yaw;
          cras::getRPY(q, roll, pitch, yaw);
          // According to docs, pitch is the angle we are interested in
          auto pitchDeg = pitch * 2 * M_PI / 180;
          if (pitchDeg < 0)
            pitchDeg += 360;

          TimedMetadata<int> msg;
          msg.stamp = lastTimestamp + entryOffset * static_cast<double>(i);
          if (std::abs(pitchDeg - 0) < 20 || std::abs(pitchDeg - 360) < 20)
            msg.value = 0;
          else if (std::abs(pitchDeg - 90) < 20)
            msg.value = 90;
          else if (std::abs(pitchDeg - 180) < 20)
            msg.value = 180;
          else if (std::abs(pitchDeg - 270) < 20)
            msg.value = 270;
          else
            continue;

          // I've never seen a video with non-unit IORI. But ffmpeg can extract video rotation. So we keep the
          // ffmpeg-extracted rotation until there is a non-zero IORI, which I hope would then be correct.
          if (!this->hasNonZeroIORI && msg.value == 0)
            continue;
          if (msg.value != 0)
            this->hasNonZeroIORI;

          this->cache.timed.rotation().push_back(msg);
        }
        break;
      }
      default:  // if you don't know the Key you can skip to the next
        break;
    }
  } while (GPMF_OK == GPMF_Next(&g_stream, GPMF_RECURSE_LEVELS));

  // We don't really know the rate of faces, so we estimate it here.
  if (!faces.empty())
  {
    const auto numMsgs = faces.size();
    this->rateEstimators[STR2FOURCC("FACE")].addSample(numMsgs, packetDuration);
    const auto entryOffset = this->rateEstimators[STR2FOURCC("FACE")].entryOffset();
    for (size_t i = 0; i < numMsgs; i++)
    {
      auto& msg = faces[i];
      if (msg.value.detections.empty())
        continue;

      msg.stamp += entryOffset * static_cast<double>(i);
      this->cache.timed.faces().push_back(msg);
    }
  }
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
    params.log, params.manager, params.info, params.config, params.avFormatContext, priority);
}

}

PLUGINLIB_EXPORT_CLASS(movie_publisher::GPMFMetadataExtractorPlugin, movie_publisher::MetadataExtractorPlugin)
