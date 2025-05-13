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
#include <pluginlib/class_list_macros.h>

#include "gpmf_parser/GPMF_parser.h"
#include "gpmf_parser/GPMF_utils.h"

namespace movie_publisher
{

/**
 * \brief A buffer that holds data read from libav packets before they are picked up by processTimedMetadata().
 */
struct MetadataBuffer
{
  std::queue<TimedMetadata<double>> cropFactor;
  std::queue<TimedMetadata<std::pair<double, double>>> sensorSizeMM;
  std::queue<TimedMetadata<std::pair<DistortionType, Distortion>>> distortion;
  std::queue<TimedMetadata<int>> rotation;
  std::queue<TimedMetadata<double>> focalLengthMM;
  std::queue<TimedMetadata<double>> focalLength35MM;
  std::queue<TimedMetadata<double>> focalLengthPx;
  std::queue<TimedMetadata<IntrinsicMatrix>> intrinsicMatrix;
  std::queue<TimedMetadata<compass_msgs::Azimuth>> azimuth;
  std::queue<TimedMetadata<std::pair<double, double>>> rollPitch;
  std::queue<TimedMetadata<GNSSFixAndDetail>> fix;
  std::queue<TimedMetadata<geometry_msgs::Vector3>> acceleration;
  std::queue<TimedMetadata<sensor_msgs::MagneticField>> magneticField;
  std::queue<TimedMetadata<geometry_msgs::Vector3>> angularVelocity;
  std::queue<TimedMetadata<vision_msgs::Detection2DArray>> faces;

  /**
   * \brief Remove all data from the buffer.
   */
  void clear()
  {
    this->cropFactor = {};
    this->sensorSizeMM = {};
    this->distortion = {};
    this->rotation = {};
    this->focalLengthMM = {};
    this->focalLength35MM = {};
    this->focalLengthPx = {};
    this->intrinsicMatrix = {};
    this->azimuth = {};
    this->rollPitch = {};
    this->fix = {};
    this->acceleration = {};
    this->magneticField = {};
    this->angularVelocity = {};
    this->faces = {};
  }
};

/**
 * \brief Cache that holds the last processed metadata value so that timed metadata can be used through the static
 *        metadata API.
 */
struct LastMetadataCache
{
  cras::optional<TimedMetadata<double>> cropFactor;
  cras::optional<TimedMetadata<std::pair<double, double>>> sensorSizeMM;
  cras::optional<TimedMetadata<std::pair<DistortionType, Distortion>>> distortion;
  cras::optional<TimedMetadata<int>> rotation;
  cras::optional<TimedMetadata<double>> focalLengthMM;
  cras::optional<TimedMetadata<double>> focalLength35MM;
  cras::optional<TimedMetadata<double>> focalLengthPx;
  cras::optional<TimedMetadata<IntrinsicMatrix>> intrinsicMatrix;
  cras::optional<TimedMetadata<compass_msgs::Azimuth>> azimuth;
  cras::optional<TimedMetadata<std::pair<double, double>>> rollPitch;
  cras::optional<TimedMetadata<GNSSFixAndDetail>> fix;
  cras::optional<TimedMetadata<geometry_msgs::Vector3>> acceleration;
  cras::optional<TimedMetadata<sensor_msgs::MagneticField>> magneticField;
  cras::optional<TimedMetadata<geometry_msgs::Vector3>> angularVelocity;
  cras::optional<TimedMetadata<vision_msgs::Detection2DArray>> faces;

  cras::optional<std::string> cameraSerialNumber;
  cras::optional<std::string> cameraMake;
  cras::optional<std::string> cameraModel;
  cras::optional<std::string> lensMake;
  cras::optional<std::string> lensModel;
  cras::optional<ros::Time> creationTime;

  /**
   * \brief Remove all data from the cache.
   */
  void clear()
  {
    this->cropFactor.reset();
    this->sensorSizeMM.reset();
    this->distortion.reset();
    this->rotation.reset();
    this->focalLengthMM.reset();
    this->focalLength35MM.reset();
    this->focalLengthPx.reset();
    this->intrinsicMatrix.reset();
    this->azimuth.reset();
    this->rollPitch.reset();
    this->fix.reset();
    this->acceleration.reset();
    this->magneticField.reset();
    this->angularVelocity.reset();
    this->faces.reset();

    this->cameraSerialNumber.reset();
    this->cameraMake.reset();
    this->cameraModel.reset();
    this->lensMake.reset();
    this->lensModel.reset();
    this->creationTime.reset();
  }
};

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
  MetadataBuffer buffer;  //!< Buffer for messages decoded from libav packets.
  LastMetadataCache lastMetadata;  //!< Cache for keeping the last decoded message of each type.

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
      MetadataType::CROP_FACTOR,
      MetadataType::SENSOR_SIZE_MM,
      MetadataType::DISTORTION,
      MetadataType::ROTATION,
      MetadataType::FOCAL_LENGTH_MM,
      MetadataType::FOCAL_LENGTH_35MM,
      MetadataType::FOCAL_LENGTH_PX,
      MetadataType::INTRINSIC_MATRIX,
      MetadataType::AZIMUTH,
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
  if (this->data->lastMetadata.cropFactor.has_value())
    return this->data->lastMetadata.cropFactor->value;
  return cras::nullopt;
}

cras::optional<std::pair<double, double>> GPMFMetadataExtractor::getSensorSizeMM()
{
  if (this->data->lastMetadata.sensorSizeMM.has_value())
    return this->data->lastMetadata.sensorSizeMM->value;
  return cras::nullopt;
}

cras::optional<double> GPMFMetadataExtractor::getFocalLengthMM()
{
  if (this->data->lastMetadata.focalLengthMM.has_value())
    return this->data->lastMetadata.focalLengthMM->value;
  return cras::nullopt;
}

cras::optional<std::pair<DistortionType, Distortion>> GPMFMetadataExtractor::getDistortion()
{
  if (this->data->lastMetadata.distortion.has_value())
    return this->data->lastMetadata.distortion->value;
  return cras::nullopt;
}

cras::optional<std::string> GPMFMetadataExtractor::getCameraSerialNumber()
{
  if (this->data->lastMetadata.cameraSerialNumber.has_value())
    return this->data->lastMetadata.cameraSerialNumber;
  return cras::nullopt;
}

cras::optional<std::string> GPMFMetadataExtractor::getCameraMake()
{
  if (this->data->lastMetadata.cameraMake.has_value())
    return this->data->lastMetadata.cameraMake;
  return cras::nullopt;
}

cras::optional<std::string> GPMFMetadataExtractor::getCameraModel()
{
  if (this->data->lastMetadata.cameraModel.has_value())
    return this->data->lastMetadata.cameraModel;
  return cras::nullopt;
}

cras::optional<std::string> GPMFMetadataExtractor::getLensMake()
{
  if (this->data->lastMetadata.lensMake.has_value())
    return this->data->lastMetadata.lensMake;
  return cras::nullopt;
}

cras::optional<std::string> GPMFMetadataExtractor::getLensModel()
{
  if (this->data->lastMetadata.lensModel.has_value())
    return this->data->lastMetadata.lensModel;
  return cras::nullopt;
}

cras::optional<int> GPMFMetadataExtractor::getRotation()
{
  if (this->data->lastMetadata.rotation.has_value())
    return this->data->lastMetadata.rotation->value;
  return cras::nullopt;
}

cras::optional<ros::Time>GPMFMetadataExtractor::getCreationTime()
{
  if (this->data->lastMetadata.creationTime.has_value())
    return this->data->lastMetadata.creationTime;
  return cras::nullopt;
}

cras::optional<double> GPMFMetadataExtractor::getFocalLength35MM()
{
  if (this->data->lastMetadata.focalLength35MM.has_value())
    return this->data->lastMetadata.focalLength35MM->value;
  return cras::nullopt;
}

cras::optional<double> GPMFMetadataExtractor::getFocalLengthPx()
{
  if (this->data->lastMetadata.focalLengthPx.has_value())
    return this->data->lastMetadata.focalLengthPx->value;
  return cras::nullopt;
}

cras::optional<IntrinsicMatrix> GPMFMetadataExtractor::getIntrinsicMatrix()
{
  if (this->data->lastMetadata.intrinsicMatrix.has_value())
    return this->data->lastMetadata.intrinsicMatrix->value;
  return cras::nullopt;
}

GNSSFixAndDetail GPMFMetadataExtractor::getGNSSPosition()
{
  if (this->data->lastMetadata.fix.has_value())
    return this->data->lastMetadata.fix->value;
  return {cras::nullopt, cras::nullopt};
}

cras::optional<compass_msgs::Azimuth> GPMFMetadataExtractor::getAzimuth()
{
  if (this->data->lastMetadata.azimuth.has_value())
    return this->data->lastMetadata.azimuth->value;
  return cras::nullopt;
}

cras::optional<std::pair<double, double>> GPMFMetadataExtractor::getRollPitch()
{
  if (this->data->lastMetadata.rollPitch.has_value())
    return this->data->lastMetadata.rollPitch->value;
  return cras::nullopt;
}

cras::optional<geometry_msgs::Vector3> GPMFMetadataExtractor::getAcceleration()
{
  if (this->data->lastMetadata.acceleration.has_value())
    return this->data->lastMetadata.acceleration->value;
  return cras::nullopt;
}

cras::optional<sensor_msgs::MagneticField> GPMFMetadataExtractor::getMagneticField()
{
  if (this->data->lastMetadata.magneticField.has_value())
    return this->data->lastMetadata.magneticField->value;
  return cras::nullopt;
}

cras::optional<geometry_msgs::Vector3> GPMFMetadataExtractor::getAngularVelocity()
{
  if (this->data->lastMetadata.angularVelocity.has_value())
    return this->data->lastMetadata.angularVelocity->value;
  return cras::nullopt;
}

cras::optional<vision_msgs::Detection2DArray> GPMFMetadataExtractor::getFaces()
{
  if (this->data->lastMetadata.faces.has_value())
    return this->data->lastMetadata.faces->value;
  return cras::nullopt;
}

void GPMFMetadataExtractor::prepareTimedMetadata(const std::unordered_set<MetadataType>& types)
{
  TimedMetadataExtractor::prepareTimedMetadata(types);
  // TODO some stuff that needs to be done before starting to extract timed metadata from the video
  //      Called with a list of metadata that should be extracted (if empty, extract all).
  //      This can e.g. read the first second of the video to get all the static metadata like camera model etc.
  this->data->requestedTimedMetadata = types;
  CRAS_INFO_NAMED("gpmf", "prepareTimedMetadata");
}

void GPMFMetadataExtractor::processPacket(const AVPacket* packet)
{
  static size_t num = 0;
  CRAS_INFO_STREAM_NAMED("gpmf", "packet " << ++num);

  if (this->data->gpmdStreamIndex.has_value() && packet->stream_index == *this->data->gpmdStreamIndex)
    this->data->processGpmdPacket(packet);
  if (this->data->tmcdStreamIndex.has_value() && packet->stream_index == *this->data->tmcdStreamIndex)
    this->data->processTmcdPacket(packet);
}

void GPMFMetadataPrivate::processGpmdPacket(const AVPacket* packet)
{
  static size_t num = 0;
  ++num;

  // Figure the stream time of the packet
  const auto& timeBase = this->avFormatContext->streams[packet->stream_index]->time_base;
  StreamTime packetTime(packet->pts, RationalNumber{timeBase.num, timeBase.den});

  CRAS_INFO_STREAM_NAMED("gpmf", "gpmd packet " << num << " stamp " << packetTime.toRosTime());

  // TODO read metadata from packet and store them in this->buffer until processTimedMetadata picks them up.
  //      static metadata should go directly to this->lastMetadata
  // pointer to mp4 container metadata

  this->lastMetadata.cameraMake = "GoPro";  // TODO hard-code to "GoPro" ?
  this->lastMetadata.cameraModel = "Hero 13 Black";  // TODO MINF
  this->lastMetadata.cameraSerialNumber = "1234";  // TODO CASN
  this->lastMetadata.lensMake = "GoPro";  // TODO hard-code to "GoPro" ?
  this->lastMetadata.lensModel = "lens";  // TODO LINF???

  // Initialize packet in parser
  const auto time = packetTime + StreamDuration(0.01);

  uint32_t* payload = reinterpret_cast<uint32_t*>(packet->data);
  uint32_t payloadsize = packet->size;
  GPMF_stream g_stream;
  uint32_t samples, elements, buffersize, faceLoadCount(0);
  uint64_t lastTimestamp_us(0);
  if (GPMF_OK != GPMF_Init(&g_stream, payload, payloadsize))
  {
    // TODO THROW CRAS ERROR UNABLE TO INITIALIZE PACKET
  }

  do
  {
    switch (GPMF_Key(&g_stream))
    {
      case STR2FOURCC("STMP"):
      {
        if (GPMF_OK != GPMF_FormattedData(&g_stream, &lastTimestamp_us, sizeof(uint64_t), 0, 1)) {
          // TODO THROW CRAS ERROR SCALING GPMD FAILED
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
          // TODO THROW CRAS ERROR SCALING GPMD FAILED
        }

        // calculate offset between accl entries based on the frequency, in microseconds
        const uint64_t entry_offset_us = 1'000'000 / 200;
        for (size_t i = 0; i < samples; i++)
        {
          TimedMetadata<geometry_msgs::Vector3> msg;
          msg.stamp = StreamTime((lastTimestamp_us + faceLoadCount * entry_offset_us) / 10e6);
          msg.value.z = tmp_buf[i];
          msg.value.x = tmp_buf[i + 1];
          msg.value.y = tmp_buf[i + 2];
          this->buffer.acceleration.emplace(msg);
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
          // TODO THROW CRAS ERROR SCALING GPMD FAILED
        }

        // calculate offset between accl entries based on the frequency, in microseconds
        const uint64_t entry_offset_us = 1'000'000 / 200;
        for (size_t i = 0; i < samples; i++)
        {
          TimedMetadata<geometry_msgs::Vector3> msg;
          msg.stamp = StreamTime((lastTimestamp_us + faceLoadCount * entry_offset_us) / 10e6);
          msg.value.z = tmp_buf[i];
          msg.value.x = tmp_buf[i + 1];
          msg.value.y = tmp_buf[i + 2];
          this->buffer.angularVelocity.emplace(msg);
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
          // TODO THROW CRAS ERROR SCALING GPMD FAILED
        }
        break;

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

          StreamTime time = StreamTime((lastTimestamp_us + faceLoadCount * entry_offset_us) / 10e6);
          TimedMetadata<std::pair<cras::optional<sensor_msgs::NavSatFix>, cras::optional<gps_common::GPSFix>>> msg = {
            time, std::make_pair(navSatFix, gpsFix)
          };
          this->buffer.fix.emplace(msg);
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
          // TODO THROW CRAS ERROR SCALING GPMD FAILED
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
          msg.stamp = StreamTime((lastTimestamp_us + faceLoadCount * entry_offset_us) / 10e6);
          // in radians
          msg.value.first = atan2(gy, gz);  // roll
          msg.value.second = atan2(-gx, sqrt(gy * gy + gz * gz));  // pitch
          this->buffer.rollPitch.emplace(msg);
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
        {
          faceLoadCount++;
          break;
        }

        if (GPMF_OK != GPMF_ScaledData(&g_stream, tmp_buf.data(), buffersize * sizeof(double),
          0, samples, GPMF_TYPE_DOUBLE))
        {
          // TODO THROW CRAS ERROR SCALING GPMD FAILED
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
          msg.stamp = StreamTime((lastTimestamp_us + faceLoadCount * entry_offset_us) / 10e6);
          // msg.value.header.stamp whats this ?
          msg.value.detections.emplace_back();
          msg.value.detections.back().bbox.center.x = center_x;
          msg.value.detections.back().bbox.center.y = center_y;
          msg.value.detections.back().bbox.size_x = w;
          msg.value.detections.back().bbox.size_y = h;

          this->buffer.faces.emplace(msg);
        }
        faceLoadCount++;
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


  // TODO This is a fake loop to generate some data
  for (size_t i = 0; i < 10; i++)
  {
    const auto time = packetTime + StreamDuration(0.01) * i;
    const auto& req = this->requestedTimedMetadata;

    if (req.empty() || req.find(MetadataType::CROP_FACTOR) != req.end())
    {
      // TODO not sure where to get it; in the worst case, use a hard-coded table of GoPro models
      TimedMetadata<double> msg;
      msg.stamp = time;
      msg.value = time.toRosTime().toSec();
      this->buffer.cropFactor.emplace(msg);
    }

    if (req.empty() || req.find(MetadataType::SENSOR_SIZE_MM) != req.end())
    {
      // TODO not sure where to get it; in the worst case, hard-code a table from https://en.wikipedia.org/wiki/GoPro#HERO13
      TimedMetadata<std::pair<double, double>> msg;
      msg.stamp = time;
      msg.value.first = time.toRosTime().sec;
      msg.value.second = time.toRosTime().nsec;
      this->buffer.sensorSizeMM.emplace(msg);
    }

    if (req.empty() || req.find(MetadataType::DISTORTION) != req.end())
    {
      // TODO https://github.com/gopro/gpmf-parser?tab=readme-ov-file#dvid-fovl-large-fov---lens-distortion
      //      If VFOV is Linear, distortion is already corrected so this function should return all zeros
      //      SuperView and HyperView use a nonlinear horizontal stretching algorithm (MXCF, MYCF, MAPX, MAPY)
      //      (https://abekislevitz.com/43-gopro-footage-explained/), so there's no way to make them fully working with
      //      the current framework.
      //      GoPro with Wide FOV has fisheye lens, so use sensor_msgs::distortion_models::EQUIDISTANT
      TimedMetadata<std::pair<DistortionType, Distortion>> msg;
      msg.stamp = time;
      msg.value.first = "test";
      msg.value.second = {static_cast<double>(time.sec), static_cast<double>(time.nsec)};
      this->buffer.distortion.emplace(msg);
    }

    if (req.empty() || req.find(MetadataType::ROTATION) != req.end())
    {
      // TODO OREN, maybe IORI?
      TimedMetadata<int> msg;
      msg.stamp = time;
      msg.value = time.sec;
      this->buffer.rotation.emplace(msg);
    }

    if (req.empty() || req.find(MetadataType::FOCAL_LENGTH_MM) != req.end())
    {
      // TODO not sure where to get it, maybe a hard-coded table based on aspect ratio, lens mode and a static table
      //      per GoPro model from https://www.google.com/search?q=gopro+Digital+Lenses+FOV+Information ?
      //      Also consider DZOM+DZST, EISE+EISA+HCTL, ZFOV+VFOV, ARUW+ARWA
      TimedMetadata<double> msg;
      msg.stamp = time;
      msg.value = time.toRosTime().toSec();
      this->buffer.focalLengthMM.emplace(msg);
    }

    if (req.empty() || req.find(MetadataType::FOCAL_LENGTH_35MM) != req.end())
    {
      // TODO not sure where to get it; if it isn't anywhere, leave it out and let manager compute it from crop factor
      //      and focal length in mm
      TimedMetadata<double> msg;
      msg.stamp = time;
      msg.value = time.toRosTime().toSec();
      this->buffer.focalLength35MM.emplace(msg);
    }

    if (req.empty() || req.find(MetadataType::FOCAL_LENGTH_PX) != req.end())
    {
      // TODO not sure where to get it; if it isn't anywhere, leave it out and let manager compute it from sensor size
      //      and focal length in mm
      TimedMetadata<double> msg;
      msg.stamp = time;
      msg.value = time.toRosTime().toSec();
      this->buffer.focalLengthPx.emplace(msg);
    }

    if (req.empty() || req.find(MetadataType::INTRINSIC_MATRIX) != req.end())
    {
      // TODO if neither calibration matrix K nor projection matrix P are defined, leave this out and let manager
      //      compute it from pixel focal length and image dimensions
      TimedMetadata<IntrinsicMatrix> msg;
      msg.stamp = time;
      msg.value = {1.0 * time.sec, 1.0 * time.nsec, 1.0 * num, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      this->buffer.intrinsicMatrix.emplace(msg);
    }

    if (req.empty() || req.find(MetadataType::AZIMUTH) != req.end())
    {
      // TODO compute from MAGN
      TimedMetadata<compass_msgs::Azimuth> msg;
      msg.stamp = time;
      msg.value.azimuth = time.toRosTime().toSec();
      this->buffer.azimuth.emplace(msg);
    }

    if (req.empty() || req.find(MetadataType::MAGNETIC_FIELD) != req.end())
    {
      // TODO
      TimedMetadata<sensor_msgs::MagneticField> msg;
      msg.stamp = time;
      msg.value.magnetic_field.x = time.sec;
      msg.value.magnetic_field.y = time.nsec;
      msg.value.magnetic_field.z = num;
      this->buffer.magneticField.emplace(msg);
    }
  }
}

void GPMFMetadataPrivate::processTmcdPacket(const AVPacket* packet)
{
  static size_t num = 0;
  ++num;

  const auto& timeBase = this->avFormatContext->streams[packet->stream_index]->time_base;
  StreamTime packetTime(packet->pts, RationalNumber{timeBase.num, timeBase.den});

  CRAS_INFO_STREAM_NAMED("gpmf", "tmcd packet " << num << " stamp " << packetTime.toRosTime());

  this->lastMetadata.creationTime = ros::Time::now();  // TODO read from the tmcd content?
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
size_t proc(std::queue<T>& bufferedData, cras::optional<T>& lastData,
  void(TimedMetadataListener::*processFn)(const T&),
  const std::vector<std::shared_ptr<TimedMetadataListener>>& listeners, const StreamTime& maxTime)
{
  size_t numProcessed {0u};
  for (; !bufferedData.empty(); bufferedData.pop())
  {
    const auto& msg = bufferedData.front();
    if (msg.stamp > maxTime)
      break;
    lastData = msg;
    numProcessed++;
    for (const auto& listener : listeners)
      (listener.get()->*processFn)(msg);
  }
  return numProcessed;
}

size_t GPMFMetadataExtractor::processTimedMetadata(
  const MetadataType type, const StreamTime& maxTime, const bool requireOptional)
{
  CRAS_INFO_NAMED("gpmf", "process %s", cras::to_string(maxTime).c_str());

  if (this->data->requestedTimedMetadata.count(type) == 0)
    return 0;

  // Shorthands to keep the following code short.
  auto& buffer = this->data->buffer;
  auto& last = this->data->lastMetadata;
  auto& l = this->listeners;
  const auto& t = maxTime;
  size_t n {0u};

  switch (type)
  {
  case MetadataType::CROP_FACTOR:
    n += proc(buffer.cropFactor, last.cropFactor, &TimedMetadataListener::processCropFactor, l, t);
    break;
  case MetadataType::SENSOR_SIZE_MM:
    n += proc(buffer.sensorSizeMM, last.sensorSizeMM, &TimedMetadataListener::processSensorSizeMM, l, t);
    break;
  case MetadataType::DISTORTION:
    n += proc(buffer.distortion, last.distortion, &TimedMetadataListener::processDistortion, l, t);
    break;
  case MetadataType::ROTATION:
    n += proc(buffer.rotation, last.rotation, &TimedMetadataListener::processRotation, l, t);
    break;
  case MetadataType::FOCAL_LENGTH_MM:
    n += proc(buffer.focalLengthMM, last.focalLengthMM, &TimedMetadataListener::processFocalLengthMM, l, t);
    break;
  case MetadataType::FOCAL_LENGTH_35MM:
    n += proc(buffer.focalLength35MM, last.focalLength35MM, &TimedMetadataListener::processFocalLength35MM, l, t);
    break;
  case MetadataType::FOCAL_LENGTH_PX:
    n += proc(buffer.focalLengthPx, last.focalLengthPx, &TimedMetadataListener::processFocalLengthPx, l, t);
    break;
  case MetadataType::INTRINSIC_MATRIX:
    n += proc(buffer.intrinsicMatrix, last.intrinsicMatrix, &TimedMetadataListener::processIntrinsicMatrix, l, t);
    break;
  case MetadataType::AZIMUTH:
    n += proc(buffer.azimuth, last.azimuth, &TimedMetadataListener::processAzimuth, l, t);
    break;
  case MetadataType::ROLL_PITCH:
    n += proc(buffer.rollPitch, last.rollPitch, &TimedMetadataListener::processRollPitch, l, t);
    break;
  case MetadataType::GNSS_POSITION:
    n += proc(buffer.fix, last.fix, &TimedMetadataListener::processGNSSPosition, l, t);
    break;
  case MetadataType::ACCELERATION:
    n += proc(buffer.acceleration, last.acceleration, &TimedMetadataListener::processAcceleration, l, t);
    break;
  case MetadataType::MAGNETIC_FIELD:
    n += proc(buffer.magneticField, last.magneticField, &TimedMetadataListener::processMagneticField, l, t);
    break;
  case MetadataType::ANGULAR_VELOCITY:
    n += proc(buffer.angularVelocity, last.angularVelocity, &TimedMetadataListener::processAngularVelocity, l, t);
    break;
  case MetadataType::FACES:
    n += proc(buffer.faces, last.faces, &TimedMetadataListener::processFaces, l, t);
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
  CRAS_INFO_NAMED("gpmf", "seek to %s", cras::to_string(seekTime).c_str());

  // When the movie is seeked, we need to flush everything we buffered and wait for reading new data.
  this->data->lastTime = seekTime;
  this->data->buffer.clear();
  this->data->lastMetadata.clear();
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
