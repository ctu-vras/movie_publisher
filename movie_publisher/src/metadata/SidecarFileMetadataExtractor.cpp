// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Extractor of metadata from sidecar YAML files.
 * \author Martin Pecka
 */

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/array.hpp>
#include <yaml-cpp/yaml.h>
#include CXX_FILESYSTEM_INCLUDE
namespace fs = CXX_FILESYSTEM_NAMESPACE;

#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/string_utils/ros.hpp>
#include <cras_cpp_common/type_utils.hpp>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <movie_publisher/metadata_cache.h>
#include <movie_publisher/metadata_manager.h>
#include <movie_publisher/metadata/SidecarFileMetadataExtractor.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/MagneticField.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "MetadataYAMLParser.cpp"

namespace cras
{
template<typename T, std::size_t N>
inline ::std::string to_string(const ::boost::array<T, N>& value)
{
  ::std::stringstream ss;
  ss << ("[");
  size_t i = 0;
  for (const auto& v : value)
  {
    ss << ::cras::quoteIfStringType(::cras::to_string(v), v);
    if (i + 1 < value.size())ss << ", ";
    ++i;
  }
  ss << ("]");
  return ss.str();
}

template<typename T, typename S>
inline ::std::string to_string(const ::std::pair<T, S>& value)
{
  return cras::to_string(value.first) + "," + cras::to_string(value.second);
}

}

namespace movie_publisher
{

struct SidecarFileMetadataExtractor::Impl : HasLogger
{
  explicit Impl(const cras::LogHelperPtr& log, const std::string& filename, const MovieInfo::ConstPtr& info,
    const std::weak_ptr<MetadataManager>& manager) : HasLogger(log), info(info), filename(filename), manager(manager)
  {
    this->yamlFilename = this->filename + ".yaml";
    if (!fs::exists(this->yamlFilename))
      this->yamlFilename = this->filename + ".yml";
    if (!fs::exists(this->yamlFilename))
      return;

    try
    {
      this->sidecar = std::make_unique<YAML::Node>(YAML::LoadFile(this->yamlFilename));
      CRAS_INFO_NAMED("sidecar", "Opened YAML sidecar file %s", this->yamlFilename.c_str());
    }
    catch (const YAML::ParserException& e)
    {
      this->sidecar.reset();
      CRAS_ERROR_NAMED("sidecar", "Error opening YAML sidecar %s: %s", this->yamlFilename.c_str(), e.what());
    }

    if (this->sidecar && (*this->sidecar)["timed"])
    {
      const auto& timed = (*this->sidecar)["timed"];
      const auto hasTimed = [timed](const std::string& key)
      {
        return timed[key].IsDefined() || timed[cras::toLower(key)].IsDefined();
      };
      if (hasTimed("ROTATION")) this->supportedTimedMetadata.insert(MetadataType::ROTATION);
      if (hasTimed("CROP_FACTOR")) this->supportedTimedMetadata.insert(MetadataType::CROP_FACTOR);
      if (hasTimed("FOCAL_LENGTH_35MM")) this->supportedTimedMetadata.insert(MetadataType::FOCAL_LENGTH_35MM);
      if (hasTimed("SENSOR_SIZE_MM")) this->supportedTimedMetadata.insert(MetadataType::SENSOR_SIZE_MM);
      if (hasTimed("FOCAL_LENGTH_MM")) this->supportedTimedMetadata.insert(MetadataType::FOCAL_LENGTH_MM);
      if (hasTimed("FOCAL_LENGTH_PX")) this->supportedTimedMetadata.insert(MetadataType::FOCAL_LENGTH_PX);
      if (hasTimed("INTRINSIC_MATRIX")) this->supportedTimedMetadata.insert(MetadataType::INTRINSIC_MATRIX);
      if (hasTimed("DISTORTION")) this->supportedTimedMetadata.insert(MetadataType::DISTORTION);
      if (hasTimed("GNSS_POSITION")) this->supportedTimedMetadata.insert(MetadataType::GNSS_POSITION);
      if (hasTimed("AZIMUTH")) this->supportedTimedMetadata.insert(MetadataType::AZIMUTH);
      if (hasTimed("MAGNETIC_FIELD")) this->supportedTimedMetadata.insert(MetadataType::MAGNETIC_FIELD);
      if (hasTimed("ROLL_PITCH")) this->supportedTimedMetadata.insert(MetadataType::ROLL_PITCH);
      if (hasTimed("ACCELERATION")) this->supportedTimedMetadata.insert(MetadataType::ACCELERATION);
      if (hasTimed("ANGULAR_VELOCITY")) this->supportedTimedMetadata.insert(MetadataType::ANGULAR_VELOCITY);
      if (hasTimed("FACES")) this->supportedTimedMetadata.insert(MetadataType::FACES);
      if (hasTimed("CAMERA_INFO")) this->supportedTimedMetadata.insert(MetadataType::CAMERA_INFO);
      if (hasTimed("IMU")) this->supportedTimedMetadata.insert(MetadataType::IMU);
      if (hasTimed("OPTICAL_FRAME_TF")) this->supportedTimedMetadata.insert(MetadataType::OPTICAL_FRAME_TF);
      if (hasTimed("ZERO_ROLL_PITCH_TF")) this->supportedTimedMetadata.insert(MetadataType::ZERO_ROLL_PITCH_TF);
    }
  }

  MovieInfo::ConstPtr info;  //!< Movie info.
  std::string filename;  //!< Filename of the movie.
  std::weak_ptr<MetadataManager> manager;  //!< Metadata manager.
  std::string yamlFilename;  //!< Filename of the sidecar file.
  bool staticMetadataLoaded {false};  //!< Whether there has already been an attempt at opening the sidecar file.
  std::unique_ptr<YAML::Node> sidecar;  //!< The loaded sidecar YAML file
  MetadataCache cache;  //!< The loaded metadata.
  LatestMetadataCache timedDefaults;  //!< Default values for timed metadata.

  //! Metadata supported by the currently loaded movie.
  std::unordered_set<MetadataType> supportedTimedMetadata;
  StreamTime lastTime;  //!< Last seek time.
  std::unordered_map<MetadataType, StreamTime> lastTimes;  //!< Last processed times for each type of metadata.

  void loadStaticMetadata();
  void parseStaticMetadata(const YAML::Node& node, LatestMetadataCache& latest);
  void loadTimedMetadata(const std::unordered_set<MetadataType>& metadata);
};

void SidecarFileMetadataExtractor::Impl::loadStaticMetadata()
{
  if (this->staticMetadataLoaded)
    return;
  this->staticMetadataLoaded = true;

  std::list<std::string> additionalSidecars = { "metadata" };
  const auto manager = this->manager.lock();
  if (manager != nullptr)
  {
    // Read camera and lens identification from manager and then reset the manager cache to previous state
    std::array<cras::optional<cras::optional<std::string>>, 7> cached = {
      manager->getCache()->latest.getCameraMake(),
      manager->getCache()->latest.getCameraModel(),
      manager->getCache()->latest.getLensMake(),
      manager->getCache()->latest.getLensMake(),
      manager->getCache()->latest.getCameraGeneralName(),
      manager->getCache()->latest.getCameraUniqueName(),
      manager->getCache()->latest.getCameraSerialNumber(),
    };
    const auto camMake = manager->getCameraMake().value_or("");
    const auto camModel = manager->getCameraModel().value_or("");
    const auto lensMake = manager->getLensMake().value_or("");
    const auto lensModel = manager->getLensModel().value_or("");
    const auto genName = manager->getCameraGeneralName().value_or("");
    const auto uniqueName = manager->getCameraUniqueName().value_or("");
    manager->getCache()->latest.getCameraMake() = cached[0];
    manager->getCache()->latest.getCameraModel() = cached[1];
    manager->getCache()->latest.getLensMake() = cached[2];
    manager->getCache()->latest.getLensModel() = cached[3];
    manager->getCache()->latest.getCameraGeneralName() = cached[4];
    manager->getCache()->latest.getCameraUniqueName() = cached[5];
    manager->getCache()->latest.getCameraSerialNumber() = cached[6];

    if (!camMake.empty())
      additionalSidecars.push_back(camMake);
    if (!camMake.empty() && !camModel.empty())
      additionalSidecars.push_back(camMake + "_" + camModel);
    if (!lensMake.empty() && !lensModel.empty())
      additionalSidecars.push_back(lensMake + "_" + lensModel);
    if (!genName.empty())
      additionalSidecars.push_back(genName);
    if (!uniqueName.empty())
      additionalSidecars.push_back(uniqueName);
    if (this->sidecar != nullptr && (*this->sidecar)["defaults"])
    {
      try
      {
        additionalSidecars.push_back((*this->sidecar)["defaults"].as<std::string>());
      }
      catch (const std::exception& e)
      {
        CRAS_WARN_NAMED("sidecar", "Invalid value in the [defaults] key. Expected string. Error: %s", e.what());
      }
    }
  }

  const auto dir = fs::path(this->filename).parent_path();
  for (const auto& additionalSidecar : additionalSidecars)
  {
    const auto path = (dir / cras::toValidRosName(additionalSidecar)).string();
    std::string sidecarFile {};
    if (fs::exists(path + ".yaml"))
      sidecarFile = path + ".yaml";
    else if (fs::exists(path + ".yml"))
      sidecarFile = path + ".yml";
    else
    {
      CRAS_DEBUG_NAMED("sidecar", "Defaults file %s.(yaml|yml) not found.", path.c_str());
      continue;
    }
    CRAS_DEBUG_NAMED("sidecar", "Defaults file %s found.", sidecarFile.c_str());

    try
    {
      auto node = YAML::LoadFile(sidecarFile);
      if (!node)
        continue;

      this->parseStaticMetadata(node, this->cache.latest);
      if (this->info->isStillImage())
      {
        this->parseStaticMetadata(node["still"], this->cache.latest);
      }
      else
      {
        this->parseStaticMetadata(node["video"], this->cache.latest);
        this->parseStaticMetadata(node["timed"], this->timedDefaults);
      }
      CRAS_INFO_NAMED("sidecar", "Defaults file %s loaded.", sidecarFile.c_str());
    }
    catch (const std::exception& e)
    {
      CRAS_WARN_NAMED("sidecar", "Error loading %s: YAML exception: %s", sidecarFile.c_str(), e.what());
    }
  }

  if (this->sidecar == nullptr)
    return;

  try
  {
    this->parseStaticMetadata(*this->sidecar, this->cache.latest);
    CRAS_INFO_NAMED("sidecar", "Loaded YAML sidecar file %s", this->yamlFilename.c_str());
  }
  catch (const YAML::ParserException& e)
  {
    CRAS_ERROR_NAMED("sidecar", "Error parsing YAML sidecar %s: %s", this->yamlFilename.c_str(), e.what());
  }
}

template<typename T>
void readYAML(const YAML::Node& node, const std::string& key, cras::optional<cras::optional<T>>& dest,
  const std::function<bool(const T&)>& validateFn = [](const T&) {return true;})
{
  auto realKey = key;
  if (!node[key] && node[cras::toLower(key)])
    realKey = cras::toLower(key);

  if (node[realKey])
  {
    T value {};
    if (dest.has_value() && dest->has_value())
      value = dest->value();
    YAML::updateFromYAML(node[realKey], value);
    CRAS_DEBUG_STREAM_NAMED("sidecar", realKey << " '" << cras::to_string(value) << "' read from sidecar YAML file.");
    if (validateFn(value))
      dest.emplace().emplace() = value;
  }
}

void readYAML(const YAML::Node& node, const std::string& key, cras::optional<GNSSFixAndDetail>& dest,
  const std::function<bool(const GNSSFixAndDetail&)>& validateFn = [](const GNSSFixAndDetail&) {return true;})
{
  auto realKey = key;
  if (!node[key] && node[cras::toLower(key)])
    realKey = cras::toLower(key);

  if (node[realKey] && node[realKey].IsMap())
  {
    auto value = dest.value_or(GNSSFixAndDetail{});
    if (!value.first.has_value())
      value.first.emplace();
    if (!value.second.has_value())
      value.second.emplace();
    YAML::updateFromYAML(node[realKey], *value.first);
    YAML::updateFromYAML(node[realKey], *value.second);

    CRAS_DEBUG_STREAM_NAMED("sidecar",
      realKey << " '" << cras::to_string(*value.first) << "' read from sidecar YAML file.");
    if (validateFn(value))
      dest.emplace() = value;
  }
}

template<typename T, typename O = cras::optional<T>>
TimedMetadata<T> getInitialTimedValue(const YAML::Node& node, const O& latest, const cras::optional<O>& defaults)
{
  TimedMetadata<T> value{};
  if (defaults.has_value() && defaults->has_value())
    value.value = defaults->value();

  try
  {
    if (latest.has_value() && node["init_from_latest"] && node["init_from_latest"].as<bool>())
      value.value = *latest;
  }
  catch (const YAML::Exception&)
  {
  }
  return value;
}

template<>
TimedMetadata<GNSSFixAndDetail> getInitialTimedValue(
  const YAML::Node& node, const GNSSFixAndDetail& latest, const cras::optional<GNSSFixAndDetail>& defaults)
{
  TimedMetadata<GNSSFixAndDetail> value{};
  if (defaults.has_value())
    value.value = *defaults;
  try
  {
    if (latest.first.has_value() && node["init_from_latest"] && node["init_from_latest"].as<bool>())
      value.value.first = *latest.first;
    if (latest.second.has_value() && node["init_from_latest"] && node["init_from_latest"].as<bool>())
      value.value.second = *latest.second;
  }
  catch (const YAML::Exception&)
  {
  }
  return value;
}

template<typename T, typename O = cras::optional<T>>
void readTimedYAML(const YAML::Node& node, const std::string& key, std::vector<TimedMetadata<T>>& dest, const O& latest,
  const cras::optional<O>& defaults)
{
  auto realKey = key;
  if (!node[key] && node[cras::toLower(key)])
    realKey = cras::toLower(key);

  if (node[realKey] && node[realKey].IsSequence())
  {
    for (const auto& data : node[realKey])
    {
      try
      {
        if (data.IsMap())
        {
          auto value = getInitialTimedValue<T>(data, latest, defaults);
          if (!YAML::convert<TimedMetadata<T>>::decode(data, value))
            throw YAML::TypedBadConversion<T>(data.Mark());
          dest.push_back(value);
        }
      }
      catch (const YAML::Exception& e)
      {
        CRAS_ERROR_THROTTLE_NAMED(1.0, "sidecar", "Error reading timed metadata %s: %s", realKey.c_str(), e.what());
      }
    }
    CRAS_DEBUG_STREAM_NAMED("sidecar",
      dest.size() << " timed metadata '" << realKey << "' read from sidecar YAML file.");
  }
}

void SidecarFileMetadataExtractor::Impl::parseStaticMetadata(const YAML::Node& node, LatestMetadataCache& latest)
{
  if (!node)
    return;
  std::function<bool(const int&)> validateRotation = [this](const int& rotation)
  {
    if (rotation == 0 || rotation == 90 || rotation == 180 || rotation == 270)
      return true;
    CRAS_WARN_NAMED("sidecar", "Invalid rotation: %i", rotation);
    return false;
  };
  readYAML(node, "CAMERA_GENERAL_NAME", latest.getCameraGeneralName());
  readYAML(node, "CAMERA_UNIQUE_NAME", latest.getCameraUniqueName());
  readYAML(node, "CAMERA_SERIAL_NUMBER", latest.getCameraSerialNumber());
  readYAML(node, "CAMERA_MAKE", latest.getCameraMake());
  readYAML(node, "CAMERA_MODEL", latest.getCameraModel());
  readYAML(node, "LENS_MAKE", latest.getLensMake());
  readYAML(node, "LENS_MODEL", latest.getLensModel());
  readYAML(node, "ROTATION", latest.getRotation(), validateRotation);
  readYAML(node, "CREATION_TIME", latest.getCreationTime());
  readYAML(node, "CROP_FACTOR", latest.getCropFactor());
  readYAML(node, "SENSOR_SIZE_MM", latest.getSensorSizeMM());
  readYAML(node, "FOCAL_LENGTH_35MM", latest.getFocalLength35MM());
  readYAML(node, "FOCAL_LENGTH_MM", latest.getFocalLengthMM());
  readYAML(node, "FOCAL_LENGTH_PX", latest.getFocalLengthPx());
  readYAML(node, "INTRINSIC_MATRIX", latest.getIntrinsicMatrix());
  readYAML(node, "DISTORTION", latest.getDistortion());
  readYAML(node, "GNSS_POSITION", latest.getGNSSPosition());
  readYAML(node, "AZIMUTH", latest.getAzimuth());
  readYAML(node, "MAGNETIC_FIELD", latest.getMagneticField());
  readYAML(node, "ROLL_PITCH", latest.getRollPitch());
  readYAML(node, "ACCELERATION", latest.getAcceleration());
  readYAML(node, "ANGULAR_VELOCITY", latest.getAngularVelocity());
  readYAML(node, "FACES", latest.getFaces());
  readYAML(node, "CAMERA_INFO", latest.getCameraInfo());
  readYAML(node, "IMU", latest.getImu());
  readYAML(node, "OPTICAL_FRAME_TF", latest.getOpticalFrameTF());
  readYAML(node, "ZERO_ROLL_PITCH_TF", latest.getZeroRollPitchTF());
}

void SidecarFileMetadataExtractor::Impl::loadTimedMetadata(const std::unordered_set<MetadataType>& metadata)
{
  if (this->sidecar == nullptr || !(*this->sidecar)["timed"])
    return;
  const auto manager = this->manager.lock();
  if (manager == nullptr)
    return;

  const auto& timed = (*this->sidecar)["timed"];
  const auto& m = manager;
  const auto& d = this->timedDefaults;

  YAML::convert<StreamTime>::fps = this->info->frameRate();

  auto& t = this->cache.timed;
  if (metadata.find(MetadataType::ROTATION) != metadata.end())
    readTimedYAML(timed, "ROTATION", t.rotation(), m->getRotation(), d.getRotation());
  if (metadata.find(MetadataType::CROP_FACTOR) != metadata.end())
    readTimedYAML(timed, "CROP_FACTOR", t.cropFactor(), m->getCropFactor(), d.getCropFactor());
  if (metadata.find(MetadataType::SENSOR_SIZE_MM) != metadata.end())
    readTimedYAML(timed, "SENSOR_SIZE_MM", t.sensorSizeMM(), m->getSensorSizeMM(), d.getSensorSizeMM());
  if (metadata.find(MetadataType::FOCAL_LENGTH_35MM) != metadata.end())
    readTimedYAML(timed, "FOCAL_LENGTH_35MM", t.focalLength35MM(), m->getFocalLength35MM(), d.getFocalLength35MM());
  if (metadata.find(MetadataType::FOCAL_LENGTH_MM) != metadata.end())
    readTimedYAML(timed, "FOCAL_LENGTH_MM", t.focalLengthMM(), m->getFocalLengthMM(), d.getFocalLengthMM());
  if (metadata.find(MetadataType::FOCAL_LENGTH_PX) != metadata.end())
    readTimedYAML(timed, "FOCAL_LENGTH_PX", t.focalLengthPx(), m->getFocalLengthPx(), d.getFocalLengthPx());
  if (metadata.find(MetadataType::INTRINSIC_MATRIX) != metadata.end())
    readTimedYAML(timed, "INTRINSIC_MATRIX", t.intrinsicMatrix(), m->getIntrinsicMatrix(), d.getIntrinsicMatrix());
  if (metadata.find(MetadataType::DISTORTION) != metadata.end())
    readTimedYAML(timed, "DISTORTION", t.distortion(), m->getDistortion(), d.getDistortion());
  if (metadata.find(MetadataType::GNSS_POSITION) != metadata.end())
    readTimedYAML(timed, "GNSS_POSITION", t.gnssPosition(), m->getGNSSPosition(), d.getGNSSPosition());
  if (metadata.find(MetadataType::AZIMUTH) != metadata.end())
    readTimedYAML(timed, "AZIMUTH", t.azimuth(), m->getAzimuth(), d.getAzimuth());
  if (metadata.find(MetadataType::MAGNETIC_FIELD) != metadata.end())
    readTimedYAML(timed, "MAGNETIC_FIELD", t.magneticField(), m->getMagneticField(), d.getMagneticField());
  if (metadata.find(MetadataType::ROLL_PITCH) != metadata.end())
    readTimedYAML(timed, "ROLL_PITCH", t.rollPitch(), m->getRollPitch(), d.getRollPitch());
  if (metadata.find(MetadataType::ACCELERATION) != metadata.end())
    readTimedYAML(timed, "ACCELERATION", t.acceleration(), m->getAcceleration(), d.getAcceleration());
  if (metadata.find(MetadataType::ANGULAR_VELOCITY) != metadata.end())
    readTimedYAML(timed, "ANGULAR_VELOCITY", t.angularVelocity(), m->getAngularVelocity(), d.getAngularVelocity());
  if (metadata.find(MetadataType::FACES) != metadata.end())
    readTimedYAML(timed, "FACES", t.faces(), m->getFaces(), d.getFaces());
  if (metadata.find(MetadataType::CAMERA_INFO) != metadata.end())
    readTimedYAML(timed, "CAMERA_INFO", t.cameraInfo(), m->getCameraInfo(), d.getCameraInfo());
  if (metadata.find(MetadataType::IMU) != metadata.end())
    readTimedYAML(timed, "IMU", t.imu(), m->getImu(), d.getImu());
  if (metadata.find(MetadataType::OPTICAL_FRAME_TF) != metadata.end())
    readTimedYAML(timed, "OPTICAL_FRAME_TF", t.opticalFrameTF(), m->getOpticalFrameTF(), d.getOpticalFrameTF());
  if (metadata.find(MetadataType::ZERO_ROLL_PITCH_TF) != metadata.end())
    readTimedYAML(timed, "ZERO_ROLL_PITCH_TF", t.zeroRollPitchTF(), m->getZeroRollPitchTF(), d.getZeroRollPitchTF());
}

SidecarFileMetadataExtractor::SidecarFileMetadataExtractor(const cras::LogHelperPtr& log, const std::string& filename,
  const MovieInfo::ConstPtr& info, const std::weak_ptr<MetadataManager>& manager)
  : TimedMetadataExtractor(log), data(new Impl(log, filename, info, manager))
{
}

std::string SidecarFileMetadataExtractor::getName() const
{
  return cras::getTypeName<std::remove_cv_t<std::remove_reference_t<decltype(*this)>>>();
}

int SidecarFileMetadataExtractor::getPriority() const
{
  return 0;
}

cras::optional<std::string> SidecarFileMetadataExtractor::getCameraGeneralName()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getCameraGeneralName().value_or(cras::nullopt);
}

cras::optional<std::string> SidecarFileMetadataExtractor::getCameraUniqueName()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getCameraUniqueName().value_or(cras::nullopt);
}

cras::optional<std::string> SidecarFileMetadataExtractor::getCameraSerialNumber()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getCameraSerialNumber().value_or(cras::nullopt);
}

cras::optional<std::string> SidecarFileMetadataExtractor::getCameraMake()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getCameraMake().value_or(cras::nullopt);
}

cras::optional<std::string> SidecarFileMetadataExtractor::getCameraModel()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getCameraModel().value_or(cras::nullopt);
}

cras::optional<std::string> SidecarFileMetadataExtractor::getLensMake()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getLensMake().value_or(cras::nullopt);
}

cras::optional<std::string> SidecarFileMetadataExtractor::getLensModel()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getLensModel().value_or(cras::nullopt);
}

cras::optional<int> SidecarFileMetadataExtractor::getRotation()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getRotation().value_or(cras::nullopt);
}

cras::optional<ros::Time> SidecarFileMetadataExtractor::getCreationTime()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getCreationTime().value_or(cras::nullopt);
}

cras::optional<double> SidecarFileMetadataExtractor::getCropFactor()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getCropFactor().value_or(cras::nullopt);
}

cras::optional<SensorSize> SidecarFileMetadataExtractor::getSensorSizeMM()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getSensorSizeMM().value_or(cras::nullopt);
}

cras::optional<double> SidecarFileMetadataExtractor::getFocalLength35MM()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getFocalLength35MM().value_or(cras::nullopt);
}

cras::optional<double> SidecarFileMetadataExtractor::getFocalLengthMM()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getFocalLengthMM().value_or(cras::nullopt);
}

cras::optional<double> SidecarFileMetadataExtractor::getFocalLengthPx()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getFocalLengthPx().value_or(cras::nullopt);
}

cras::optional<IntrinsicMatrix> SidecarFileMetadataExtractor::getIntrinsicMatrix()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getIntrinsicMatrix().value_or(cras::nullopt);
}

cras::optional<DistortionData> SidecarFileMetadataExtractor::getDistortion()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getDistortion().value_or(cras::nullopt);
}

GNSSFixAndDetail SidecarFileMetadataExtractor::getGNSSPosition()
{
  this->data->loadStaticMetadata();
  if (!this->data->cache.latest.getGNSSPosition().has_value())
    return {cras::nullopt, cras::nullopt};
  return *this->data->cache.latest.getGNSSPosition();
}

cras::optional<compass_msgs::Azimuth> SidecarFileMetadataExtractor::getAzimuth()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getAzimuth().value_or(cras::nullopt);
}

cras::optional<sensor_msgs::MagneticField> SidecarFileMetadataExtractor::getMagneticField()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getMagneticField().value_or(cras::nullopt);
}

cras::optional<RollPitch> SidecarFileMetadataExtractor::getRollPitch()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getRollPitch().value_or(cras::nullopt);
}

cras::optional<geometry_msgs::Vector3> SidecarFileMetadataExtractor::getAcceleration()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getAcceleration().value_or(cras::nullopt);
}

cras::optional<geometry_msgs::Vector3> SidecarFileMetadataExtractor::getAngularVelocity()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getAngularVelocity().value_or(cras::nullopt);
}

cras::optional<vision_msgs::Detection2DArray> SidecarFileMetadataExtractor::getFaces()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getFaces().value_or(cras::nullopt);
}

cras::optional<sensor_msgs::CameraInfo> SidecarFileMetadataExtractor::getCameraInfo()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getCameraInfo().value_or(cras::nullopt);
}

cras::optional<sensor_msgs::Imu> SidecarFileMetadataExtractor::getImu()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getImu().value_or(cras::nullopt);
}

cras::optional<geometry_msgs::Transform> SidecarFileMetadataExtractor::getOpticalFrameTF()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getOpticalFrameTF().value_or(cras::nullopt);
}

cras::optional<geometry_msgs::Transform> SidecarFileMetadataExtractor::getZeroRollPitchTF()
{
  this->data->loadStaticMetadata();
  return this->data->cache.latest.getZeroRollPitchTF().value_or(cras::nullopt);
}

// TIMED METADATA

std::unordered_set<MetadataType> SidecarFileMetadataExtractor::supportedTimedMetadata(
  const std::unordered_set<MetadataType>& availableMetadata) const
{
  return this->data->supportedTimedMetadata;
}

bool SidecarFileMetadataExtractor::hasTimedMetadata() const
{
  return !this->data->supportedTimedMetadata.empty();
}

void SidecarFileMetadataExtractor::prepareTimedMetadata(const std::unordered_set<MetadataType>& metadataTypes)
{
  this->data->loadTimedMetadata(metadataTypes);
}

/**
 * \brief Helper function to pass timed metadata to listeners.
 * \tparam T Type of metadata.
 * \param[in] timed The timed metadata to be processed.
 * \param[in] processFn The listener function that should be called on each listener.
 * \param[in] listeners The list of listeners.
 */
template<typename T>
size_t proc(const std::vector<TimedMetadata<T>>& timed,
  void(TimedMetadataListener::*processFn)(const TimedMetadata<T>&),
  const std::vector<std::shared_ptr<TimedMetadataListener>>& listeners)
{
  for (const auto& data : timed)
  {
    for (const auto& listener : listeners)
      (listener.get()->*processFn)(data);
  }
  return timed.size();
}

size_t SidecarFileMetadataExtractor::processTimedMetadata(const MetadataType type, const StreamTime& maxTime, bool)
{
  auto minTime = this->data->lastTime;
  auto includeMinTime = true;  // Whether metadata from exactly minTime should be included or not
  if (this->data->lastTimes.count(type) > 0)
  {
    minTime = this->data->lastTimes[type];
    includeMinTime = false;
  }

  const auto& timed = this->data->cache.timed;
  const auto& lists = this->listeners;
  const auto& i = includeMinTime;
  using TML = TimedMetadataListener;

  size_t n = 0;
  switch (type)
  {
    case MetadataType::ROTATION:
      n = proc(findBetweenStamps(timed.rotation(), minTime, maxTime, i), &TML::processRotation, lists);
      break;
    case MetadataType::CROP_FACTOR:
      n = proc(findBetweenStamps(timed.cropFactor(), minTime, maxTime, i), &TML::processCropFactor, lists);
      break;
    case MetadataType::SENSOR_SIZE_MM:
      n = proc(findBetweenStamps(timed.sensorSizeMM(), minTime, maxTime, i), &TML::processSensorSizeMM, lists);
      break;
    case MetadataType::FOCAL_LENGTH_35MM:
      n = proc(findBetweenStamps(timed.focalLength35MM(), minTime, maxTime, i), &TML::processFocalLength35MM, lists);
      break;
    case MetadataType::FOCAL_LENGTH_MM:
      n = proc(findBetweenStamps(timed.focalLengthMM(), minTime, maxTime, i), &TML::processFocalLengthMM, lists);
      break;
    case MetadataType::FOCAL_LENGTH_PX:
      n = proc(findBetweenStamps(timed.focalLengthPx(), minTime, maxTime, i), &TML::processFocalLengthPx, lists);
      break;
    case MetadataType::INTRINSIC_MATRIX:
      n = proc(findBetweenStamps(timed.intrinsicMatrix(), minTime, maxTime, i), &TML::processIntrinsicMatrix, lists);
      break;
    case MetadataType::DISTORTION:
      n = proc(findBetweenStamps(timed.distortion(), minTime, maxTime, i), &TML::processDistortion, lists);
      break;
    case MetadataType::GNSS_POSITION:
      n = proc(findBetweenStamps(timed.gnssPosition(), minTime, maxTime, i), &TML::processGNSSPosition, lists);
      break;
    case MetadataType::AZIMUTH:
      n = proc(findBetweenStamps(timed.azimuth(), minTime, maxTime, i), &TML::processAzimuth, lists);
      break;
    case MetadataType::MAGNETIC_FIELD:
      n = proc(findBetweenStamps(timed.magneticField(), minTime, maxTime, i), &TML::processMagneticField, lists);
      break;
    case MetadataType::ROLL_PITCH:
      n = proc(findBetweenStamps(timed.rollPitch(), minTime, maxTime, i), &TML::processRollPitch, lists);
      break;
    case MetadataType::ACCELERATION:
      n = proc(findBetweenStamps(timed.acceleration(), minTime, maxTime, i), &TML::processAcceleration, lists);
      break;
    case MetadataType::ANGULAR_VELOCITY:
      n = proc(findBetweenStamps(timed.angularVelocity(), minTime, maxTime, i), &TML::processAngularVelocity, lists);
      break;
    case MetadataType::FACES:
      n = proc(findBetweenStamps(timed.faces(), minTime, maxTime, i), &TML::processFaces, lists);
      break;
    case MetadataType::CAMERA_INFO:
      n = proc(findBetweenStamps(timed.cameraInfo(), minTime, maxTime, i), &TML::processCameraInfo, lists);
      break;
    case MetadataType::IMU:
      n = proc(findBetweenStamps(timed.imu(), minTime, maxTime, i), &TML::processImu, lists);
      break;
    case MetadataType::OPTICAL_FRAME_TF:
      n = proc(findBetweenStamps(timed.opticalFrameTF(), minTime, maxTime, i), &TML::processOpticalFrameTF, lists);
      break;
    case MetadataType::ZERO_ROLL_PITCH_TF:
      n = proc(findBetweenStamps(timed.zeroRollPitchTF(), minTime, maxTime, i), &TML::processZeroRollPitchTF, lists);
      break;
    default:
      n = 0;
      break;
  }
  this->data->lastTimes[type] = maxTime;
  return n;
}

void SidecarFileMetadataExtractor::seekTimedMetadata(const StreamTime& seekTime)
{
  this->data->lastTime = seekTime;
  this->data->lastTimes.clear();
}

MetadataExtractor::Ptr SidecarFileMetadataExtractorPlugin::getExtractor(const MetadataExtractorParams& params)
{
  if (params.log == nullptr || params.info == nullptr || params.info->filenameOrURL().empty())
    return nullptr;

  return std::make_shared<SidecarFileMetadataExtractor>(
    params.log, params.info->filenameOrURL(), params.info, params.manager);
}

}

PLUGINLIB_EXPORT_CLASS(movie_publisher::SidecarFileMetadataExtractorPlugin, movie_publisher::MetadataExtractorPlugin)
