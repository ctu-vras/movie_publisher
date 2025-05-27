// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Manager of multiple image metadata providers which can cooperate in parsing.
 * \author Martin Pecka
 */

#include <string>
#include <unordered_set>
#include <utility>

#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/suppress_warnings.h>
#include <cras_cpp_common/type_utils.hpp>
#include <geometry_msgs/Quaternion.h>
#include <movie_publisher/metadata_cache.h>
#include <movie_publisher/metadata_manager.h>
#include <ros/time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/NavSatFix.h>
#include <vision_msgs/Detection2DArray.h>

namespace movie_publisher
{

#define STACK_VERBOSE 0
#define STACK_SKIP_NON_OVERRIDEN 0

#if STACK_VERBOSE
void checkCacheDebugPrint(const std::string& func)
{
  CRAS_DEBUG_NAMED("metadata_manager", "Returned %s value from result cache.", func.c_str());
}
#else
void checkCacheDebugPrint(const std::string&) {}
#endif

// Enable the first branch to simplify the debugging outputs of stack by removing calls to non-overridden functions.
// The expression uses a GCC extension and some IDEs have a problem with it, so it is disabled by default.
#if STACK_SKIP_NON_OVERRIDEN && defined __GNUC__
template<typename T, typename M>
bool functionHasOverride(M* extractor, T(M::*getFn)())
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpmf-conversions"
  return (reinterpret_cast<void*>(extractor->*getFn) != reinterpret_cast<void*>(getFn));  // NOLINT
#pragma GCC diagnostic pop
}
#else
template<typename T, typename M>
bool functionHasOverride(M*, T(M::*)())
{
  return true;
}
#endif

template<typename T>
bool hasValue(const T& msg)
{
  return msg.has_value();
}
template<>
bool hasValue(const GNSSFixAndDetail& msg)
{
  return msg.first.has_value() || msg.second.has_value();
}

template<typename T, std::enable_if_t<!ros::message_traits::HasHeader<T>::value>* = nullptr>
std::string getFrameId(const cras::optional<T>&)
{
  return "";
}
template<typename T, std::enable_if_t<ros::message_traits::HasHeader<T>::value>* = nullptr>
std::string getFrameId(const cras::optional<T>& msg)
{
  return *ros::message_traits::frameId(*msg);
}
template<typename = GNSSFixAndDetail>
std::string getFrameId(const GNSSFixAndDetail& msg)
{
  if (msg.first.has_value() && !msg.first->header.frame_id.empty())
    return msg.first->header.frame_id;
  if (msg.second.has_value() && !msg.second->header.frame_id.empty())
    return msg.second->header.frame_id;
  return "";
}

template<typename T, std::enable_if_t<!ros::message_traits::HasHeader<T>::value>* = nullptr>
void setFrameId(cras::optional<T>&, const std::string&)
{
}
template<typename T, std::enable_if_t<ros::message_traits::HasHeader<T>::value>* = nullptr>
void setFrameId(cras::optional<T>& msg, const std::string& frameId)
{
  *ros::message_traits::frameId(*msg) = frameId;
}
template<typename = GNSSFixAndDetail>
void setFrameId(GNSSFixAndDetail& msg, const std::string& frameId)
{
  if (msg.first.has_value())
  {
    msg.first->header.frame_id = frameId;
  }
  if (msg.second.has_value())
  {
    msg.second->header.frame_id = frameId;
    msg.second->status.header.frame_id = frameId;
  }
}
template<typename = vision_msgs::Detection2DArray>
void setFrameId(cras::optional<vision_msgs::Detection2DArray>& msg, const std::string& frameId)
{
  msg->header.frame_id = frameId;
  for (auto& det : msg->detections)
    det.header.frame_id = frameId;
}

template<typename T, typename O = cras::optional<T>>
T MetadataManager::checkExtractors(const std::string& func,
  T(MetadataExtractor::*getFn)(), O(LatestMetadataCache::*getFnLatest)(), const std::string& frame)
{
  if ((this->cache->latest.*getFnLatest)().has_value())
  {
    checkCacheDebugPrint(func);
    return (this->cache->latest.*getFnLatest)().value();
  }
  if (this->stopRecursion(func, this))
    return T{};

  StackGuard stackGuard1(this->callStack, func, this);
  for (const auto& extractor : this->extractors)
  {
    if (this->stopRecursion(func, extractor.get()))
      continue;
    if (!functionHasOverride(extractor.get(), getFn))
      continue;
    StackGuard stackGuard2(this->callStack, func, extractor.get());
    auto val = (extractor.get()->*getFn)();
    if (hasValue(val))
    {
      if (!frame.empty() && getFrameId(val).empty())
        setFrameId(val, frame);

      (this->cache->latest.*getFnLatest)() = val;
      return val;
    }
  }
  return (this->cache->latest.*getFnLatest)().emplace(T{});
}

/**
 * \brief Call the requested function on the registered extractors and cache.
 * \param[in] getFn Name of the function.
 */
#define CHECK_EXTRACTORS(getFn) \
  return checkExtractors(__func__, &MetadataExtractor::getFn, &LatestMetadataCache::getFn);

/**
 * \brief Call the requested function on the registered extractors and cache.
 * \param[in] getFn Name of the function.
 * \param[in] frame If the metadata to be returned contain a frame_id and it is empty, set it to this value.
 */
#define CHECK_EXTRACTORS_WITH_FRAME(getFn, frame) \
  return checkExtractors(__func__, &MetadataExtractor::getFn, &LatestMetadataCache::getFn, frame);

/**
 * \brief A proxy for multiple metadata listeners that caches the data passed to their callbacks.
 */
struct CachingMetadataListener : public TimedMetadataListener
{
  /**
   * \param[in] listeners Reference to the array of registered listeners.
   * \param[in] cache The cache that is filled by this listener.
   */
  explicit CachingMetadataListener(
    std::vector<std::shared_ptr<TimedMetadataListener>>& listeners, const std::shared_ptr<MetadataCache>& cache)
    : listeners(listeners), cache(cache)
  {
  }
  void processRotation(const TimedMetadata<int>& data) override
  {
    this->cache->timed.rotation().push_back(data);
    this->cache->latest.getRotation() = data.value;
    for (const auto& listener : this->listeners)
      listener->processRotation(data);
  }
  void processCropFactor(const TimedMetadata<double>& data) override
  {
    this->cache->timed.cropFactor().push_back(data);
    this->cache->latest.getCropFactor() = data.value;
    for (const auto& listener : this->listeners)
      listener->processCropFactor(data);
  }
  void processSensorSizeMM(const TimedMetadata<SensorSize>& data) override
  {
    this->cache->timed.sensorSizeMM().push_back(data);
    this->cache->latest.getSensorSizeMM() = data.value;
    for (const auto& listener : this->listeners)
      listener->processSensorSizeMM(data);
  }
  void processFocalLength35MM(const TimedMetadata<double>& data) override
  {
    this->cache->timed.focalLength35MM().push_back(data);
    this->cache->latest.getFocalLength35MM() = data.value;
    for (const auto& listener : this->listeners)
      listener->processFocalLength35MM(data);
  }
  void processFocalLengthMM(const TimedMetadata<double>& data) override
  {
    this->cache->timed.focalLengthMM().push_back(data);
    this->cache->latest.getFocalLengthMM() = data.value;
    for (const auto& listener : this->listeners)
      listener->processFocalLengthMM(data);
  }
  void processFocalLengthPx(const TimedMetadata<double>& data) override
  {
    this->cache->timed.focalLengthPx().push_back(data);
    this->cache->latest.getFocalLengthPx() = data.value;
    for (const auto& listener : this->listeners)
      listener->processFocalLengthPx(data);
  }
  void processIntrinsicMatrix(const TimedMetadata<IntrinsicMatrix>& data) override
  {
    this->cache->timed.intrinsicMatrix().push_back(data);
    this->cache->latest.getIntrinsicMatrix() = data.value;
    for (const auto& listener : this->listeners)
      listener->processIntrinsicMatrix(data);
  }
  void processDistortion(const TimedMetadata<std::pair<DistortionType, Distortion>>& data) override
  {
    this->cache->timed.distortion().push_back(data);
    this->cache->latest.getDistortion() = data.value;
    for (const auto& listener : this->listeners)
      listener->processDistortion(data);
  }
  void processAzimuth(const TimedMetadata<compass_msgs::Azimuth>& data) override
  {
    this->cache->timed.azimuth().push_back(data);
    this->cache->latest.getAzimuth() = data.value;
    for (const auto& listener : this->listeners)
      listener->processAzimuth(data);
  }
  void processMagneticField(const TimedMetadata<sensor_msgs::MagneticField>& data) override
  {
    this->cache->timed.magneticField().push_back(data);
    this->cache->latest.getMagneticField() = data.value;
    for (const auto& listener : this->listeners)
      listener->processMagneticField(data);
  }
  void processRollPitch(const TimedMetadata<RollPitch>& data) override
  {
    this->cache->timed.rollPitch().push_back(data);
    this->cache->latest.getRollPitch() = data.value;
    for (const auto& listener : this->listeners)
      listener->processRollPitch(data);
  }
  void processAcceleration(const TimedMetadata<geometry_msgs::Vector3>& data) override
  {
    this->cache->timed.acceleration().push_back(data);
    this->cache->latest.getAcceleration() = data.value;
    for (const auto& listener : this->listeners)
      listener->processAcceleration(data);
  }
  void processAngularVelocity(const TimedMetadata<geometry_msgs::Vector3>& data) override
  {
    this->cache->timed.angularVelocity().push_back(data);
    this->cache->latest.getAngularVelocity() = data.value;
    for (const auto& listener : this->listeners)
      listener->processAngularVelocity(data);
  }
  void processFaces(const TimedMetadata<vision_msgs::Detection2DArray>& data) override
  {
    this->cache->timed.faces().push_back(data);
    this->cache->latest.getFaces() = data.value;
    for (const auto& listener : this->listeners)
      listener->processFaces(data);
  }
  void processCameraInfo(const TimedMetadata<sensor_msgs::CameraInfo>& data) override
  {
    this->cache->timed.cameraInfo().push_back(data);
    this->cache->latest.getCameraInfo() = data.value;
    for (const auto& listener : this->listeners)
      listener->processCameraInfo(data);
  }
  void processImu(const TimedMetadata<sensor_msgs::Imu>& data) override
  {
    this->cache->timed.imu().push_back(data);
    this->cache->latest.getImu() = data.value;
    for (const auto& listener : this->listeners)
      listener->processImu(data);
  }
  void processOpticalFrameTF(const TimedMetadata<geometry_msgs::Transform>& data) override
  {
    this->cache->timed.opticalFrameTF().push_back(data);
    this->cache->latest.getOpticalFrameTF() = data.value;
    for (const auto& listener : this->listeners)
      listener->processOpticalFrameTF(data);
  }
  void processGNSSPosition(const TimedMetadata<GNSSFixAndDetail>& data) override
  {
    this->cache->timed.gnssPosition().push_back(data);
    this->cache->latest.getGNSSPosition() = data.value;
    for (const auto& listener : this->listeners)
      listener->processGNSSPosition(data);
  }

  std::vector<std::shared_ptr<TimedMetadataListener>>& listeners;
  std::shared_ptr<MetadataCache> cache;
};

MetadataManager::MetadataManager(
  const cras::LogHelperPtr& log, const MovieOpenConfig& config, const MovieInfo::ConstPtr& info)
  : TimedMetadataExtractor(log),
    loader("movie_publisher", "movie_publisher::MetadataExtractorPlugin", "metadata_plugins"),
    config(config), info(info), width(info->width()), height(info->height()), cache(new MetadataCache())
{
  this->metadataListener = std::make_shared<CachingMetadataListener>(this->listeners, this->cache);
}

MetadataManager::~MetadataManager() = default;

std::string MetadataManager::getName() const
{
  return cras::getTypeName<std::remove_cv_t<std::remove_reference_t<decltype(*this)>>>();
}

int MetadataManager::getPriority() const
{
  return 0;
}

bool MetadataManager::stopRecursion(const std::string& fn, const MetadataExtractor* extractor) const
{
  const auto stackKey {std::pair{fn, extractor}};
  return std::find(this->callStack.begin(), this->callStack.end(), stackKey) != this->callStack.end();
}

StackGuard::StackGuard(decltype(MetadataManager::callStack)& stack,
  const std::string& fn, const MetadataExtractor* extractor) : stack(stack), fn(fn), extractor(extractor)
{
#if STACK_VERBOSE
  CRAS_DEBUG_NAMED("metadata_manager.call_stack", "Enter %-38s :: %s (%s)",
    extractor->getName().c_str(), fn.c_str(), this->getStackDescription().c_str());
#endif
  stack.emplace_back(fn, extractor);
}

StackGuard::~StackGuard()
{
  this->stack.pop_back();
#if STACK_VERBOSE
  CRAS_DEBUG_NAMED("metadata_manager.call_stack", "Exit  %-38s :: %s (%s)",
    this->extractor->getName().c_str(), this->fn.c_str(), this->getStackDescription().c_str());
#endif
}

std::string StackGuard::getStackDescription() const
{
  std::stringstream ss;
  bool first {true};
  for (const auto& [key, val] : this->stack)
  {
    if (!first)
      ss << "->";
    first = false;
    ss << key;
  }
  return ss.str();
}

void MetadataManager::addExtractor(const std::shared_ptr<MetadataExtractor>& extractor)
{
  this->extractors.insert(extractor);

  const auto timedExtractor = std::dynamic_pointer_cast<TimedMetadataExtractor>(extractor);
  if (timedExtractor != nullptr && timedExtractor->hasTimedMetadata())
  {
    this->timedExtractors.insert(timedExtractor);
    // Insert our proxy timed metadata listener
    timedExtractor->addTimedMetadataListener(this->metadataListener);
  }

  CRAS_DEBUG_NAMED("metadata_plugins", "%s %s added to metadata manager.",
    timedExtractor != nullptr ? "Timed extractor" : "Extractor", extractor->getName().c_str());
}

void MetadataManager::loadExtractorPlugins(const MetadataExtractorParams& params)
{
  const bool hasAllowed = this->config.rosParams()->hasParam("allowed_extractors");
  const auto allowed = this->config.rosParams()->getParam("allowed_extractors", std::unordered_set<std::string>{});
  const auto excluded = this->config.rosParams()->getParam("excluded_extractors", std::unordered_set<std::string>{});

  for (const auto& cl : this->loader.getDeclaredClasses())
  {
    if (excluded.count(cl) > 0)
    {
      CRAS_DEBUG_NAMED("metadata_plugins", "Excluding extractor %s .", cl.c_str());
      continue;
    }
    if (hasAllowed && allowed.count(cl) == 0)
    {
      CRAS_DEBUG_NAMED("metadata_plugins", "Extractor %s is not allowed.", cl.c_str());
      continue;
    }
    try
    {
      CRAS_DEBUG_NAMED("metadata_plugins", "Loading extractor plugin %s.", cl.c_str());
      const auto instance = this->loader.createUniqueInstance(cl);
      CRAS_DEBUG_NAMED("metadata_plugins", "Creating extractor from %s.", cl.c_str());
      auto extractor = instance->getExtractor(params);
      if (extractor != nullptr)
      {
        CRAS_DEBUG_NAMED("metadata_plugins", "Extractor %s successfully created.", extractor->getName().c_str());
        this->addExtractor(extractor);
      }
    }
    catch (const std::exception& e)
    {
      CRAS_WARN_NAMED("metadata_plugins", "Error loading metadata extractor plugin %s: %s.", cl.c_str(), e.what());
    }
  }
}

std::shared_ptr<MetadataCache> MetadataManager::getCache()
{
  return this->cache;
}

void MetadataManager::clearTimedMetadataCache()
{
  this->cache->timed.clear();
}

void MetadataManager::prepareTimedMetadata(const std::unordered_set<MetadataType>& metadataTypes)
{
  for (auto& extractor : this->timedExtractors)
    extractor->prepareTimedMetadata(metadataTypes);
}

std::unordered_set<MetadataType> MetadataManager::supportedTimedMetadata(
  const std::unordered_set<MetadataType>& availableMetadata) const
{
  std::unordered_set<MetadataType> supportedMetadata;
  std::unordered_set<MetadataType> mergedMetadata(availableMetadata);
  size_t lastSize;
  do
  {
    lastSize = supportedMetadata.size();
    for (const auto& extractor : this->timedExtractors)
    {
      const auto extractorSupported = extractor->supportedTimedMetadata(mergedMetadata);
      supportedMetadata.insert(extractorSupported.begin(), extractorSupported.end());
      mergedMetadata.insert(extractorSupported.begin(), extractorSupported.end());
    }
  } while (lastSize != supportedMetadata.size());
  return supportedMetadata;
}

size_t MetadataManager::processTimedMetadata(
  const MetadataType type, const StreamTime& maxTime, const bool requireOptional)
{
  for (const auto& extractor : this->timedExtractors)
  {
    const auto numProcessed = extractor->processTimedMetadata(type, maxTime, requireOptional);
    if (numProcessed > 0)
      return numProcessed;
  }

  return 0;
}

void MetadataManager::seekTimedMetadata(const StreamTime& seekTime)
{
  for (const auto& extractor : this->timedExtractors)
    extractor->seekTimedMetadata(seekTime);
}

bool MetadataManager::hasTimedMetadata() const
{
  return !this->timedExtractors.empty();
}

void MetadataManager::processPacket(const AVPacket* packet)
{
  for (const auto& extractor : this->extractors)
    extractor->processPacket(packet);
}

cras::optional<std::string> MetadataManager::getCameraGeneralName()
{
  CHECK_EXTRACTORS(getCameraGeneralName);
}

cras::optional<std::string> MetadataManager::getCameraUniqueName()
{
  CHECK_EXTRACTORS(getCameraUniqueName);
}

cras::optional<std::string> MetadataManager::getCameraSerialNumber()
{
  CHECK_EXTRACTORS(getCameraSerialNumber);
}

cras::optional<std::string> MetadataManager::getCameraMake()
{
  CHECK_EXTRACTORS(getCameraMake);
}

cras::optional<std::string> MetadataManager::getCameraModel()
{
  CHECK_EXTRACTORS(getCameraModel);
}

cras::optional<std::string> MetadataManager::getLensMake()
{
  CHECK_EXTRACTORS(getLensMake);
}

cras::optional<std::string> MetadataManager::getLensModel()
{
  CHECK_EXTRACTORS(getLensModel);
}

cras::optional<int> MetadataManager::getRotation()
{
  CHECK_EXTRACTORS(getRotation);
}

cras::optional<ros::Time> MetadataManager::getCreationTime()
{
  CHECK_EXTRACTORS(getCreationTime);
}

cras::optional<double> MetadataManager::getCropFactor()
{
  CHECK_EXTRACTORS(getCropFactor);
}

cras::optional<SensorSize> MetadataManager::getSensorSizeMM()
{
  CHECK_EXTRACTORS(getSensorSizeMM)
}

cras::optional<double> MetadataManager::getFocalLength35MM()
{
  CHECK_EXTRACTORS(getFocalLength35MM)
}
cras::optional<double> MetadataManager::getFocalLengthPx()
{
  CHECK_EXTRACTORS(getFocalLengthPx)
}

cras::optional<double> MetadataManager::getFocalLengthMM()
{
  CHECK_EXTRACTORS(getFocalLengthMM);
}

cras::optional<IntrinsicMatrix> MetadataManager::getIntrinsicMatrix()
{
  CHECK_EXTRACTORS(getIntrinsicMatrix);
}

cras::optional<std::pair<DistortionType, Distortion>> MetadataManager::getDistortion()
{
  CHECK_EXTRACTORS(getDistortion);
}

GNSSFixAndDetail MetadataManager::getGNSSPosition()
{
  CHECK_EXTRACTORS_WITH_FRAME(getGNSSPosition, this->config.frameId());
}

cras::optional<sensor_msgs::MagneticField> MetadataManager::getMagneticField()
{
  CHECK_EXTRACTORS_WITH_FRAME(getMagneticField, this->config.frameId());
}

cras::optional<compass_msgs::Azimuth> MetadataManager::getAzimuth()
{
  // TODO(peci1) compute from magnetic field and roll/pitch
  CHECK_EXTRACTORS_WITH_FRAME(getAzimuth, this->config.frameId());
}

cras::optional<RollPitch> MetadataManager::getRollPitch()
{
  CHECK_EXTRACTORS(getRollPitch);
}

cras::optional<geometry_msgs::Vector3> MetadataManager::getAngularVelocity()
{
  CHECK_EXTRACTORS(getAngularVelocity);
}

cras::optional<geometry_msgs::Vector3> MetadataManager::getAcceleration()
{
  CHECK_EXTRACTORS(getAcceleration);
}

cras::optional<vision_msgs::Detection2DArray> MetadataManager::getFaces()
{
  CHECK_EXTRACTORS_WITH_FRAME(getFaces, this->config.opticalFrameId());
}

cras::optional<sensor_msgs::CameraInfo> MetadataManager::getCameraInfo()
{
  CHECK_EXTRACTORS_WITH_FRAME(getCameraInfo, this->config.opticalFrameId());
}

cras::optional<sensor_msgs::Imu> MetadataManager::getImu()
{
  CHECK_EXTRACTORS_WITH_FRAME(getImu, this->config.frameId());
}

cras::optional<geometry_msgs::Transform> MetadataManager::getOpticalFrameTF()
{
  CHECK_EXTRACTORS(getOpticalFrameTF)
}

cras::optional<geometry_msgs::Transform> MetadataManager::getZeroRollPitchTF()
{
  CHECK_EXTRACTORS(getZeroRollPitchTF)
}
}
