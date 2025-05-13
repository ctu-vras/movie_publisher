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
#define CHECK_CACHE_DEBUG_PRINT(getFn) \
  CRAS_DEBUG_NAMED("metadata_manager", "Returned " #getFn " value from result cache.")
#else
#define CHECK_CACHE_DEBUG_PRINT(getFn)
#endif

/**
 * \brief Check if the function call result has already been cached. If so, return the cached result.
 * \param[in] getFn Name of the function.
 */
#define CHECK_CACHE(getFn) \
  if (this->cache->latest.getFn().has_value()) { \
    CHECK_CACHE_DEBUG_PRINT(getFn); \
    return this->cache->latest.getFn().value(); \
  }

// Enable the first branch to simplify the debugging outputs of stack by removing calls to non-overriden functions.
// The expression uses a GCC extension and some IDEs have a problem with it, so it is disabled by default.
#if STACK_SKIP_NON_OVERRIDEN
#define FUNCTION_HAS_OVERRIDE(extractor, getFn) \
  ((void*)(extractor.get()->*(&MetadataExtractor::getFn)) == (void*)(&MetadataExtractor::getFn))  // NOLINT
#else
#define FUNCTION_HAS_OVERRIDE(extractor, getFn) (true)
#endif

/**
 * \brief Call the given function in all extractors and return and cache the first valid result.
 * \param[in] getFn Name of the function.
 */
#define CHECK_EXTRACTORS(getFn) \
  CHECK_CACHE(getFn) \
  if (this->stopRecursion(__func__, this)) \
    return cras::nullopt; \
  StackGuard stackGuard1(this->callStack, __func__, this); \
  for (const auto& extractor : this->extractors) \
  {\
    if (this->stopRecursion(__func__, extractor.get())) \
      continue; \
    if (!FUNCTION_HAS_OVERRIDE(extractor, getFn)) \
      continue; \
    StackGuard stackGuard2(this->callStack, __func__, extractor.get()); \
    const auto& val = extractor->getFn(); \
    if (val.has_value()) \
    { \
      this->cache->latest.getFn() = val; \
      return val; \
    }\
  }

/**
 * \brief Last statement. Call when extracting data from all extractors failed and nothing is cached.
 * \param[in] getFn The function name.
 */
#define FINISH(getFn) \
  return this->cache->latest.getFn().emplace(cras::nullopt);

/**
 * \brief Call the requested function only on the registered extractors and cache, nothing more.
 * \param[in] getFn Name of the function.
 */
#define ONLY_CHECK_EXTRACTORS(getFn) \
  CHECK_EXTRACTORS(getFn) \
  FINISH(getFn)

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
    for (const auto& listener : this->listeners)
      listener->processRotation(data);
  }
  void processCropFactor(const TimedMetadata<double>& data) override
  {
    this->cache->timed.cropFactor().push_back(data);
    for (const auto& listener : this->listeners)
      listener->processCropFactor(data);
  }
  void processSensorSizeMM(const TimedMetadata<SensorSize>& data) override
  {
    this->cache->timed.sensorSizeMM().push_back(data);
    for (const auto& listener : this->listeners)
      listener->processSensorSizeMM(data);
  }
  void processFocalLength35MM(const TimedMetadata<double>& data) override
  {
    this->cache->timed.focalLength35MM().push_back(data);
    for (const auto& listener : this->listeners)
      listener->processFocalLength35MM(data);
  }
  void processFocalLengthMM(const TimedMetadata<double>& data) override
  {
    this->cache->timed.focalLengthMM().push_back(data);
    for (const auto& listener : this->listeners)
      listener->processFocalLengthMM(data);
  }
  void processFocalLengthPx(const TimedMetadata<double>& data) override
  {
    this->cache->timed.focalLengthPx().push_back(data);
    for (const auto& listener : this->listeners)
      listener->processFocalLengthPx(data);
  }
  void processIntrinsicMatrix(const TimedMetadata<IntrinsicMatrix>& data) override
  {
    this->cache->timed.intrinsicMatrix().push_back(data);
    for (const auto& listener : this->listeners)
      listener->processIntrinsicMatrix(data);
  }
  void processDistortion(const TimedMetadata<std::pair<DistortionType, Distortion>>& data) override
  {
    this->cache->timed.distortion().push_back(data);
    for (const auto& listener : this->listeners)
      listener->processDistortion(data);
  }
  void processAzimuth(const TimedMetadata<compass_msgs::Azimuth>& data) override
  {
    this->cache->timed.azimuth().push_back(data);
    for (const auto& listener : this->listeners)
      listener->processAzimuth(data);
  }
  void processMagneticField(const TimedMetadata<sensor_msgs::MagneticField>& data) override
  {
    this->cache->timed.magneticField().push_back(data);
    for (const auto& listener : this->listeners)
      listener->processMagneticField(data);
  }
  void processRollPitch(const TimedMetadata<RollPitch>& data) override
  {
    this->cache->timed.rollPitch().push_back(data);
    for (const auto& listener : this->listeners)
      listener->processRollPitch(data);
  }
  void processAcceleration(const TimedMetadata<geometry_msgs::Vector3>& data) override
  {
    this->cache->timed.acceleration().push_back(data);
    for (const auto& listener : this->listeners)
      listener->processAcceleration(data);
  }
  void processAngularVelocity(const TimedMetadata<geometry_msgs::Vector3>& data) override
  {
    this->cache->timed.angularVelocity().push_back(data);
    for (const auto& listener : this->listeners)
      listener->processAngularVelocity(data);
  }
  void processFaces(const TimedMetadata<vision_msgs::Detection2DArray>& data) override
  {
    this->cache->timed.faces().push_back(data);
    for (const auto& listener : this->listeners)
      listener->processFaces(data);
  }
  void processCameraInfo(const TimedMetadata<sensor_msgs::CameraInfo>& data) override
  {
    this->cache->timed.cameraInfo().push_back(data);
    for (const auto& listener : this->listeners)
      listener->processCameraInfo(data);
  }
  void processImu(const TimedMetadata<sensor_msgs::Imu>& data) override
  {
    this->cache->timed.imu().push_back(data);
    for (const auto& listener : this->listeners)
      listener->processImu(data);
  }
  void processOpticalFrameTF(const TimedMetadata<geometry_msgs::Transform>& data) override
  {
    this->cache->timed.opticalFrameTF().push_back(data);
    for (const auto& listener : this->listeners)
      listener->processOpticalFrameTF(data);
  }
  void processGNSSPosition(const TimedMetadata<GNSSFixAndDetail>& data) override
  {
    this->cache->timed.gnssPosition().push_back(data);
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
  ONLY_CHECK_EXTRACTORS(getCameraGeneralName);
}

cras::optional<std::string> MetadataManager::getCameraUniqueName()
{
  ONLY_CHECK_EXTRACTORS(getCameraUniqueName);
}

cras::optional<std::string> MetadataManager::getCameraSerialNumber()
{
  ONLY_CHECK_EXTRACTORS(getCameraSerialNumber);
}

cras::optional<std::string> MetadataManager::getCameraMake()
{
  ONLY_CHECK_EXTRACTORS(getCameraMake);
}

cras::optional<std::string> MetadataManager::getCameraModel()
{
  ONLY_CHECK_EXTRACTORS(getCameraModel);
}

cras::optional<std::string> MetadataManager::getLensMake()
{
  ONLY_CHECK_EXTRACTORS(getLensMake);
}

cras::optional<std::string> MetadataManager::getLensModel()
{
  ONLY_CHECK_EXTRACTORS(getLensModel);
}

cras::optional<int> MetadataManager::getRotation()
{
  ONLY_CHECK_EXTRACTORS(getRotation);
}

cras::optional<ros::Time> MetadataManager::getCreationTime()
{
  ONLY_CHECK_EXTRACTORS(getCreationTime);
}

cras::optional<double> MetadataManager::getCropFactor()
{
  ONLY_CHECK_EXTRACTORS(getCropFactor);
}

cras::optional<SensorSize> MetadataManager::getSensorSizeMM()
{
  ONLY_CHECK_EXTRACTORS(getSensorSizeMM)
}

cras::optional<double> MetadataManager::getFocalLength35MM()
{
  ONLY_CHECK_EXTRACTORS(getFocalLength35MM)
}
cras::optional<double> MetadataManager::getFocalLengthPx()
{
  ONLY_CHECK_EXTRACTORS(getFocalLengthPx)
}

cras::optional<double> MetadataManager::getFocalLengthMM()
{
  ONLY_CHECK_EXTRACTORS(getFocalLengthMM);
}

cras::optional<IntrinsicMatrix> MetadataManager::getIntrinsicMatrix()
{
  ONLY_CHECK_EXTRACTORS(getIntrinsicMatrix);
}

cras::optional<std::pair<DistortionType, Distortion>> MetadataManager::getDistortion()
{
  ONLY_CHECK_EXTRACTORS(getDistortion);
}

GNSSFixAndDetail MetadataManager::getGNSSPosition()
{
  CHECK_CACHE(getGNSSPosition)
  StackGuard g(this->callStack, __func__, this);

  std::pair<cras::optional<sensor_msgs::NavSatFix>, cras::optional<gps_common::GPSFix>> result;
  for (const auto& extractor : this->extractors)
  {
    const auto& [navMsg, gpsMsg] = extractor->getGNSSPosition();
    if (!result.first.has_value() && navMsg.has_value())
      result.first = navMsg;
    if (!result.second.has_value() && gpsMsg.has_value())
      result.second = gpsMsg;
    if (result.first.has_value() && result.second.has_value())
      break;
  }
  return this->cache->latest.getGNSSPosition().emplace(result);
}

cras::optional<sensor_msgs::MagneticField> MetadataManager::getMagneticField()
{
  ONLY_CHECK_EXTRACTORS(getMagneticField);
}

cras::optional<compass_msgs::Azimuth> MetadataManager::getAzimuth()
{
  // TODO(peci1) compute from magnetic field and roll/pitch
  ONLY_CHECK_EXTRACTORS(getAzimuth);
}

cras::optional<RollPitch> MetadataManager::getRollPitch()
{
  ONLY_CHECK_EXTRACTORS(getRollPitch);
}

cras::optional<geometry_msgs::Vector3> MetadataManager::getAngularVelocity()
{
  ONLY_CHECK_EXTRACTORS(getAngularVelocity);
}

cras::optional<geometry_msgs::Vector3> MetadataManager::getAcceleration()
{
  ONLY_CHECK_EXTRACTORS(getAcceleration);
}

cras::optional<vision_msgs::Detection2DArray> MetadataManager::getFaces()
{
  ONLY_CHECK_EXTRACTORS(getFaces);
}

cras::optional<sensor_msgs::CameraInfo> MetadataManager::getCameraInfo()
{
  ONLY_CHECK_EXTRACTORS(getCameraInfo)
}

cras::optional<sensor_msgs::Imu> MetadataManager::getImu()
{
  ONLY_CHECK_EXTRACTORS(getImu)
}

cras::optional<geometry_msgs::Transform> MetadataManager::getOpticalFrameTF()
{
  ONLY_CHECK_EXTRACTORS(getOpticalFrameTF)
}

cras::optional<geometry_msgs::Transform> MetadataManager::getZeroRollPitchTF()
{
  ONLY_CHECK_EXTRACTORS(getZeroRollPitchTF)
}
}
