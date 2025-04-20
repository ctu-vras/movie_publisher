// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Manager of multiple image metadata providers which can cooperate in parsing.
 * \author Martin Pecka
 */

#include <string>
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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
      return val.value(); \
    }\
  }

/**
 * \brief Last statement. Call when extracting data from all extractors failed and nothing is cached.
 * \param[in] getFn The function name.
 */
#define FINISH(getFn) \
  return this->getFn() = cras::nullopt;

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

MetadataManager::MetadataManager(const cras::LogHelperPtr& log, const MovieOpenConfig& config, const MovieInfo& info) :
  TimedMetadataExtractor(log),
  loader("movie_publisher", "movie_publisher::MetadataExtractorPlugin", "metadata_plugins"),
  config(config), info(info), width(info.width()), height(info.height()), cache(new MetadataCache())
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

/**
 * \brief Compare TimedMetadata according to their stamps.
 * \tparam M Type of the metadata.
 * \param[in] a The timed metadata to compare.
 * \param[in] b The timestamp to compare with.
 * \return Whether a is greater than b.
 */
template<typename M>
bool CompareStamp(const TimedMetadata<M>& a, const StreamTime& b)
{
  return a.stamp > b;
}

/**
 * \brief Find index of the latest data from `data` that have their timestamp less than or equal to `stamp`.
 * \tparam M Metadata type.
 * \param[in] data A stamp-ordered list of metadata.
 * \param[in] stamp The maximum timestamp.
 * \return Index of the latest data up to stamp.
 */
template<typename M>
auto findLastUpToStamp(const std::vector<TimedMetadata<M>>& data, const StreamTime& stamp)
{
  return std::lower_bound(data.crbegin(), data.crend(), stamp, &CompareStamp<M>);
}

/**
 * \brief Find the latest data from `data` that have their timestamp less than or equal to `stamp`.
 * \tparam M Metadata type.
 * \param[in] data A stamp-ordered list of metadata.
 * \param[in] stamp The maximum timestamp.
 * \param[in] defaultVal The default value to return in case no value was found in `data`.
 * \return Index of the latest data up to stamp.
 */
template<typename M>
cras::optional<TimedMetadata<M>> findLastUpToStamp(const std::vector<TimedMetadata<M>>& data, const StreamTime& stamp,
  const cras::optional<cras::optional<M>>& defaultVal)
{
  const auto it = findLastUpToStamp(data, stamp);
  if (it != data.crend())
    return *it;
  if (defaultVal.has_value() && defaultVal->has_value())
    return TimedMetadata<M>{StreamTime{}, defaultVal->value()};
  return cras::nullopt;
}

class MetadataComposer : public TimedMetadataExtractor
{
public:
  explicit MetadataComposer(const cras::LogHelperPtr& log, const MovieOpenConfig& config, const MovieInfo& info,
    const std::unordered_map<TimedMetadataType, int>& metadata, const std::shared_ptr<MetadataCache>& cache)
    : TimedMetadataExtractor(log), config(config), info(info), baseSupportedMetadata(metadata), cache(cache)
  {
  }

  const std::unordered_map<TimedMetadataType, int>& supportedTimedMetadata() const override
  {
    return this->supportedMetadata;
  }

  bool hasTimedMetadata() const override
  {
    return !this->supportedMetadata.empty();
  }

protected:
  std::unordered_map<TimedMetadataType, int> baseSupportedMetadata;
  std::unordered_map<TimedMetadataType, int> supportedMetadata;
  MovieOpenConfig config;
  MovieInfo info;
  std::shared_ptr<MetadataCache> cache;
};

class CameraInfoComposer : public MetadataComposer
{
public:
  explicit CameraInfoComposer(const cras::LogHelperPtr& log, const MovieOpenConfig& config, const MovieInfo& info,
    const std::unordered_map<TimedMetadataType, int>& metadata, const std::shared_ptr<MetadataCache>& cache)
  : MetadataComposer(log, config, info, metadata, cache)
  {
    if (this->baseSupportedMetadata.find(TimedMetadataType::INTRINSIC_MATRIX) != this->baseSupportedMetadata.end())
    {
      this->supportedMetadata[TimedMetadataType::CAMERA_INFO] = CameraInfoComposer::getPriority();
    }
  }

  std::string getName() const override
  {
    return cras::getTypeName<std::remove_cv_t<std::remove_reference_t<decltype(*this)>>>();
  }

  int getPriority() const override
  {
    return 100;
  }

  void processTimedMetadata(const StreamTime& maxTime) override
  {
    const auto& intrinsicMatrix = this->cache->timed.intrinsicMatrix();
    const auto& rotation = this->cache->timed.rotation();
    const auto& distortion = this->cache->timed.distortion();

    if (intrinsicMatrix.empty())
      return;

    std::set<StreamTime> stamps;
    for (const auto& m : intrinsicMatrix)
      stamps.insert(m.stamp);
    for (const auto& m : rotation)
      stamps.insert(m.stamp);
    for (const auto& m : distortion)
      stamps.insert(m.stamp);

    for (const auto& stamp : stamps)
    {
      TimedMetadata<sensor_msgs::CameraInfo> msg;
      msg.stamp = stamp;
      msg.value.width = this->info.width();
      msg.value.height = this->info.height();

      const auto KIt = findLastUpToStamp(intrinsicMatrix, stamp, this->cache->latest.getIntrinsicMatrix());
      if (KIt.has_value())
        continue;;
      msg.value.K = KIt->value;

      const auto rotationIt = findLastUpToStamp(rotation, stamp, this->cache->latest.getRotation());
      if (rotationIt.has_value() && (rotationIt->value == 90 || rotationIt->value == 270))
        std::swap(msg.value.width, msg.value.height);

      msg.value.R[0 * 3 + 0] = msg.value.R[1 * 3 + 1] = msg.value.R[2 * 3 + 2] = 1;

      for (size_t row = 0; row < 3; ++row)
        std::copy_n(&msg.value.K[row * 3], 3, &msg.value.P[row * 4]);

      const auto distortionIt = findLastUpToStamp(distortion, stamp, this->cache->latest.getDistortion());
      if (distortionIt.has_value())
      {
        msg.value.distortion_model = distortionIt->value.first;
        msg.value.D = distortionIt->value.second;
      }

      for (const auto& listener : this->listeners)
        listener->processCameraInfo(msg);

      if (stamp == *stamps.crbegin())
        this->cache->latest.getCameraInfo().emplace(msg.value);
    }
  }
};

class ImuComposer : public MetadataComposer
{
public:
  explicit ImuComposer(const cras::LogHelperPtr& log, const MovieOpenConfig& config, const MovieInfo& info,
    const std::unordered_map<TimedMetadataType, int>& metadata, const std::shared_ptr<MetadataCache>& cache)
  : MetadataComposer(log, config, info, metadata, cache)
  {
    const auto& meta = this->baseSupportedMetadata;
    if (
      meta.find(TimedMetadataType::ACCELERATION) != meta.end() ||
      meta.find(TimedMetadataType::ANGULAR_VELOCITY) != meta.end() ||
      meta.find(TimedMetadataType::ROLL_PITCH) != meta.end() ||
      meta.find(TimedMetadataType::AZIMUTH) != meta.end()
    )
    {
      this->supportedMetadata[TimedMetadataType::IMU] = ImuComposer::getPriority();
    }
  }

  std::string getName() const override
  {
    return cras::getTypeName<std::remove_cv_t<std::remove_reference_t<decltype(*this)>>>();
  }

  int getPriority() const override
  {
    return 100;
  }

  void processTimedMetadata(const StreamTime& maxTime) override
  {
    const auto& rollPitch = this->cache->timed.rollPitch();
    const auto& acceleration = this->cache->timed.acceleration();
    const auto& angularVelocity = this->cache->timed.angularVelocity();
    const auto& azimuth = this->cache->timed.azimuth();

    if (rollPitch.empty() && acceleration.empty() && angularVelocity.empty() && azimuth.empty())
      return;

    std::set<StreamTime> stamps;
    for (const auto& m: rollPitch)
      stamps.insert(m.stamp);
    for (const auto& m: acceleration)
      stamps.insert(m.stamp);
    for (const auto& m: angularVelocity)
      stamps.insert(m.stamp);
    for (const auto& m: azimuth)
      stamps.insert(m.stamp);

    for (const auto& stamp: stamps)
    {
      TimedMetadata<sensor_msgs::Imu> msg;
      msg.stamp = stamp;

      if (const auto accelIt = findLastUpToStamp(acceleration, stamp, this->cache->latest.getAcceleration());
        accelIt.has_value())
      {
        msg.value.linear_acceleration_covariance = {0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1};
        msg.value.linear_acceleration = accelIt->value;
      }
      else
      {
        msg.value.linear_acceleration_covariance[0] = -1;
      }

      if (const auto angVelIt = findLastUpToStamp(angularVelocity, stamp, this->cache->latest.getAngularVelocity());
        angVelIt.has_value())
      {
        msg.value.angular_velocity_covariance = {0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1};
        msg.value.angular_velocity = angVelIt->value;
      }
      else
      {
        msg.value.angular_velocity_covariance[0] = -1;
      }

      if (!rollPitch.empty() || !azimuth.empty())
      {
        double roll{0.0};
        double pitch{0.0};
        double yaw{0.0};

        msg.value.orientation_covariance[0 * 3 + 0] = M_PI * M_PI;
        msg.value.orientation_covariance[1 * 3 + 1] = M_PI * M_PI;
        msg.value.orientation_covariance[2 * 3 + 2] = M_PI * M_PI;

        if (const auto rpIt = findLastUpToStamp(rollPitch, stamp, this->cache->latest.getRollPitch()); rpIt.has_value())
        {
          msg.value.orientation_covariance[0 * 3 + 0] = 0.1;
          msg.value.orientation_covariance[1 * 3 + 1] = 0.1;
          roll = rpIt->value.first;
          pitch = rpIt->value.first;
        }

        if (const auto azIt = findLastUpToStamp(azimuth, stamp, this->cache->latest.getAzimuth()); azIt.has_value())
        {
          const auto variance = azIt->value.variance != 0 ? azIt->value.variance : 0.1;
          msg.value.orientation_covariance[2 * 3 + 2] = variance;
          yaw = azIt->value.azimuth;
          if (azIt->value.unit == compass_msgs::Azimuth::UNIT_DEG)
            yaw *= M_PI / 180.0;
          if (azIt->value.orientation == compass_msgs::Azimuth::ORIENTATION_NED)
            yaw = M_PI_2 - yaw;
        }

        tf2::Quaternion quat;
        quat.setRPY(roll, pitch, yaw);
        msg.value.orientation = tf2::toMsg(quat);
      }
      else
      {
        msg.value.orientation.w = 1;
        msg.value.orientation_covariance[0] = -1;
      }
      for (const auto& listener: this->listeners)
        listener->processImu(msg);

      if (stamp == *stamps.crbegin())
        this->cache->latest.getImu().emplace(msg.value);
    }
  }
};

class OpticalFrameTFComposer : public MetadataComposer
{
public:
  explicit OpticalFrameTFComposer(const cras::LogHelperPtr& log, const MovieOpenConfig& config, const MovieInfo& info,
    const std::unordered_map<TimedMetadataType, int>& metadata, const std::shared_ptr<MetadataCache>& cache)
  : MetadataComposer(log, config, info, metadata, cache)
  {
    const auto& meta = this->baseSupportedMetadata;
    if (meta.find(TimedMetadataType::ROTATION) != meta.end())
    {
      this->supportedMetadata[TimedMetadataType::OPTICAL_FRAME_TF] = OpticalFrameTFComposer::getPriority();
    }
  }

  std::string getName() const override
  {
    return cras::getTypeName<std::remove_cv_t<std::remove_reference_t<decltype(*this)>>>();
  }

  int getPriority() const override
  {
    return 100;
  }

  void processTimedMetadata(const StreamTime& maxTime) override
  {
    const auto& rotations = this->cache->timed.rotation();

    if (rotations.empty())
      return;

    for (const auto& rotation : rotations)
    {
      TimedMetadata<geometry_msgs::Transform> msg;
      msg.stamp = rotation.stamp;
      switch (rotation.value)
      {
        case 0:
          msg.value.rotation.x = msg.value.rotation.z = -0.5;
          msg.value.rotation.y = msg.value.rotation.w = 0.5;
          break;
        case 90:
          msg.value.rotation.x = msg.value.rotation.z = M_SQRT1_2;
          msg.value.rotation.y = msg.value.rotation.w = 0;
          break;
        case 180:
          msg.value.rotation.x = msg.value.rotation.z = 0.5;
          msg.value.rotation.y = msg.value.rotation.w = 0.5;
          break;
        case 270:
          msg.value.rotation.x = msg.value.rotation.z = 0;
          msg.value.rotation.y = msg.value.rotation.w = M_SQRT1_2;
          break;
        default:
          continue;
      }

      for (const auto& listener : this->listeners)
        listener->processOpticalFrameTF(msg);

      if (rotation.stamp == rotations.back().stamp)
        this->cache->latest.getOpticalFrameTF().emplace(msg.value);
    }
  }
};

cras::optional<RollPitch> composeRollPitch(const cras::optional<geometry_msgs::Vector3>& acceleration)
{
  if (!acceleration.has_value())
    return cras::nullopt;

  tf2::Vector3 a;
  tf2::fromMsg(*acceleration, a);
  a.normalize();
  const auto normYZ = std::sqrt(a.y() * a.y() + a.z() * a.z());
  return std::make_pair(
    normYZ > 1e-5 ? std::atan2(-a.x(), normYZ) : (a.x() >= 0 ? -M_PI_2 : M_PI_2),
    std::abs(a.z()) > 1e-5 ? std::atan2(a.y(), a.z()) : (a.y() >= 0 ? -M_PI_2 : M_PI_2)
  );
}



class RollPitchComposer : public MetadataComposer
{
public:
  explicit RollPitchComposer(const cras::LogHelperPtr& log, const MovieOpenConfig& config, const MovieInfo& info,
    const std::unordered_map<TimedMetadataType, int>& metadata, const std::shared_ptr<MetadataCache>& cache)
  : MetadataComposer(log, config, info, metadata, cache)
  {
    const auto& meta = this->baseSupportedMetadata;
    if (meta.find(TimedMetadataType::ROTATION) != meta.end())
    {
      this->supportedMetadata[TimedMetadataType::ROLL_PITCH] = RollPitchComposer::getPriority();
    }
  }

  std::string getName() const override
  {
    return cras::getTypeName<std::remove_cv_t<std::remove_reference_t<decltype(*this)>>>();
  }

  int getPriority() const override
  {
    return 100;
  }

  void processTimedMetadata(const StreamTime& maxTime) override
  {
    const auto& acceleration = this->cache->timed.acceleration();
    if (acceleration.empty())
      return;

    std::vector<TimedMetadata<std::pair<double, double>>> msgs;
    for (const auto& accel : acceleration)
    {
      const auto rollPitch = composeRollPitch(accel.value);
      if (rollPitch.has_value())
        msgs.emplace_back(TimedMetadata<std::pair<double, double>>{accel.stamp, *rollPitch});
    }

    for (const auto& msg : msgs)
    {
      for (const auto& listener : this->listeners)
        listener->processRollPitch(msg);
    }
    this->cache->latest.getRollPitch().emplace(msgs.back().value);
  }
};

void MetadataManager::addExtractor(const std::shared_ptr<MetadataExtractor>& extractor)
{
  // Insert the extractor sorted by increasing priority
  const auto cmp = [](const std::shared_ptr<MetadataExtractor>& e1, const std::shared_ptr<MetadataExtractor>& e2)
  {
    return e1->getPriority() < e2->getPriority();
  };
  const auto insertPos = std::lower_bound(this->extractors.begin(), this->extractors.end(), extractor, cmp);
  this->extractors.insert(insertPos, extractor);

  const auto timedExtractor = std::dynamic_pointer_cast<TimedMetadataExtractor>(extractor);
  if (timedExtractor != nullptr)
  {
    // Insert our proxy timed metadata listener
    const auto timedInsertPos = std::lower_bound(
      this->timedExtractors.begin(), this->timedExtractors.end(), timedExtractor, cmp);
    this->timedExtractors.insert(timedInsertPos, timedExtractor);
    timedExtractor->addTimedMetadataListener(this->metadataListener);

    for (const auto& [metadataType, priority] : timedExtractor->supportedTimedMetadata())
    {
      auto& merged = this->mergedSupportedTimedMetadata;
      if (merged.find(metadataType) == merged.end() || merged[metadataType] >= priority)
      {
        merged[metadataType] = priority;
        this->timedMetadataExtractorPerType[metadataType] = timedExtractor;
      }
    }
  }

  CRAS_DEBUG_NAMED("metadata_plugins", "%s %s added to metadata manager.",
    timedExtractor != nullptr ? "Timed extractor" : "Extractor", extractor->getName().c_str());
}

void MetadataManager::loadExtractorPlugins(const MetadataExtractorParams& params)
{
  for (const auto& cl : this->loader.getDeclaredClasses())
  {
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

void MetadataManager::prepareTimedMetadata(const std::vector<TimedMetadataType>& types)
{
  auto& meta = this->mergedSupportedTimedMetadata;
  std::vector<std::shared_ptr<MetadataComposer>> composers = {
    std::make_shared<CameraInfoComposer>(this->log, this->config, this->info, meta, this->cache),
    std::make_shared<ImuComposer>(this->log, this->config, this->info, meta, this->cache),
    std::make_shared<OpticalFrameTFComposer>(this->log, this->config, this->info, meta, this->cache),
    std::make_shared<RollPitchComposer>(this->log, this->config, this->info, meta, this->cache),
  };

  for (const auto& composer : composers)
    this->addExtractor(composer);

  // Collect the best extractors for each type of metadata and call their prepareTimedMetadata() method.
  std::unordered_map<std::shared_ptr<TimedMetadataExtractor>, std::vector<TimedMetadataType>> timedTypes;
  for (const auto& [metadataType, extractor] : this->timedMetadataExtractorPerType)
  {
    if (types.empty() || std::find(types.begin(), types.end(), metadataType) != types.end())
      timedTypes[extractor].push_back(metadataType);
  }

  for (const auto& [extractor, metadata] : timedTypes)
    extractor->prepareTimedMetadata(metadata);
}

void MetadataManager::processTimedMetadata(const StreamTime& maxTime)
{
  for (const auto& extractor : this->timedExtractors)
    extractor->processTimedMetadata(maxTime);

  this->cache->timed.clear();
}

void MetadataManager::seekTimedMetadata(const StreamTime& seekTime)
{
  for (const auto& extractor : this->timedExtractors)
    extractor->seekTimedMetadata(seekTime);
}

void MetadataManager::processPacket(const AVPacket* packet)
{
  for (const auto& extractor : this->extractors)
    extractor->processPacket(packet);
}

const std::unordered_map<TimedMetadataType, int>& MetadataManager::supportedTimedMetadata() const
{
  return this->mergedSupportedTimedMetadata;
}

cras::optional<std::string> MetadataManager::getCameraGeneralName()
{
  CHECK_EXTRACTORS(getCameraGeneralName);

  const auto make = this->getCameraMake().value_or("");
  const auto model = this->getCameraModel().value_or("");
  const auto lensMake = this->getLensMake().value_or("");
  const auto lensModel = this->getLensModel().value_or("");

  if (!make.empty() || !model.empty() || !lensMake.empty() || !lensModel.empty())
  {
    const auto cameraName = cras::strip(cras::join<std::list<std::string>>({make, model}, " "));
    const auto lensName = cras::strip(cras::join<std::list<std::string>>({lensMake, lensModel}, " "));
    auto name = cras::strip(cras::join<std::list<std::string>>({cameraName, lensName}, " "));

    if (name.empty())
      return this->cache->latest.getCameraGeneralName().emplace(cras::nullopt);

    CRAS_DEBUG_NAMED("metadata_manager", "Camera name composed from make and model of the camera and lens.");
    return this->cache->latest.getCameraGeneralName().emplace(name);
  }

  FINISH(getCameraGeneralName)
}

cras::optional<std::string> MetadataManager::getCameraUniqueName()
{
  CHECK_EXTRACTORS(getCameraUniqueName);

  const auto serial = this->getCameraSerialNumber();
  if (serial.has_value() && !serial->empty())
  {
    const auto name = this->getCameraGeneralName().value_or("camera");
    CRAS_DEBUG_NAMED("metadata_manager", "Camera unique name has been composed from its general name and serial nr.");
    return cras::format("%s (%s)", name.c_str(), serial->c_str());
  }

  FINISH(getCameraUniqueName);
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
  CHECK_EXTRACTORS(getCropFactor);

  const auto focalLengthMM = this->getFocalLengthMM();
  const auto focalLength35MM = this->getFocalLength35MM();
  if (focalLengthMM.has_value() && focalLength35MM.has_value())
  {
    const auto cropFactor = *focalLength35MM / *focalLengthMM;
    CRAS_DEBUG_NAMED("metadata_manager",
      "Crop factor %.2f was determined from real and 35 mm focal lengths.", cropFactor);
    return this->cache->latest.getCropFactor().emplace(cropFactor);
  }

  FINISH(getCropFactor)
}

cras::optional<SensorSize> MetadataManager::getSensorSizeMM()
{
  CHECK_EXTRACTORS(getSensorSizeMM)

  const auto cropFactor = this->getCropFactor();
  if (cropFactor.has_value())
  {
    const auto& w = this->width;
    const auto& h = this->height;
    const auto sensorWidthMM = 36.0 / *cropFactor;
    const auto sensorHeightMM = sensorWidthMM * std::min(w, h) / std::max(w, h);
    CRAS_DEBUG_NAMED("metadata_manager",
      "Sensor size %.1fx%1.f mm was determined from crop factor.", sensorWidthMM, sensorHeightMM);
    return this->cache->latest.getSensorSizeMM().emplace(std::pair{sensorWidthMM, sensorHeightMM});
  }

  FINISH(getSensorSizeMM)
}

cras::optional<double> MetadataManager::getFocalLength35MM()
{
  CHECK_EXTRACTORS(getFocalLength35MM)

  const auto cropFactor = this->getCropFactor();
  const auto focalLength = this->getFocalLengthMM();

  if (cropFactor.has_value() && focalLength.has_value())
  {
    const auto f35mm = *focalLength * *cropFactor;
    CRAS_DEBUG_NAMED("metadata_manager",
      "Focal length %.1f mm (35 mm equiv) determined from crop factor and real focal length.", f35mm);
    return this->cache->latest.getFocalLength35MM().emplace(f35mm);
  }

  FINISH(getFocalLength35MM)
}
cras::optional<double> MetadataManager::getFocalLengthPx()
{
  CHECK_EXTRACTORS(getFocalLengthPx)

  const auto imageMaxSize = std::max(this->width, this->height);

  const auto focalLength35mm = this->getFocalLength35MM();
  if (focalLength35mm.has_value() && *focalLength35mm != 0)
  {
    const auto focalLengthPx = *focalLength35mm * imageMaxSize / 36.0;
    CRAS_DEBUG_NAMED("metadata_manager", "Focal length %.1f px determined from 35 mm focal length.", focalLengthPx);
    return this->cache->latest.getFocalLengthPx().emplace(focalLengthPx);
  }

  const auto sensorSizeMM = this->getSensorSizeMM();
  const auto focalLengthMM = this->getFocalLengthMM();
  if (sensorSizeMM.has_value() && focalLengthMM.has_value())
  {
    const auto sensorMaxSizeMM = std::max(sensorSizeMM->first, sensorSizeMM->second);
    const auto focalLengthPx = *focalLengthMM * imageMaxSize / sensorMaxSizeMM;
    CRAS_DEBUG_NAMED("metadata_manager",
      "Focal length %.1f px determined from real focal length and sensor size.", focalLengthPx);
    return this->cache->latest.getFocalLengthPx().emplace(focalLengthPx);
  }

  FINISH(getFocalLengthPx)
}

cras::optional<double> MetadataManager::getFocalLengthMM()
{
  CHECK_EXTRACTORS(getFocalLengthMM);

  const auto cropFactor = this->getCropFactor();
  const auto focalLength35MM = this->getFocalLength35MM();

  if (cropFactor.has_value() && focalLength35MM.has_value())
  {
    const auto f = *focalLength35MM / *cropFactor;
    CRAS_DEBUG_NAMED("metadata_manager",
      "Real focal length %.1f mm determined from crop factor and 35 mm focal length.", f);
    return this->cache->latest.getFocalLengthMM().emplace(f);
  }

  FINISH(getFocalLengthMM)
}

cras::optional<IntrinsicMatrix> MetadataManager::getIntrinsicMatrix()
{
  CHECK_EXTRACTORS(getIntrinsicMatrix);

  const auto focalLengthPx = this->getFocalLengthPx();
  if (focalLengthPx.has_value())
  {
    IntrinsicMatrix K{};
    K[0 * 3 + 0] = *focalLengthPx;
    K[1 * 3 + 1] = *focalLengthPx;
    K[0 * 3 + 2] = this->width / 2.0;
    K[1 * 3 + 2] = this->height / 2.0;
    K[2 * 3 + 2] = 1;

    const auto rotation = this->getRotation();
    if (rotation.has_value() && (*rotation == 90 || *rotation == 270))
      std::swap(K[0 * 3 + 2], K[1 * 3 + 2]);

    CRAS_DEBUG_NAMED("metadata_manager", "Camera intrinsics have been computed from pixel focal length.");
    return this->cache->latest.getIntrinsicMatrix().emplace(K);
  }

  FINISH(getIntrinsicMatrix)
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
  CHECK_EXTRACTORS(getRollPitch);

  const auto rollPitch = composeRollPitch(this->getAcceleration());
  if (rollPitch.has_value())
  {
    CRAS_DEBUG_NAMED("metadata_manager", "Orientation computed from acceleration.");
    return this->cache->latest.getRollPitch().emplace(rollPitch);
  }

  FINISH(getRollPitch)
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
  CHECK_CACHE(getCameraInfo)
  if (this->stopRecursion(__func__, this))
    return cras::nullopt;
  StackGuard g(this->callStack, __func__, this);

  const auto K = this->getIntrinsicMatrix();
  if (K.has_value())
  {
    sensor_msgs::CameraInfo msg;
    msg.width = this->width;
    msg.height = this->height;

    msg.K = *K;

    const auto rotation = this->getRotation();
    if (rotation == 90 || rotation == 270)
      std::swap(msg.width, msg.height);

    msg.R[0 * 3 + 0] = msg.R[1 * 3 + 1] = msg.R[2 * 3 + 2] = 1;

    for (size_t row = 0; row < 3; ++row)
      std::copy_n(&msg.K[row * 3], 3, &msg.P[row * 4]);

    const auto distortion = this->getDistortion();
    if (distortion.has_value())
    {
      msg.distortion_model = distortion->first;
      msg.D = distortion->second;
    }

    return this->cache->latest.getCameraInfo().emplace(msg);
  }

  FINISH(getCameraInfo)
}

cras::optional<sensor_msgs::Imu> MetadataManager::getImu()
{
  CHECK_CACHE(getImu)
  if (this->stopRecursion(__func__, this))
    return cras::nullopt;
  StackGuard g(this->callStack, __func__, this);

  const auto rollPitch = this->getRollPitch();
  const auto acceleration = this->getAcceleration();
  const auto angularVelocity = this->getAngularVelocity();
  const auto azimuth = this->getAzimuth();

  if (!rollPitch.has_value() && !acceleration.has_value() && !angularVelocity.has_value() && !azimuth.has_value())
    FINISH(getImu)

  sensor_msgs::Imu msg;
  if (acceleration.has_value())
  {
    msg.linear_acceleration_covariance = {0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1};
    msg.linear_acceleration = *acceleration;
  }
  else
  {
    msg.linear_acceleration_covariance[0] = -1;
  }

  if (angularVelocity.has_value())
  {
    msg.angular_velocity_covariance = {0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1};
    msg.angular_velocity = *angularVelocity;
  }
  else
  {
    msg.angular_velocity_covariance[0] = -1;
  }

  if (rollPitch.has_value() || azimuth.has_value())
  {
    double roll {0.0};
    double pitch {0.0};
    double yaw {0.0};

    msg.orientation_covariance[0 * 3 + 0] = M_PI * M_PI;
    msg.orientation_covariance[1 * 3 + 1] = M_PI * M_PI;
    msg.orientation_covariance[2 * 3 + 2] = M_PI * M_PI;

    if (rollPitch.has_value())
    {
      msg.orientation_covariance[0 * 3 + 0] = 0.1;
      msg.orientation_covariance[1 * 3 + 1] = 0.1;
      roll = rollPitch->first;
      pitch = rollPitch->second;
    }

    if (azimuth.has_value())
    {
      const auto variance = azimuth->variance != 0 ? azimuth->variance : 0.1;
      msg.orientation_covariance[2 * 3 + 2] = variance;
      yaw = azimuth->azimuth;
      if (azimuth->unit == compass_msgs::Azimuth::UNIT_DEG)
        yaw *= M_PI / 180.0;
      if (azimuth->orientation == compass_msgs::Azimuth::ORIENTATION_NED)
        yaw = M_PI_2 - yaw;
    }

    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    msg.orientation = tf2::toMsg(quat);
  }
  else
  {
    msg.orientation.w = 1;
    msg.orientation_covariance[0] = -1;
  }

  return this->cache->latest.getImu().emplace(msg);
}

cras::optional<geometry_msgs::Transform> MetadataManager::getOpticalFrameTF()
{
  CHECK_CACHE(getOpticalFrameTF)
  if (this->stopRecursion(__func__, this))
    return cras::nullopt;
  StackGuard g(this->callStack, __func__, this);

  const auto rotation = this->getRotation();
  if (!rotation.has_value())
    return this->cache->latest.getOpticalFrameTF().emplace(cras::nullopt);

  geometry_msgs::Transform result;
  switch (*rotation)
  {
    case 0:
      result.rotation.x = result.rotation.z = -0.5;
      result.rotation.y = result.rotation.w = 0.5;
      break;
    case 90:
      result.rotation.x = result.rotation.z = M_SQRT1_2;
      result.rotation.y = result.rotation.w = 0;
      break;
    case 180:
      result.rotation.x = result.rotation.z = 0.5;
      result.rotation.y = result.rotation.w = 0.5;
      break;
    case 270:
      result.rotation.x = result.rotation.z = 0;
      result.rotation.y = result.rotation.w = M_SQRT1_2;
      break;
    default:
      return this->cache->latest.getOpticalFrameTF().emplace(cras::nullopt);
  }

  return this->cache->latest.getOpticalFrameTF().emplace(result);
}

}
