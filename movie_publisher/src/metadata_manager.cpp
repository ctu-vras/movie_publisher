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
#include <movie_publisher/metadata_manager.h>
#include <ros/time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


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
  if (this->getFn##Result.has_value()) { \
    CHECK_CACHE_DEBUG_PRINT(getFn); \
    return this->getFn##Result.value(); \
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
      return this->getFn##Result.emplace(val); \
  }

/**
 * \brief Last statement. Call when extracting data from all extractors failed and nothing is cached.
 * \param[in] getFn The function name.
 */
#define FINISH(getFn) \
  return this->getFn##Result.emplace(cras::nullopt);

/**
 * \brief Call the requested function only on the registered extractors and cache, nothing more.
 * \param[in] getFn Name of the function.
 */
#define ONLY_CHECK_EXTRACTORS(getFn) \
  CHECK_EXTRACTORS(getFn) \
  FINISH(getFn)

MetadataManager::MetadataManager(const cras::LogHelperPtr& log, const size_t width, const size_t height) :
  MetadataExtractor(log), loader("movie_publisher", "movie_publisher::MetadataExtractorPlugin", "metadata_plugins"),
  width(width), height(height)
{
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
  // Insert the extractor sorted by increasing priority
  const auto cmp = [](const std::shared_ptr<MetadataExtractor>& e1, const std::shared_ptr<MetadataExtractor>& e2)
  {
    return e1->getPriority() < e2->getPriority();
  };
  const auto insertPos = std::lower_bound(this->extractors.begin(), this->extractors.end(), extractor, cmp);
  this->extractors.insert(insertPos, extractor);
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
      return this->getCameraGeneralNameResult.emplace(cras::nullopt);

    CRAS_DEBUG_NAMED("metadata_manager", "Camera name composed from make and model of the camera and lens.");
    return this->getCameraGeneralNameResult.emplace(name);
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
    return this->getCropFactorResult.emplace(cropFactor);
  }

  FINISH(getCropFactor)
}

cras::optional<std::pair<double, double>> MetadataManager::getSensorSizeMM()
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
    return this->getSensorSizeMMResult.emplace(std::pair{sensorWidthMM, sensorHeightMM});
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
    return this->getFocalLength35MMResult.emplace(f35mm);
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
    return this->getFocalLengthPxResult.emplace(focalLengthPx);
  }

  const auto sensorSizeMM = this->getSensorSizeMM();
  const auto focalLengthMM = this->getFocalLengthMM();
  if (sensorSizeMM.has_value() && focalLengthMM.has_value())
  {
    const auto sensorMaxSizeMM = std::max(sensorSizeMM->first, sensorSizeMM->second);
    const auto focalLengthPx = *focalLengthMM * imageMaxSize / sensorMaxSizeMM;
    CRAS_DEBUG_NAMED("metadata_manager",
      "Focal length %.1f px determined from real focal length and sensor size.", focalLengthPx);
    return this->getFocalLengthPxResult.emplace(focalLengthPx);
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
    return this->getFocalLengthMMResult.emplace(f);
  }

  FINISH(getFocalLengthMM)
}

cras::optional<CI::_K_type> MetadataManager::getIntrinsicMatrix()
{
  CHECK_EXTRACTORS(getIntrinsicMatrix);

  const auto focalLengthPx = this->getFocalLengthPx();
  if (focalLengthPx.has_value())
  {
    CI::_K_type K{};
    K[0 * 3 + 0] = *focalLengthPx;
    K[1 * 3 + 1] = *focalLengthPx;
    K[0 * 3 + 2] = this->width / 2.0;
    K[1 * 3 + 2] = this->height / 2.0;
    K[2 * 3 + 2] = 1;

    const auto rotation = this->getRotation();
    if (rotation.has_value() && (*rotation == 90 || *rotation == 270))
      std::swap(K[0 * 3 + 2], K[1 * 3 + 2]);

    CRAS_DEBUG_NAMED("metadata_manager", "Camera intrinsics have been computed from pixel focal length.");
    return this->getIntrinsicMatrixResult.emplace(K);
  }

  FINISH(getIntrinsicMatrix)
}

cras::optional<std::pair<CI::_distortion_model_type, CI::_D_type>> MetadataManager::getDistortion()
{
  ONLY_CHECK_EXTRACTORS(getDistortion);
}

std::pair<cras::optional<sensor_msgs::NavSatFix>, cras::optional<gps_common::GPSFix>> MetadataManager::getGNSSPosition()
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
  return this->getGNSSPositionResult.emplace(result);
}

cras::optional<compass_msgs::Azimuth> MetadataManager::getAzimuth()
{
  ONLY_CHECK_EXTRACTORS(getAzimuth);
}

cras::optional<std::pair<double, double>> MetadataManager::getRollPitch()
{
  ONLY_CHECK_EXTRACTORS(getRollPitch);
}

cras::optional<geometry_msgs::Vector3> MetadataManager::getAcceleration()
{
  ONLY_CHECK_EXTRACTORS(getAcceleration);
}

cras::optional<sensor_msgs::CameraInfo> MetadataManager::getCameraInfo()
{
  CHECK_CACHE(getCameraInfo)
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

    return this->getCameraInfoResult.emplace(msg);
  }

  FINISH(getCameraInfo)
}

cras::optional<sensor_msgs::Imu> MetadataManager::getImu()
{
  CHECK_CACHE(getImu)
  StackGuard g(this->callStack, __func__, this);

  const auto rollPitchOrientation = this->getRollPitchOrientation();
  const auto acceleration = this->getAcceleration();
  const auto azimuth = this->getAzimuth();

  if (!rollPitchOrientation.has_value() && !acceleration.has_value() && !azimuth.has_value())
    FINISH(getImu)

  sensor_msgs::Imu msg;
  msg.angular_velocity_covariance[0] = -1;

  if (acceleration.has_value())
  {
    msg.linear_acceleration_covariance = {0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1};
    msg.linear_acceleration = *acceleration;
  }
  else
  {
    msg.linear_acceleration_covariance[0] = -1;
  }

  if (rollPitchOrientation.has_value() || azimuth.has_value())
  {
    double roll {0.0};
    double pitch {0.0};
    double yaw {0.0};

    msg.orientation_covariance[0 * 3 + 0] = M_PI * M_PI;
    msg.orientation_covariance[1 * 3 + 1] = M_PI * M_PI;
    msg.orientation_covariance[2 * 3 + 2] = M_PI * M_PI;

    if (rollPitchOrientation.has_value())
    {
      msg.orientation_covariance[0 * 3 + 0] = 0.1;
      msg.orientation_covariance[1 * 3 + 1] = 0.1;
      tf2::Quaternion quat;
      tf2::fromMsg(*rollPitchOrientation, quat);
      double y;
      tf2::Matrix3x3(quat).getRPY(roll, pitch, y);
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

  return this->getImuResult.emplace(msg);
}

cras::optional<geometry_msgs::Quaternion> MetadataManager::getRollPitchOrientation()
{
  CHECK_CACHE(getRollPitchOrientation)
  StackGuard g(this->callStack, __func__, this);

  auto rollPitch = this->getRollPitch();
  if (rollPitch.has_value())
  {
    CRAS_DEBUG_NAMED("metadata_manager", "Orientation computed from roll and pitch.");
  }
  else
  {
    const auto accel = this->getAcceleration();
    if (accel.has_value())
    {
      tf2::Vector3 a;
      tf2::fromMsg(*accel, a);
      a.normalize();
      const auto normYZ = std::sqrt(a.y() * a.y() + a.z() * a.z());
      rollPitch = {
        normYZ > 1e-5 ? std::atan2(-a.x(), normYZ) : (a.x() >= 0 ? -M_PI_2 : M_PI_2),
        std::abs(a.z()) > 1e-5 ? std::atan2(a.y(), a.z()) : (a.y() >= 0 ? -M_PI_2 : M_PI_2)
      };
      CRAS_DEBUG_NAMED("metadata_manager", "Orientation computed from acceleration.");
    }
  }

  if (rollPitch.has_value())
  {
    tf2::Quaternion quat;
    quat.setRPY(rollPitch->first, rollPitch->second, 0);
    return this->getRollPitchOrientationResult.emplace(tf2::toMsg(quat));
  }

  FINISH(getRollPitchOrientation)
}

cras::optional<geometry_msgs::Transform> MetadataManager::getOpticalFrameTF()
{
  CHECK_CACHE(getOpticalFrameTF)
  StackGuard g(this->callStack, __func__, this);

  const auto rotation = this->getRotation();
  if (!rotation.has_value())
    return this->getOpticalFrameTFResult.emplace(cras::nullopt);

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
      return this->getOpticalFrameTFResult.emplace(cras::nullopt);
  }

  return this->getOpticalFrameTFResult.emplace(result);
}

}
