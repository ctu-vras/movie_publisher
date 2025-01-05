// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Manager of multiple image metadata providers which can cooperate in parsing.
 * \author Martin Pecka
 */

#pragma once

#include "metadata_extractor.h"

#include <deque>
#include <memory>
#include <string>
#include <utility>

#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/optional.hpp>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <pluginlib/class_loader.hpp>
#include <ros/time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

namespace movie_publisher
{

class StackGuard;

/**
 * \brief Manager of multiple image metadata providers which can cooperate in parsing.
 *
 * The manager itself behaves like a metadata extractor, but it doesn't extract much itself. Most of the work is relayed
 * to the loaded extractor plugins. The manager takes the first valid result and caches it, so that subsequent calls
 * for the same data will just re-use the cache.
 *
 * If you pass the manager to the extractor plugins, make sure you pass a std::weak_ptr and not normal std::shared_ptr.
 * The weak pointer will break reference cycle which would otherwise be inevitable (because the manager holds pointers
 * to the extractors).
 */
class MetadataManager : public MetadataExtractor
{
public:
  /**
   * \brief Constructor. Each movie should have its own manager.
   * \param[in] log Logger.
   * \param[in] width Width of the parsed movie.
   * \param[in] height Height of the parsed movie.
   */
  explicit MetadataManager(const cras::LogHelperPtr& log, size_t width, size_t height);
  ~MetadataManager() override;

  /**
   * \brief Register a new extractor.
   * \param[in] extractor The new extractor to register.
   */
  void addExtractor(const std::shared_ptr<MetadataExtractor>& extractor);

  /**
   * \brief Load all known extractors from plugins.
   * \param[in] params Parameters passed to the plugin initialization methods.
   */
  void loadExtractorPlugins(const MetadataExtractorParams& params);

  std::string getName() const override;
  int getPriority() const override;
  cras::optional<std::string> getCameraGeneralName() override;
  cras::optional<std::string> getCameraUniqueName() override;
  cras::optional<std::string> getCameraSerialNumber() override;
  cras::optional<std::string> getCameraMake() override;
  cras::optional<std::string> getCameraModel() override;
  cras::optional<std::string> getLensMake() override;
  cras::optional<std::string> getLensModel() override;
  cras::optional<int> getRotation() override;
  cras::optional<ros::Time> getCreationTime() override;
  cras::optional<double> getCropFactor() override;
  cras::optional<std::pair<double, double>> getSensorSizeMM() override;
  cras::optional<double> getFocalLength35MM() override;
  cras::optional<double> getFocalLengthPx() override;
  cras::optional<double> getFocalLengthMM() override;
  cras::optional<CI::_K_type> getIntrinsicMatrix() override;
  cras::optional<std::pair<CI::_distortion_model_type, CI::_D_type>> getDistortion() override;
  std::pair<cras::optional<sensor_msgs::NavSatFix>, cras::optional<gps_common::GPSFix>> getGNSSPosition() override;
  cras::optional<compass_msgs::Azimuth> getAzimuth() override;
  cras::optional<std::pair<double, double>> getRollPitch() override;
  cras::optional<geometry_msgs::Vector3> getAcceleration() override;

  /**
   * \brief Extract as much camera info as possible.
   * \return Extracted camera info, or nothing.
   */
  virtual cras::optional<sensor_msgs::CameraInfo> getCameraInfo();

  /**
   * \brief Extract as much IMU information as possible.
   * \return Extracted IMU info, or nothing.
   */
  virtual cras::optional<sensor_msgs::Imu> getImu();

  /**
   * \brief Extract gravity-aligned roll and pitch.
   * \return Extracted orientation, or nothing.
   */
  virtual cras::optional<geometry_msgs::Quaternion> getRollPitchOrientation();

  /**
   * \brief Get the optical frame transform (might be affected by image rotation).
   * \return Transform between camera's geometrical and optical frame.
   */
  virtual cras::optional<geometry_msgs::Transform> getOpticalFrameTF();

protected:
  /**
   * \brief Check for infinite recursion when individual extractors call common methods via this manager.
   * \param[in] fn The stringified name of the called function.
   * \param[in] extractor The extractor that calls the function.
   * \return Whether this extractor would be calling itself recursively if the given function were called.
   */
  bool stopRecursion(const std::string& fn, const MetadataExtractor* extractor) const;

  pluginlib::ClassLoader<MetadataExtractorPlugin> loader;  //!< The extractor plugin loader.
  std::list<std::shared_ptr<MetadataExtractor>> extractors;  //!< Registered extractor instances.
  std::deque<std::pair<std::string, const MetadataExtractor*>> callStack;  //!< The stack of all calls via the manager.
  size_t width {0u};  //!< Width of the analyzed movie [px].
  size_t height {0u};  //!< Height of the analyzed movie [px].

  cras::optional<cras::optional<std::string>> getCameraGeneralNameResult;  //!< Cached result.
  cras::optional<cras::optional<std::string>> getCameraUniqueNameResult;  //!< Cached result.
  cras::optional<cras::optional<std::string>> getCameraSerialNumberResult;  //!< Cached result.
  cras::optional<cras::optional<std::string>> getCameraMakeResult;  //!< Cached result.
  cras::optional<cras::optional<std::string>> getCameraModelResult;  //!< Cached result.
  cras::optional<cras::optional<std::string>> getLensMakeResult;  //!< Cached result.
  cras::optional<cras::optional<std::string>> getLensModelResult;  //!< Cached result.
  cras::optional<cras::optional<int>> getRotationResult;  //!< Cached result.
  cras::optional<cras::optional<ros::Time>> getCreationTimeResult;  //!< Cached result.
  cras::optional<cras::optional<double>> getCropFactorResult;  //!< Cached result.
  cras::optional<cras::optional<std::pair<double, double>>> getSensorSizeMMResult;  //!< Cached result.
  cras::optional<cras::optional<double>> getFocalLength35MMResult;  //!< Cached result.
  cras::optional<cras::optional<double>> getFocalLengthPxResult;  //!< Cached result.
  cras::optional<cras::optional<double>> getFocalLengthMMResult;  //!< Cached result.
  cras::optional<cras::optional<CI::_K_type>> getIntrinsicMatrixResult;  //!< Cached result.
  //! Cached result.
  cras::optional<cras::optional<std::pair<CI::_distortion_model_type, CI::_D_type>>> getDistortionResult;
  cras::optional<std::pair<cras::optional<sensor_msgs::NavSatFix>, cras::optional<gps_common::GPSFix>>>
    getGNSSPositionResult;  //!< Cached result.
  cras::optional<cras::optional<compass_msgs::Azimuth>> getAzimuthResult;  //!< Cached result.
  cras::optional<cras::optional<std::pair<double, double>>> getRollPitchResult;  //!< Cached result.
  cras::optional<cras::optional<geometry_msgs::Vector3>> getAccelerationResult;  //!< Cached result.
  cras::optional<cras::optional<sensor_msgs::CameraInfo>> getCameraInfoResult;  //!< Cached result.
  cras::optional<cras::optional<sensor_msgs::Imu>> getImuResult;  //!< Cached result.
  cras::optional<cras::optional<geometry_msgs::Quaternion>> getRollPitchOrientationResult;  //!< Cached result.
  cras::optional<cras::optional<geometry_msgs::Transform>> getOpticalFrameTFResult;  //!< Cached result.

  friend StackGuard;
};

/**
 * \brief RAII guard saving a function call to stack. Can be used to protect against infinite recursion.
 */
class StackGuard
{
public:
  /**
   * \brief Constructor.
   * \param[in] stack The stack where this guard should be recorded.
   * \param[in] fn The called function.
   * \param[in] extractor The extractor that calls the function.
   */
  StackGuard(decltype(MetadataManager::callStack)& stack, const std::string& fn, const MetadataExtractor* extractor);
  ~StackGuard();
private:
  std::string getStackDescription() const;  //!< Get a human-readable representation of the whole call stack.
  decltype(MetadataManager::callStack)& stack;  //!< The stack to operate on.
  std::string fn;  //!< The called function.
  const MetadataExtractor* extractor;  //!< The extractor that calls the function.
};
}
