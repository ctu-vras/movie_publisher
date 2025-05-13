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
#include <set>
#include <string>
#include <utility>

#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/optional.hpp>
#include <geometry_msgs/Transform.h>
#include <movie_publisher/metadata_cache.h>
#include <pluginlib/class_loader.hpp>
#include <ros/time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <vision_msgs/Detection2DArray.h>

namespace movie_publisher
{

class StackGuard;
struct CachingMetadataListener;

struct PriorityComparator
{
  bool operator()(const MetadataExtractor::ConstPtr& lhs, const MetadataExtractor::ConstPtr& rhs) const
  {
    if (lhs == nullptr)
      return true;
    if (rhs == nullptr)
      return false;
    return lhs->getPriority() < rhs->getPriority();
  }
};

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
 *
 * The manager also processes timed metadata. When timed metadata are advanced using processTimedMetadata(), the cached
 * return values of some of the get*() functions will get recomputed so that these functions always return metadata that
 * are closest to the current playback state.
 */
class MetadataManager : public TimedMetadataExtractor
{
public:
  /**
   * \brief Constructor. Each movie should have its own manager.
   * \param[in] log Logger.
   * \param[in] width Width of the parsed movie.
   * \param[in] height Height of the parsed movie.
   */
  MetadataManager(const cras::LogHelperPtr& log, const MovieOpenConfig& config, const MovieInfo::ConstPtr& info);
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

  /**
   * \brief Return the metadata cache.
   * \return The metadata cache.
   */
  std::shared_ptr<MetadataCache> getCache();

  /**
   * \brief Clear all cached timed metadata.
   */
  void clearTimedMetadataCache();

  void prepareTimedMetadata(const std::unordered_set<MetadataType>& metadataTypes) override;
  std::unordered_set<MetadataType> supportedTimedMetadata(
    const std::unordered_set<MetadataType>& availableMetadata) const override;
  size_t processTimedMetadata(MetadataType type, const StreamTime& maxTime, bool requireOptional) override;
  void seekTimedMetadata(const StreamTime& seekTime) override;
  bool hasTimedMetadata() const override;
  void processPacket(const AVPacket* packet) override;

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
  cras::optional<SensorSize> getSensorSizeMM() override;
  cras::optional<double> getFocalLength35MM() override;
  cras::optional<double> getFocalLengthPx() override;
  cras::optional<double> getFocalLengthMM() override;
  cras::optional<IntrinsicMatrix> getIntrinsicMatrix() override;
  cras::optional<std::pair<DistortionType, Distortion>> getDistortion() override;
  GNSSFixAndDetail getGNSSPosition() override;
  cras::optional<sensor_msgs::MagneticField> getMagneticField() override;
  cras::optional<compass_msgs::Azimuth> getAzimuth() override;
  cras::optional<RollPitch> getRollPitch() override;
  cras::optional<geometry_msgs::Vector3> getAngularVelocity() override;
  cras::optional<geometry_msgs::Vector3> getAcceleration() override;
  cras::optional<vision_msgs::Detection2DArray> getFaces() override;
  cras::optional<sensor_msgs::CameraInfo> getCameraInfo() override;
  cras::optional<sensor_msgs::Imu> getImu() override;
  cras::optional<geometry_msgs::Transform> getOpticalFrameTF() override;
  cras::optional<geometry_msgs::Transform> getZeroRollPitchTF() override;

protected:
  /**
   * \brief Check for infinite recursion when individual extractors call common methods via this manager.
   * \param[in] fn The stringified name of the called function.
   * \param[in] extractor The extractor that calls the function.
   * \return Whether this extractor would be calling itself recursively if the given function were called.
   */
  bool stopRecursion(const std::string& fn, const MetadataExtractor* extractor) const;

  pluginlib::ClassLoader<MetadataExtractorPlugin> loader;  //!< The extractor plugin loader.
  std::multiset<MetadataExtractor::Ptr, PriorityComparator> extractors;  //!< Registered extractor instances.
  //! Registered timed extractor instances.
  std::multiset<TimedMetadataExtractor::Ptr, PriorityComparator> timedExtractors;
  std::deque<std::pair<std::string, const MetadataExtractor*>> callStack;  //!< The stack of all calls via the manager.
  size_t width {0u};  //!< Width of the analyzed movie [px].
  size_t height {0u};  //!< Height of the analyzed movie [px].

  MovieOpenConfig config;  //!< Configuration of the open movie.
  MovieInfo::ConstPtr info;  //!< Information about the open movie.
  std::shared_ptr<MetadataCache> cache;  //!< Cache of static and timed metadata.

  //! The timed metadata listener proxy passed to all timed extractors to collect and cache their output.
  std::shared_ptr<CachingMetadataListener> metadataListener;

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
