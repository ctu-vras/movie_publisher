// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Composer of roll/pitch angles.
 * \author Martin Pecka
 */

#include <Eigen/Geometry>

#include <cras_cpp_common/tf2_utils.hpp>
#include <cras_cpp_common/type_utils.hpp>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <movie_publisher/metadata_manager.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace movie_publisher
{

class RollPitchComposer : public TimedMetadataExtractor
{
public:
  explicit RollPitchComposer(const cras::LogHelperPtr& log, const std::weak_ptr<MetadataManager>& manager,
    const std::shared_ptr<MetadataCache>& cache) : TimedMetadataExtractor(log), cache(cache), manager(manager)
  {
  }

  std::string getName() const override
  {
    return cras::getTypeName<std::remove_cv_t<std::remove_reference_t<decltype(*this)>>>();
  }

  int getPriority() const override
  {
    return 100;
  }

  bool hasTimedMetadata() const override
  {
    return true;
  }

  std::unordered_set<MetadataType> supportedTimedMetadata(
    const std::unordered_set<MetadataType>& availableMetadata) const override
  {
    if (availableMetadata.find(MetadataType::ACCELERATION) != availableMetadata.end())
      return {MetadataType::ROLL_PITCH};
    return {};
  }

  cras::optional<RollPitch> compose(const geometry_msgs::Vector3& acceleration) const
  {
    // Compute the rotation between measured gravity vector and steady-state gravity vector
    Eigen::Vector3d v1(acceleration.x, acceleration.y, acceleration.z);
    Eigen::Vector3d v2(0, 0, 9.81);

    // If the acceleration doesn't contain gravity, we cannot tell roll/pitch.
    if (v1.norm() < 9.8 / 2)
      return cras::nullopt;

    const auto q = Eigen::Quaterniond::FromTwoVectors(v1, v2);
    geometry_msgs::Quaternion quat;
    quat.x = q.x(); quat.y = q.y(); quat.z = q.z(); quat.w = q.w();

    double roll, pitch, yaw;
    cras::getRPY(quat, roll, pitch, yaw);

    return std::make_pair(roll, pitch);
  }

  cras::optional<RollPitch> getRollPitch() override
  {
    const auto manager = this->manager.lock();
    if (manager == nullptr)
      return cras::nullopt;

    const auto acceleration = manager->getAcceleration();
    if (!acceleration.has_value())
      return cras::nullopt;
    const auto result = this->compose(*acceleration);
    if (result.has_value())
      CRAS_DEBUG_NAMED("metadata_composer", "Orientation computed from acceleration.");
    return result;
  }

  size_t processTimedMetadata(const MetadataType type, const StreamTime& maxTime, const bool requireOptional) override
  {
    if (type != MetadataType::ROLL_PITCH)
      return 0;

    const auto& acceleration = this->cache->timed.acceleration();
    if (acceleration.empty())
      return 0;

    size_t numProcessed {0u};
    for (const auto& accel : acceleration)
    {
      TimedMetadata<RollPitch> msg;
      msg.stamp = accel.stamp;
      const auto maybeRollPitch = this->compose(accel.value);
      if (!maybeRollPitch.has_value())
        continue;

      msg.value = *maybeRollPitch;
      numProcessed++;

      for (const auto& listener : this->listeners)
        listener->processRollPitch(msg);

      if (accel.stamp == acceleration.back().stamp)
        this->cache->latest.getRollPitch().emplace(msg.value);
    }

    return numProcessed;
  }

private:
  std::shared_ptr<MetadataCache> cache;
  std::weak_ptr<MetadataManager> manager;
};

/**
 * \brief Loader plugin for RollPitchComposer.
 */
struct RollPitchComposerPlugin : MetadataExtractorPlugin
{
  MetadataExtractor::Ptr getExtractor(const MetadataExtractorParams& params) override
  {
    if (params.log == nullptr || params.manager.lock() == nullptr || params.cache == nullptr)
      return nullptr;

    return std::make_shared<RollPitchComposer>(params.log, params.manager, params.cache);
  }
};

}

PLUGINLIB_EXPORT_CLASS(movie_publisher::RollPitchComposerPlugin, movie_publisher::MetadataExtractorPlugin)
