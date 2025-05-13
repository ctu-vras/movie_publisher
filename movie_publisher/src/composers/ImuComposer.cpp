// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Composer of Imu messages.
 * \author Martin Pecka
 */

#include <cras_cpp_common/type_utils.hpp>
#include <movie_publisher/metadata_manager.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace movie_publisher
{

class ImuComposer : public TimedMetadataExtractor
{
public:
  explicit ImuComposer(const cras::LogHelperPtr& log, const std::weak_ptr<MetadataManager>& manager,
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
    if (availableMetadata.find(MetadataType::ROLL_PITCH) != availableMetadata.end())
      return {MetadataType::IMU};
    if (availableMetadata.find(MetadataType::ACCELERATION) != availableMetadata.end())
      return {MetadataType::IMU};
    if (availableMetadata.find(MetadataType::ANGULAR_VELOCITY) != availableMetadata.end())
      return {MetadataType::IMU};
    if (availableMetadata.find(MetadataType::AZIMUTH) != availableMetadata.end())
      return {MetadataType::IMU};
    return {};
  }

  cras::optional<sensor_msgs::Imu> compose(
    const cras::optional<RollPitch>& rollPitch, const cras::optional<geometry_msgs::Vector3>& acceleration,
    const cras::optional<geometry_msgs::Vector3>& angularVelocity, const cras::optional<compass_msgs::Azimuth>& azimuth)
    const
  {
    if (!rollPitch.has_value() && !acceleration.has_value() && !angularVelocity.has_value() && !azimuth.has_value())
      return cras::nullopt;

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
      double roll{0.0};
      double pitch{0.0};
      double yaw{0.0};

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
    return msg;
  }

  cras::optional<sensor_msgs::Imu> getImu() override
  {
    const auto manager = this->manager.lock();
    if (manager == nullptr)
      return cras::nullopt;
    const auto rollPitch = manager->getRollPitch();
    const auto acceleration = manager->getAcceleration();
    const auto angularVelocity = manager->getAngularVelocity();
    const auto azimuth = manager->getAzimuth();

    return this->compose(rollPitch, acceleration, angularVelocity, azimuth);
  }

  size_t processTimedMetadata(const MetadataType type, const StreamTime& maxTime, const bool requireOptional) override
  {
    if (type != MetadataType::IMU)
      return 0;

    const auto manager = this->manager.lock();
    if (manager == nullptr)
      return 0;

    const auto& rollPitch = this->cache->timed.rollPitch();
    const auto& acceleration = this->cache->timed.acceleration();
    const auto& angularVelocity = this->cache->timed.angularVelocity();
    const auto& azimuth = this->cache->timed.azimuth();

    if (rollPitch.empty() && acceleration.empty() && angularVelocity.empty() && azimuth.empty())
      return 0;
    if (requireOptional && (rollPitch.empty() || acceleration.empty() || angularVelocity.empty() || azimuth.empty()))
      return 0;

    std::set<StreamTime> stamps;
    for (const auto& m : rollPitch)
      stamps.insert(m.stamp);
    for (const auto& m : acceleration)
      stamps.insert(m.stamp);
    for (const auto& m : angularVelocity)
      stamps.insert(m.stamp);
    for (const auto& m : azimuth)
      stamps.insert(m.stamp);

    size_t numProcessed {0u};
    for (const auto& stamp : stamps)
    {
      TimedMetadata<sensor_msgs::Imu> msg;
      msg.stamp = stamp;

      const auto accelIt = findLastUpToStamp(acceleration, stamp, manager->getAcceleration());
      const auto angVelIt = findLastUpToStamp(angularVelocity, stamp, manager->getAngularVelocity());
      const auto rpIt = findLastUpToStamp(rollPitch, stamp, manager->getRollPitch());
      const auto azIt = findLastUpToStamp(azimuth, stamp, manager->getAzimuth());

      auto maybeImu = this->compose(
        rpIt.has_value() ? rpIt->value : cras::optional<RollPitch>{},
        accelIt.has_value() ? accelIt->value : cras::optional<geometry_msgs::Vector3>{},
        angVelIt.has_value() ? angVelIt->value : cras::optional<geometry_msgs::Vector3>{},
        azIt.has_value() ? azIt->value : cras::optional<compass_msgs::Azimuth>{});

      if (!maybeImu.has_value())
        continue;

      msg.value = *maybeImu;
      numProcessed++;

      for (const auto& listener : this->listeners)
        listener->processImu(msg);

      this->cache->latest.getImu().emplace(*maybeImu);
    }
    return numProcessed;
  }

private:
  std::shared_ptr<MetadataCache> cache;
  std::weak_ptr<MetadataManager> manager;
};

/**
 * \brief Loader plugin for ImuComposer.
 */
struct ImuComposerPlugin : MetadataExtractorPlugin
{
  MetadataExtractor::Ptr getExtractor(const MetadataExtractorParams& params) override
  {
    if (params.log == nullptr || params.manager.lock() == nullptr || params.cache == nullptr)
      return nullptr;

    return std::make_shared<ImuComposer>(params.log, params.manager, params.cache);
  }
};

}

PLUGINLIB_EXPORT_CLASS(movie_publisher::ImuComposerPlugin, movie_publisher::MetadataExtractorPlugin)
