// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Composer of optical TF frame.
 * \author Martin Pecka
 */

#include <cras_cpp_common/type_utils.hpp>
#include <movie_publisher/metadata_manager.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace movie_publisher
{

class OpticalFrameTFComposer : public TimedMetadataExtractor
{
public:
  explicit OpticalFrameTFComposer(const cras::LogHelperPtr& log, const std::weak_ptr<MetadataManager>& manager,
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
    if (availableMetadata.find(MetadataType::ROTATION) != availableMetadata.end())
      return {MetadataType::OPTICAL_FRAME_TF};
    return {};
  }

  geometry_msgs::Transform compose(const int rotation) const
  {
    geometry_msgs::Transform msg;

    switch (rotation)
    {
    case 0:
      msg.rotation.x = msg.rotation.z = -0.5;
      msg.rotation.y = msg.rotation.w = 0.5;
      break;
    case 90:
      msg.rotation.x = msg.rotation.z = M_SQRT1_2;
      msg.rotation.y = msg.rotation.w = 0;
      break;
    case 180:
      msg.rotation.x = msg.rotation.z = 0.5;
      msg.rotation.y = msg.rotation.w = 0.5;
      break;
    case 270:
      msg.rotation.x = msg.rotation.z = 0;
      msg.rotation.y = msg.rotation.w = M_SQRT1_2;
      break;
    default:
      CRAS_WARN_ONCE("Invalid rotation: %i", rotation);
      break;
    }

    return msg;
  }

  cras::optional<geometry_msgs::Transform> getOpticalFrameTF() override
  {
    const auto manager = this->manager.lock();
    if (manager == nullptr)
      return cras::nullopt;

    const auto maybeRotation = manager->getRotation();
    if (!maybeRotation.has_value())
      return cras::nullopt;
    return this->compose(*maybeRotation);
  }

  size_t processTimedMetadata(const MetadataType type, const StreamTime& maxTime, const bool requireOptional) override
  {
    const auto& rotations = this->cache->timed.rotation();

    if (rotations.empty())
      return 0;

    for (const auto& rotation : rotations)
    {
      TimedMetadata<geometry_msgs::Transform> msg;
      msg.stamp = rotation.stamp;
      msg.value = this->compose(rotation.value);

      for (const auto& listener : this->listeners)
        listener->processOpticalFrameTF(msg);

      if (rotation.stamp == rotations.back().stamp)
        this->cache->latest.getOpticalFrameTF().emplace(msg.value);
    }
    return rotations.size();
  }

private:
  std::shared_ptr<MetadataCache> cache;
  std::weak_ptr<MetadataManager> manager;
};

/**
 * \brief Loader plugin for OpticalFrameTFComposer.
 */
struct OpticalFrameTFComposerPlugin : MetadataExtractorPlugin
{
  MetadataExtractor::Ptr getExtractor(const MetadataExtractorParams& params) override
  {
    if (params.log == nullptr || params.manager.lock() == nullptr || params.cache == nullptr)
      return nullptr;

    return std::make_shared<OpticalFrameTFComposer>(params.log, params.manager, params.cache);
  }
};

}

PLUGINLIB_EXPORT_CLASS(movie_publisher::OpticalFrameTFComposerPlugin, movie_publisher::MetadataExtractorPlugin)
