// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Composer of zero roll/pitch TF frame.
 * \author Martin Pecka
 */

#include <cras_cpp_common/type_utils.hpp>
#include <movie_publisher/metadata_manager.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace movie_publisher
{

class ZeroRollPitchTFComposer : public TimedMetadataExtractor
{
public:
  explicit ZeroRollPitchTFComposer(const cras::LogHelperPtr& log, const std::weak_ptr<MetadataManager>& manager,
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
      return {MetadataType::ZERO_ROLL_PITCH_TF};
    return {};
  }

  geometry_msgs::Transform compose(const RollPitch& rollPitch) const
  {
    tf2::Quaternion quat;
    quat.setRPY(rollPitch.first, rollPitch.second, 0);
    geometry_msgs::Transform msg;
    msg.rotation = tf2::toMsg(quat.inverse());
    return msg;
  }

  cras::optional<geometry_msgs::Transform> getZeroRollPitchTF() override
  {
    const auto manager = this->manager.lock();
    if (manager == nullptr)
      return cras::nullopt;

    const auto rollPitch = manager->getRollPitch();
    if (!rollPitch.has_value())
      return cras::nullopt;
    return this->compose(*rollPitch);
  }

  size_t processTimedMetadata(const MetadataType type, const StreamTime& maxTime, const bool requireOptional) override
  {
    if (type != MetadataType::ZERO_ROLL_PITCH_TF)
      return 0;

    const auto& rollsPitches = this->cache->timed.rollPitch();
    if (rollsPitches.empty())
      return 0;

    for (const auto& [stamp, rollPitch] : rollsPitches)
    {
      TimedMetadata<geometry_msgs::Transform> msg;
      msg.stamp = stamp;
      msg.value = this->compose(rollPitch);

      for (const auto& listener : this->listeners)
        listener->processZeroRollPitchTF(msg);

      if (stamp == rollsPitches.back().stamp)
        this->cache->latest.getZeroRollPitchTF().emplace(msg.value);
    }
    return rollsPitches.size();
  }

private:
  std::shared_ptr<MetadataCache> cache;
  std::weak_ptr<MetadataManager> manager;
};

/**
 * \brief Loader plugin for ZeroRollPitchTFComposer.
 */
struct ZeroRollPitchTFComposerPlugin : MetadataExtractorPlugin
{
  MetadataExtractor::Ptr getExtractor(const MetadataExtractorParams& params) override
  {
    if (params.log == nullptr || params.manager.lock() == nullptr || params.cache == nullptr)
      return nullptr;

    return std::make_shared<ZeroRollPitchTFComposer>(params.log, params.manager, params.cache);
  }
};

}

PLUGINLIB_EXPORT_CLASS(movie_publisher::ZeroRollPitchTFComposerPlugin, movie_publisher::MetadataExtractorPlugin)
