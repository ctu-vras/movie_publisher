// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Composer of CameraInfo messages.
 * \author Martin Pecka
 */

#include <cras_cpp_common/type_utils.hpp>
#include <movie_publisher/metadata_manager.h>
#include <pluginlib/class_list_macros.h>

namespace movie_publisher
{

class CameraInfoComposer : public TimedMetadataExtractor
{
public:
  explicit CameraInfoComposer(const cras::LogHelperPtr& log, const MovieInfo::ConstPtr& info,
    const std::weak_ptr<MetadataManager>& manager, const std::shared_ptr<MetadataCache>& cache)
  : TimedMetadataExtractor(log), info(info), cache(cache), manager(manager)
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
    if (availableMetadata.find(MetadataType::INTRINSIC_MATRIX) != availableMetadata.end())
      return {MetadataType::CAMERA_INFO};

    const auto manager = this->manager.lock();
    if (manager == nullptr)
      return {};

    const auto K = manager->getIntrinsicMatrix();
    if (K.has_value())
      return {MetadataType::CAMERA_INFO};

    return {};
  }

  sensor_msgs::CameraInfo compose(
    const IntrinsicMatrix& intrinsicMatrix, const cras::optional<int>& rotation,
    const cras::optional<DistortionData>& distortion) const
  {
    sensor_msgs::CameraInfo msg;
    msg.width = this->info->width();
    msg.height = this->info->height();

    msg.K = intrinsicMatrix;

    if (rotation.has_value() && (*rotation == 90 || *rotation == 270))
      std::swap(msg.width, msg.height);

    msg.R[0 * 3 + 0] = msg.R[1 * 3 + 1] = msg.R[2 * 3 + 2] = 1;

    for (size_t row = 0; row < 3; ++row)
      std::copy_n(&msg.K[row * 3], 3, &msg.P[row * 4]);

    if (distortion.has_value())
    {
      msg.distortion_model = distortion->first;
      msg.D = distortion->second;
    }

    return msg;
  }

  cras::optional<sensor_msgs::CameraInfo> getCameraInfo() override
  {
    const auto manager = this->manager.lock();
    if (manager == nullptr)
      return cras::nullopt;

    const auto K = manager->getIntrinsicMatrix();
    if (!K.has_value())
      return cras::nullopt;

    const auto rotation = manager->getRotation();
    const auto distortion = manager->getDistortion();

    return this->compose(*K, rotation, distortion);
  }

  size_t processTimedMetadata(const MetadataType type, const StreamTime& maxTime, const bool requireOptional) override
  {
    if (type != MetadataType::CAMERA_INFO)
      return 0;

    const auto manager = this->manager.lock();
    if (manager == nullptr)
      return false;

    const auto& intrinsicMatrix = this->cache->timed.intrinsicMatrix();
    const auto& rotation = this->cache->timed.rotation();
    const auto& distortion = this->cache->timed.distortion();

    if (intrinsicMatrix.empty() && !manager->getIntrinsicMatrix().has_value())
      return 0;
    if (requireOptional && rotation.empty() && !manager->getRotation().has_value())
      return 0;
    if (requireOptional && distortion.empty() && !manager->getDistortion().has_value())
      return 0;

    std::set<StreamTime> stamps {maxTime};  // always produce camera info for the given timestamp
    for (const auto& m : intrinsicMatrix)
      stamps.insert(m.stamp);
    for (const auto& m : rotation)
      stamps.insert(m.stamp);
    for (const auto& m : distortion)
      stamps.insert(m.stamp);

    for (const auto& stamp : stamps)
    {
      const auto KIt = findLastUpToStamp(intrinsicMatrix, stamp, manager->getIntrinsicMatrix());
      if (!KIt.has_value())
        continue;

      const auto rotationIt = findLastUpToStamp(rotation, stamp, manager->getRotation());
      const auto distortionIt = findLastUpToStamp(distortion, stamp, manager->getDistortion());

      TimedMetadata<sensor_msgs::CameraInfo> msg;
      msg.stamp = stamp;
      msg.value = this->compose(KIt->value, rotationIt.has_value() ? rotationIt->value : cras::optional<int>{},
        distortionIt.has_value() ? distortionIt->value : cras::optional<DistortionData>{});

      for (const auto& listener : this->listeners)
        listener->processCameraInfo(msg);

      if (stamp == *stamps.crbegin())
        this->cache->latest.getCameraInfo().emplace(msg.value);
    }
    return stamps.size();
  }

private:
  MovieInfo::ConstPtr info;
  std::shared_ptr<MetadataCache> cache;
  std::weak_ptr<MetadataManager> manager;
};

/**
 * \brief Loader plugin for CameraInfoComposer.
 */
struct CameraInfoComposerPlugin : MetadataExtractorPlugin
{
  MetadataExtractor::Ptr getExtractor(const MetadataExtractorParams& params) override
  {
    if (params.log == nullptr || params.manager.lock() == nullptr || params.cache == nullptr)
      return nullptr;

    return std::make_shared<CameraInfoComposer>(params.log, params.info, params.manager, params.cache);
  }
};

}

PLUGINLIB_EXPORT_CLASS(movie_publisher::CameraInfoComposerPlugin, movie_publisher::MetadataExtractorPlugin)
