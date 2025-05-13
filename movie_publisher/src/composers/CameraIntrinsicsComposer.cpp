// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Composer of camera intrinsics.
 * \author Martin Pecka
 */

#include <cras_cpp_common/type_utils.hpp>
#include <movie_publisher/metadata_manager.h>
#include <pluginlib/class_list_macros.h>

namespace movie_publisher
{

class CameraIntrinsicsComposer : public TimedMetadataExtractor
{
public:
  explicit CameraIntrinsicsComposer(const cras::LogHelperPtr& log, const MovieInfo::ConstPtr& info,
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
    return 0;
  }

  bool hasTimedMetadata() const override
  {
    return true;
  }

  std::unordered_set<MetadataType> supportedTimedMetadata(
    const std::unordered_set<MetadataType>& availableMetadata) const override
  {
    const auto hasMeta = [&availableMetadata](const MetadataType type)
    {
      return availableMetadata.find(type) != availableMetadata.end();
    };
    std::unordered_set<MetadataType> supported;

    if (hasMeta(MetadataType::FOCAL_LENGTH_MM) && hasMeta(MetadataType::FOCAL_LENGTH_35MM))
      supported.insert(MetadataType::CROP_FACTOR);
    if (hasMeta(MetadataType::CROP_FACTOR))
      supported.insert(MetadataType::SENSOR_SIZE_MM);
    if (hasMeta(MetadataType::FOCAL_LENGTH_MM) && hasMeta(MetadataType::CROP_FACTOR))
      supported.insert(MetadataType::FOCAL_LENGTH_35MM);
    if (hasMeta(MetadataType::FOCAL_LENGTH_35MM))
      supported.insert(MetadataType::FOCAL_LENGTH_PX);
    if (hasMeta(MetadataType::FOCAL_LENGTH_MM) && hasMeta(MetadataType::SENSOR_SIZE_MM))
      supported.insert(MetadataType::FOCAL_LENGTH_PX);
    if (hasMeta(MetadataType::FOCAL_LENGTH_35MM) && hasMeta(MetadataType::CROP_FACTOR))
      supported.insert(MetadataType::FOCAL_LENGTH_MM);
    if (hasMeta(MetadataType::FOCAL_LENGTH_PX))
      supported.insert(MetadataType::CAMERA_INFO);

    return supported;
  }

  cras::optional<double> composeCropFactor(const double focalLengthMM, const double focalLength35MM) const
  {
    if (focalLengthMM == 0 || focalLength35MM == 0)
      return cras::nullopt;

    return focalLength35MM / focalLengthMM;
  }

  cras::optional<double> getCropFactor() override
  {
    const auto manager = this->manager.lock();
    if (manager == nullptr)
      return cras::nullopt;

    const auto focalLengthMM = manager->getFocalLengthMM();
    const auto focalLength35MM = manager->getFocalLength35MM();

    if (!focalLengthMM.has_value() || !focalLength35MM.has_value())
      return cras::nullopt;

    const auto cropFactor = this->composeCropFactor(*focalLengthMM, *focalLength35MM);
    if (!cropFactor.has_value())
      return cras::nullopt;
    CRAS_DEBUG_NAMED("metadata_composer",
      "Crop factor %.2f was determined from real and 35 mm focal lengths.", *cropFactor);
    return cropFactor;
  }

  cras::optional<SensorSize> composeSensorSizeMM(const double cropFactor) const
  {
    if (cropFactor == 0)
      return cras::nullopt;

    const auto w = static_cast<double>(this->info->width());
    const auto h = static_cast<double>(this->info->height());
    const auto sensorWidthMM = 36.0 / cropFactor;
    const auto sensorHeightMM = sensorWidthMM * std::min(w, h) / std::max(w, h);
    return SensorSize{sensorWidthMM, sensorHeightMM};
  }

  cras::optional<SensorSize> getSensorSizeMM() override
  {
    const auto manager = this->manager.lock();
    if (manager == nullptr)
      return cras::nullopt;

    const auto cropFactor = manager->getCropFactor();
    if (!cropFactor.has_value())
      return cras::nullopt;

    const auto sensorSize = this->composeSensorSizeMM(*cropFactor);
    if (!sensorSize.has_value())
      return cras::nullopt;
    CRAS_DEBUG_NAMED("metadata_composer",
      "Sensor size %.1fx%1.f mm was determined from crop factor.", sensorSize->first, sensorSize->second);
    return sensorSize;
  }

  cras::optional<double> composeFocalLength35MM(const double cropFactor, const double focalLengthMM) const
  {
    if (cropFactor == 0 || focalLengthMM == 0)
      return cras::nullopt;

    return focalLengthMM * cropFactor;
  }

  cras::optional<double> getFocalLength35MM() override
  {
    const auto manager = this->manager.lock();
    if (manager == nullptr)
      return cras::nullopt;

    const auto cropFactor = manager->getCropFactor();
    const auto focalLength = manager->getFocalLengthMM();

    if (!cropFactor.has_value() || !focalLength.has_value())
      return cras::nullopt;

    const auto f35mm = this->composeFocalLength35MM(*cropFactor, *focalLength);
    if (!f35mm.has_value())
      return cras::nullopt;

    CRAS_DEBUG_NAMED("metadata_composer",
      "Focal length %.1f mm (35 mm equiv) determined from crop factor and real focal length.", *f35mm);
    return f35mm;
  }

  cras::optional<double> composeFocalLengthPxFromFocalLength35MM(const double focalLength35mm) const
  {
    if (focalLength35mm == 0)
      return cras::nullopt;

    const auto imageMaxSize = std::max(this->info->width(), this->info->height());
    return focalLength35mm * static_cast<double>(imageMaxSize) / 36.0;
  }

  cras::optional<double> composeFocalLengthPxFromSensorSizeAndFocalLengthMM(
    const SensorSize& sensorSize, const double focalLengthMM) const
  {
    if (focalLengthMM == 0 || sensorSize.first == 0 || sensorSize.second == 0)
      return cras::nullopt;

    const auto imageMaxSize = std::max(this->info->width(), this->info->height());
    const auto sensorMaxSizeMM = std::max(sensorSize.first, sensorSize.second);
    return focalLengthMM * static_cast<double>(imageMaxSize) / sensorMaxSizeMM;
  }

  cras::optional<double> getFocalLengthPx() override
  {
    const auto manager = this->manager.lock();
    if (manager == nullptr)
      return cras::nullopt;

    const auto focalLength35mm = manager->getFocalLength35MM();
    if (focalLength35mm.has_value())
    {
      const auto focalLengthPx = this->composeFocalLengthPxFromFocalLength35MM(*focalLength35mm);
      if (focalLengthPx.has_value())
      {
        CRAS_DEBUG_NAMED("metadata_composer",
          "Focal length %.1f px determined from 35 mm focal length.", *focalLengthPx);
        return focalLengthPx;
      }
    }

    const auto sensorSizeMM = manager->getSensorSizeMM();
    const auto focalLengthMM = manager->getFocalLengthMM();
    if (sensorSizeMM.has_value() && focalLengthMM.has_value())
    {
      const auto focalLengthPx = this->composeFocalLengthPxFromSensorSizeAndFocalLengthMM(
        *sensorSizeMM, *focalLengthMM);
      if (focalLengthPx.has_value())
      {
        CRAS_DEBUG_NAMED("metadata_composer",
          "Focal length %.1f px determined from real focal length and sensor size.", *focalLengthPx);
        return focalLengthPx;
      }
    }

    return cras::nullopt;
  }

  cras::optional<double> composeFocalLengthMM(const double cropFactor, const double focalLength35MM) const
  {
    if (cropFactor == 0 || focalLength35MM == 0)
      return cras::nullopt;

    return focalLength35MM / cropFactor;
  }

  cras::optional<double> getFocalLengthMM() override
  {
    const auto manager = this->manager.lock();
    if (manager == nullptr)
      return cras::nullopt;

    const auto cropFactor = manager->getCropFactor();
    const auto focalLength35MM = manager->getFocalLength35MM();

    if (!cropFactor.has_value() || !focalLength35MM.has_value())
      return cras::nullopt;

    const auto f = this->composeFocalLengthMM(*cropFactor, *focalLength35MM);
    if (!f.has_value())
      return cras::nullopt;

    CRAS_DEBUG_NAMED("metadata_composer",
      "Real focal length %.1f mm determined from crop factor and 35 mm focal length.", *f);
    return f;
  }

  cras::optional<IntrinsicMatrix> composeIntrinsicMatrix(
    const double focalLengthPx, const cras::optional<int>& rotation) const
  {
    if (focalLengthPx == 0)
      return cras::nullopt;

    IntrinsicMatrix K{};
    K[0 * 3 + 0] = focalLengthPx;
    K[1 * 3 + 1] = focalLengthPx;
    K[0 * 3 + 2] = static_cast<double>(this->info->width()) / 2.0;
    K[1 * 3 + 2] = static_cast<double>(this->info->height()) / 2.0;
    K[2 * 3 + 2] = 1;

    if (rotation.has_value() && (*rotation == 90 || *rotation == 270))
      std::swap(K[0 * 3 + 2], K[1 * 3 + 2]);

    return K;
  }

  cras::optional<IntrinsicMatrix> getIntrinsicMatrix() override
  {
    const auto manager = this->manager.lock();
    if (manager == nullptr)
      return cras::nullopt;

    const auto focalLengthPx = manager->getFocalLengthPx();
    if (!focalLengthPx.has_value())
      return cras::nullopt;

    const auto rotation = manager->getRotation();

    const auto K = this->composeIntrinsicMatrix(*focalLengthPx, rotation);
    if (!K.has_value())
      return cras::nullopt;

    CRAS_DEBUG_NAMED("metadata_composer", "Camera intrinsics have been computed from pixel focal length.");
    return K;
  }

  size_t processTimedMetadata(const MetadataType type, const StreamTime& maxTime, const bool requireOptional) override
  {
    const auto manager = this->manager.lock();
    if (manager == nullptr)
      return 0;

    switch (type)
    {
      case MetadataType::CROP_FACTOR:
        {
          const auto focalLengthMM = this->cache->timed.focalLengthMM();
          const auto focalLength35MM = this->cache->timed.focalLength35MM();

          std::set<StreamTime> stamps {};
          for (const auto& m : focalLengthMM)
            stamps.insert(m.stamp);
          for (const auto& m : focalLength35MM)
            stamps.insert(m.stamp);

          size_t numProcessed = 0;
          for (const auto& stamp : stamps)
          {
            const auto f = findLastUpToStamp(focalLengthMM, stamp, manager->getFocalLengthMM());
            if (!f.has_value())
              continue;

            const auto f35 = findLastUpToStamp(focalLength35MM, stamp, manager->getFocalLength35MM());
            if (!f35.has_value())
              continue;

            const auto cropFactor = this->composeCropFactor(f->value, f35->value);
            if (!cropFactor.has_value())
              continue;

            TimedMetadata<double> msg;
            msg.stamp = stamp;
            msg.value = *cropFactor;

            for (const auto& listener : this->listeners)
              listener->processCropFactor(msg);

            this->cache->latest.getCropFactor().emplace(msg.value);
            numProcessed++;
          }
          return numProcessed;
        }
      case MetadataType::SENSOR_SIZE_MM:
        {
          const auto cropFactors = this->cache->timed.cropFactor();

          size_t numProcessed = 0;
          for (const auto& [stamp, cropFactor] : cropFactors)
          {
            const auto sensorSize = this->composeSensorSizeMM(cropFactor);
            if (!sensorSize.has_value())
              continue;

            TimedMetadata<SensorSize> msg;
            msg.stamp = stamp;
            msg.value = *sensorSize;

            for (const auto& listener : this->listeners)
              listener->processSensorSizeMM(msg);

            this->cache->latest.getSensorSizeMM().emplace(msg.value);
            numProcessed++;
          }
          return numProcessed;
        }
      case MetadataType::FOCAL_LENGTH_35MM:
        {
          const auto cropFactors = this->cache->timed.cropFactor();
          const auto focalLengthsMM = this->cache->timed.focalLengthMM();

          std::set<StreamTime> stamps {};
          for (const auto& m : cropFactors)
            stamps.insert(m.stamp);
          for (const auto& m : focalLengthsMM)
            stamps.insert(m.stamp);

          size_t numProcessed = 0;
          for (const auto& stamp : stamps)
          {
            const auto f = findLastUpToStamp(focalLengthsMM, stamp, manager->getFocalLengthMM());
            if (!f.has_value())
              continue;

            const auto cropFactor = findLastUpToStamp(cropFactors, stamp, manager->getCropFactor());
            if (!cropFactor.has_value())
              continue;

            const auto f35 = this->composeFocalLength35MM(cropFactor->value, f->value);
            if (!f35.has_value())
              continue;

            TimedMetadata<double> msg;
            msg.stamp = stamp;
            msg.value = *f35;

            for (const auto& listener : this->listeners)
              listener->processFocalLength35MM(msg);

            this->cache->latest.getFocalLength35MM().emplace(msg.value);
            numProcessed++;
          }
          return numProcessed;
        }
      case MetadataType::FOCAL_LENGTH_MM:
        {
          const auto cropFactors = this->cache->timed.cropFactor();
          const auto focalLengths35MM = this->cache->timed.focalLength35MM();

          std::set<StreamTime> stamps {};
          for (const auto& m : cropFactors)
            stamps.insert(m.stamp);
          for (const auto& m : focalLengths35MM)
            stamps.insert(m.stamp);

          size_t numProcessed = 0;
          for (const auto& stamp : stamps)
          {
            const auto f35 = findLastUpToStamp(focalLengths35MM, stamp, manager->getFocalLength35MM());
            if (!f35.has_value())
              continue;

            const auto cropFactor = findLastUpToStamp(cropFactors, stamp, manager->getCropFactor());
            if (!cropFactor.has_value())
              continue;

            const auto f = this->composeFocalLengthMM(cropFactor->value, f35->value);
            if (!f.has_value())
              continue;

            TimedMetadata<double> msg;
            msg.stamp = stamp;
            msg.value = *f;

            for (const auto& listener : this->listeners)
              listener->processFocalLengthMM(msg);

            this->cache->latest.getFocalLengthMM().emplace(msg.value);
            numProcessed++;
          }
          return numProcessed;
        }
      case MetadataType::FOCAL_LENGTH_PX:
        {
          const auto focalLengths35MM = this->cache->timed.focalLength35MM();

          size_t numProcessed = 0;
          for (const auto& [stamp, f35] : focalLengths35MM)
          {
            const auto fPx = this->composeFocalLengthPxFromFocalLength35MM(f35);
            if (!fPx.has_value())
              continue;

            TimedMetadata<double> msg;
            msg.stamp = stamp;
            msg.value = *fPx;

            for (const auto& listener : this->listeners)
              listener->processFocalLengthPx(msg);

            this->cache->latest.getFocalLengthPx().emplace(msg.value);
            numProcessed++;
          }

          if (numProcessed > 0)
            return numProcessed;

          const auto sensorSizesMM = this->cache->timed.sensorSizeMM();
          const auto focalLengthsMM = this->cache->timed.focalLengthMM();

          std::set<StreamTime> stamps {};
          for (const auto& m : sensorSizesMM)
            stamps.insert(m.stamp);
          for (const auto& m : focalLengthsMM)
            stamps.insert(m.stamp);

          for (const auto& stamp : stamps)
          {
            const auto f = findLastUpToStamp(focalLengthsMM, stamp, manager->getFocalLengthMM());
            if (!f.has_value())
              continue;

            const auto sensorSizeMM = findLastUpToStamp(sensorSizesMM, stamp, manager->getSensorSizeMM());
            if (!sensorSizeMM.has_value())
              continue;

            const auto fPx = this->composeFocalLengthPxFromSensorSizeAndFocalLengthMM(sensorSizeMM->value, f->value);
            if (!fPx.has_value())
              continue;

            TimedMetadata<double> msg;
            msg.stamp = stamp;
            msg.value = *fPx;

            for (const auto& listener : this->listeners)
              listener->processFocalLengthPx(msg);

            this->cache->latest.getFocalLengthPx().emplace(msg.value);
            numProcessed++;
          }
          return numProcessed;
        }
      case MetadataType::INTRINSIC_MATRIX:
        {
          const auto focalLengthsPx = this->cache->timed.focalLengthPx();
          const auto rotations = this->cache->timed.rotation();
          if (requireOptional && rotations.empty() && !manager->getRotation().has_value())
            return 0;

          size_t numProcessed = 0;
          for (const auto& [stamp, fPx] : focalLengthsPx)
          {
            const auto rotation = findLastUpToStamp(rotations, stamp, manager->getRotation());

            const auto K = this->composeIntrinsicMatrix(
              fPx, rotation.has_value() ? rotation->value : cras::optional<int>{});
            if (!K.has_value())
              continue;

            TimedMetadata<IntrinsicMatrix> msg;
            msg.stamp = stamp;
            msg.value = *K;

            for (const auto& listener : this->listeners)
              listener->processIntrinsicMatrix(msg);

            this->cache->latest.getIntrinsicMatrix().emplace(msg.value);
            numProcessed++;
          }
          return numProcessed;
        }
      default:
        return 0;
    }
  }

private:
  MovieInfo::ConstPtr info;
  std::shared_ptr<MetadataCache> cache;
  std::weak_ptr<MetadataManager> manager;
};

/**
 * \brief Loader plugin for CameraIntrinsicsComposer.
 */
struct CameraIntrinsicsComposerPlugin : MetadataExtractorPlugin
{
  MetadataExtractor::Ptr getExtractor(const MetadataExtractorParams& params) override
  {
    if (params.log == nullptr || params.manager.lock() == nullptr || params.cache == nullptr)
      return nullptr;

    return std::make_shared<CameraIntrinsicsComposer>(params.log, params.info, params.manager, params.cache);
  }
};

}

PLUGINLIB_EXPORT_CLASS(movie_publisher::CameraIntrinsicsComposerPlugin, movie_publisher::MetadataExtractorPlugin)
