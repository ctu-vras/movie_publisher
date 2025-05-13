// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Composer of camera name elements.
 * \author Martin Pecka
 */

#include <cras_cpp_common/type_utils.hpp>
#include <movie_publisher/metadata_manager.h>
#include <pluginlib/class_list_macros.h>

namespace movie_publisher
{

class CameraNameComposer : public MetadataExtractor
{
public:
  explicit CameraNameComposer(const cras::LogHelperPtr& log, const std::weak_ptr<MetadataManager>& manager,
    const std::shared_ptr<MetadataCache>& cache) : MetadataExtractor(log), cache(cache), manager(manager)
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

  cras::optional<std::string> getCameraGeneralName() override
  {
    const auto manager = this->manager.lock();
    if (manager == nullptr)
      return cras::nullopt;

    const auto make = manager->getCameraMake().value_or("");
    const auto model = manager->getCameraModel().value_or("");
    const auto lensMake = manager->getLensMake().value_or("");
    const auto lensModel = manager->getLensModel().value_or("");

    if (make.empty() && model.empty() && lensMake.empty() && lensModel.empty())
      return cras::nullopt;

    const auto cameraName = cras::strip(cras::join<std::list<std::string>>({make, model}, " "));
    const auto lensName = cras::strip(cras::join<std::list<std::string>>({lensMake, lensModel}, " "));
    auto name = cras::strip(cras::join<std::list<std::string>>({cameraName, lensName}, " "));

    if (name.empty())
      return this->cache->latest.getCameraGeneralName().emplace(cras::nullopt);

    CRAS_DEBUG_NAMED("metadata_composer", "Camera name composed from make and model of the camera and lens.");
    return name;
  }

  cras::optional<std::string> getCameraUniqueName() override
  {
    const auto manager = this->manager.lock();
    if (manager == nullptr)
      return cras::nullopt;

    const auto serial = manager->getCameraSerialNumber();
    if (!serial.has_value() || serial->empty())
      return cras::nullopt;

    const auto name = this->getCameraGeneralName().value_or("camera");
    CRAS_DEBUG_NAMED("metadata_composer", "Camera unique name has been composed from its general name and serial nr.");
    return cras::format("%s (%s)", name.c_str(), serial->c_str());
  }

private:
  std::shared_ptr<MetadataCache> cache;
  std::weak_ptr<MetadataManager> manager;
};

/**
 * \brief Loader plugin for CameraNameComposer.
 */
struct CameraNameComposerPlugin : MetadataExtractorPlugin
{
  MetadataExtractor::Ptr getExtractor(const MetadataExtractorParams& params) override
  {
    if (params.log == nullptr || params.manager.lock() == nullptr || params.cache == nullptr)
      return nullptr;

    return std::make_shared<CameraNameComposer>(params.log, params.manager, params.cache);
  }
};

}

PLUGINLIB_EXPORT_CLASS(movie_publisher::CameraNameComposerPlugin, movie_publisher::MetadataExtractorPlugin)
