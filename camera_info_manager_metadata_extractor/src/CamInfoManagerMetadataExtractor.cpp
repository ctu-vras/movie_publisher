// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include "CamInfoManagerMetadataExtractor.h"

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include <camera_info_manager_lib/camera_info_manager.h>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/type_utils.hpp>
#include <pluginlib/class_list_macros.h>

namespace movie_publisher
{

struct CamInfoManagerMetadataPrivate : public cras::HasLogger
{
  explicit CamInfoManagerMetadataPrivate(const cras::LogHelperPtr& log) : cras::HasLogger(log) {}

  std::weak_ptr<MetadataManager> manager;
  std::list<std::string> calibrationURLs;
  std::list<std::shared_ptr<camera_info_manager_lib::CameraInfoManager>> cameraInfoManagers;
  mutable std::unordered_map<double, cras::optional<sensor_msgs::CameraInfo>> camInfoCache;

  cras::optional<sensor_msgs::CameraInfo> getCameraInfo() const
  {
    const auto manager = this->manager.lock();
    if (manager == nullptr)
      return cras::nullopt;

    const auto focalLengthMM = manager->getFocalLengthMM();
    if (!focalLengthMM.has_value())
      return cras::nullopt;

    if (this->camInfoCache.find(*focalLengthMM) != this->camInfoCache.end())
      return this->camInfoCache.at(*focalLengthMM);

    const auto uniqueName = manager->getCameraUniqueName();
    if (!uniqueName.has_value())
      return cras::nullopt;

    std::string cameraName;
    try
    {
      cameraName = cras::toValidRosName(*uniqueName);
    }
    catch (const std::invalid_argument&)
    {
      return this->camInfoCache[*focalLengthMM] = cras::nullopt;
    }

    CRAS_DEBUG_NAMED("caminfo_manager", "Loading calibrations for camera %s .", cameraName.c_str());
    for (auto& cameraInfoManager : this->cameraInfoManagers)
    {
      cameraInfoManager->setCameraName(cameraName);
      cameraInfoManager->setFocalLength(*focalLengthMM);
      if (cameraInfoManager->isCalibrated())
        return this->camInfoCache[*focalLengthMM] = cameraInfoManager->getCameraInfo();
    }

    return this->camInfoCache[*focalLengthMM] = cras::nullopt;
  }
};

CamInfoManagerMetadataExtractor::CamInfoManagerMetadataExtractor(
  const cras::LogHelperPtr& log, const std::weak_ptr<MetadataManager>& manager,
  const std::list<std::string>& calibrationURLs)
  : MetadataExtractor(log), data(new CamInfoManagerMetadataPrivate(log))
{
  this->data->manager = manager;
  this->data->calibrationURLs = calibrationURLs;

  for (const auto& url : this->data->calibrationURLs)
  {
    if (camera_info_manager_lib::CameraInfoManager::validateURL(url))
      this->data->cameraInfoManagers.emplace_back(
        std::make_shared<camera_info_manager_lib::CameraInfoManager>(this->log, "", url));
    else
      CRAS_WARN_NAMED("caminfo_manager", "Camera calibration URL %s is not valid or supported.", url.c_str());
  }
}

CamInfoManagerMetadataExtractor::~CamInfoManagerMetadataExtractor() = default;

std::string CamInfoManagerMetadataExtractor::getName() const
{
  return cras::getTypeName<std::remove_cv_t<std::remove_reference_t<decltype(*this)>>>();
}

int CamInfoManagerMetadataExtractor::getPriority() const
{
  return 70;
}

cras::optional<CI::_K_type> CamInfoManagerMetadataExtractor::getIntrinsicMatrix()
{
  const auto cameraInfo = this->data->getCameraInfo();
  if (!cameraInfo.has_value())
    return cras::nullopt;

  CRAS_DEBUG_NAMED("caminfo_manager", "Camera intrinsics have been read from stored camera info.");
  return cameraInfo->K;
}

cras::optional<std::pair<CI::_distortion_model_type, CI::_D_type>> CamInfoManagerMetadataExtractor::getDistortion()
{
  const auto cameraInfo = this->data->getCameraInfo();
  if (!cameraInfo.has_value())
    return cras::nullopt;

  CRAS_DEBUG_NAMED("caminfo_manager", "Camera distortion parameters have been read from stored camera info.");
  return std::pair<CI::_distortion_model_type, CI::_D_type>{cameraInfo->distortion_model, cameraInfo->D};
}

MetadataExtractor::Ptr CamInfoManagerMetadataExtractorPlugin::getExtractor(const MetadataExtractorParams& params)
{
  if (params.log == nullptr || params.manager.lock() == nullptr || params.params == nullptr)
    return nullptr;

  const auto& p = params.params->paramsInNamespace("caminfo_manager");
  const std::list<std::string> defaultCalibURLs
  {
    "",
    "file://${ROS_HOME}/camera_info/${NAME}-${FOCAL_LENGTH:%.1fmm}.yaml"
  };
  const auto calibrationURLs = p->getParam("calibration_urls", defaultCalibURLs);

  return std::make_shared<CamInfoManagerMetadataExtractor>(params.log, params.manager, calibrationURLs);
}

}

PLUGINLIB_EXPORT_CLASS(movie_publisher::CamInfoManagerMetadataExtractorPlugin, movie_publisher::MetadataExtractorPlugin)
