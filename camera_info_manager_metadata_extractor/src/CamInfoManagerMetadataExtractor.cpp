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
/**
 * \brief Private data.
 */
struct CamInfoManagerMetadataPrivate : public cras::HasLogger
{
  explicit CamInfoManagerMetadataPrivate(const cras::LogHelperPtr& log) : cras::HasLogger(log) {}

  std::weak_ptr<MetadataManager> manager;  //!< Metadata manager.
  std::list<std::string> calibrationURLs;  //!< The calibration URLs.
  //! Instances of CameraInfoManager, on for each calibration URL.
  std::list<std::shared_ptr<camera_info_manager_lib::CameraInfoManager>> cameraInfoManagers;
  //! Cache of loaded CameraInfo, indexed by focal length.
  mutable std::unordered_map<double, cras::optional<sensor_msgs::CameraInfo>> camInfoCache;
  size_t width {0u};
  size_t height {0u};

  /**
   * \brief Search for and retrieve the camera info corresponding to the current movie.
   *
   * This requires the metadata manager ot be able to extract focalLengthMM and cameraUniqueName.
   *
   * \return Camera info if it was found, or nothing.
   */
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
      {
        const auto& camInfo = cameraInfoManager->getCameraInfo();
        if (camInfo.width == this->width && camInfo.height == this->height)
          return this->camInfoCache[*focalLengthMM] = cameraInfoManager->getCameraInfo();
        else
          CRAS_DEBUG_NAMED("caminfo_manager",
            "Skipping camera calibration because it has wrong image dimensions.");
      }
    }

    return this->camInfoCache[*focalLengthMM] = cras::nullopt;
  }
};

CamInfoManagerMetadataExtractor::CamInfoManagerMetadataExtractor(
  const cras::LogHelperPtr& log, const std::weak_ptr<MetadataManager>& manager, const size_t width, const size_t height,
  const std::list<std::string>& calibrationURLs)
  : MetadataExtractor(log), data(new CamInfoManagerMetadataPrivate(log))
{
  this->data->manager = manager;
  this->data->calibrationURLs = calibrationURLs;
  this->data->width = width;
  this->data->height = height;

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

  CRAS_DEBUG("Camera intrinsics have been read from stored camera info.");
  return cameraInfo->K;
}

cras::optional<std::pair<CI::_distortion_model_type, CI::_D_type>> CamInfoManagerMetadataExtractor::getDistortion()
{
  const auto cameraInfo = this->data->getCameraInfo();
  if (!cameraInfo.has_value())
    return cras::nullopt;

  CRAS_DEBUG("Camera distortion parameters have been read from stored camera info.");
  return std::pair<CI::_distortion_model_type, CI::_D_type>{cameraInfo->distortion_model, cameraInfo->D};
}

MetadataExtractor::Ptr CamInfoManagerMetadataExtractorPlugin::getExtractor(const MetadataExtractorParams& params)
{
  if (params.log == nullptr || params.manager.lock() == nullptr || params.params == nullptr)
    return nullptr;

  const std::list<std::string> defaultCalibURLs
  {
    "",
    "file://${ROS_HOME}/camera_info/${NAME}-${FOCAL_LENGTH:%0.01fmm}.yaml"
  };
  auto calibrationURLs = defaultCalibURLs;

  try
  {
    const auto& p = params.params->paramsInNamespace("caminfo_manager");
    calibrationURLs = p->getParam("calibration_urls", defaultCalibURLs);
  }
  catch (const std::runtime_error&)
  {
    // Parameter namespace does not exist.
  }

  return std::make_shared<CamInfoManagerMetadataExtractor>(
    params.log, params.manager, params.width, params.height, calibrationURLs);
}

}

PLUGINLIB_EXPORT_CLASS(movie_publisher::CamInfoManagerMetadataExtractorPlugin, movie_publisher::MetadataExtractorPlugin)
