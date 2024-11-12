// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief
 * \author Martin Pecka
 */

#include "LensfunMetadataExtractor.h"

#include <lensfun/lensfun.h>
#include <opencv2/calib3d.hpp>

#include <cras_cpp_common/type_utils.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/distortion_models.h>

#if LF_VERSION_MAJOR > 0 || LF_VERSION_MINOR > 3 || LF_VERSION_MICRO > 2
#define lfDatabaseReadTimestamp lfDatabase::ReadTimestamp
#else
extern long int _lf_read_database_timestamp(char const*);  // NOLINT
#define lfDatabaseReadTimestamp _lf_read_database_timestamp
#endif

namespace movie_publisher
{

struct LensfunMetadataPrivate : public cras::HasLogger
{
  explicit LensfunMetadataPrivate(const cras::LogHelperPtr& log) : cras::HasLogger(log) {}

  std::weak_ptr<MetadataManager> manager;

  std::unique_ptr<lfDatabase> dbPtr;
  cras::optional<lfDatabase*> db;
  size_t width {0u};
  size_t height {0u};

  lfDatabase* getDb()
  {
    if (this->db.has_value())
      return *this->db;

    this->dbPtr = std::make_unique<lfDatabase>();

    if (this->dbPtr->Load() == LF_NO_ERROR)
    {
      this->db = this->dbPtr.get();
    }
    else
    {
      CRAS_DEBUG_NAMED("lensfun", "Loading lensfun databases failed.");
      this->db = nullptr;
    }

    return *this->db;
  }

  void warnIfDbOld(const lfDatabase* db) const
  {
    // Check the time of the last lensfun db update. If it is older than a month, issue a warning.
#if LF_VERSION_MAJOR > 0 || LF_VERSION_MINOR > 3 || LF_VERSION_MICRO > 2
    const auto homeDir = lfDatabase::UserLocation;
    const auto updatesDir = lfDatabase::UserUpdatesLocation;
#else
    const auto homeDir = db->HomeDataDir;
    const auto updatesDir = db->UserUpdatesDir;
#endif
    long int maxStamp = -1;  // NOLINT
    maxStamp = std::max(maxStamp, lfDatabaseReadTimestamp(homeDir));
    maxStamp = std::max(maxStamp, lfDatabaseReadTimestamp(updatesDir));

    if (maxStamp == -1 || std::abs(maxStamp - ros::WallTime::now().sec) > 60 * 60 * 24 * 30)
      CRAS_WARN_ONCE_NAMED("lensfun", "Lensfun database is old. Consider calling lensfun-update-data.");
  }

  bool cameraAndLensCallback(const std::function<bool(const lfCamera* camera, const lfLens* lens)>& cb)
  {
    if (this->manager.lock() == nullptr)
      return false;

    const auto db = this->getDb();
    if (db == nullptr)
      return false;

    const auto make = this->manager.lock()->getCameraMake().value_or("");
    const auto model = this->manager.lock()->getCameraModel().value_or("");
    const auto lensMake = this->manager.lock()->getLensMake().value_or("");
    const auto lensModel = this->manager.lock()->getLensModel().value_or("");

    if (make.empty() && model.empty() && lensMake.empty() && lensModel.empty())
      return false;

    const auto cameras = db->FindCameras(
      make.empty() ? nullptr : make.c_str(), model.empty() ? nullptr : model.c_str());
    const lfCamera* camera = nullptr;
    if (cameras != nullptr)
      camera = *cameras;

    const auto lenses = db->FindLenses(
      camera, lensMake.empty() ? nullptr : lensMake.c_str(), lensModel.empty() ? nullptr : lensModel.c_str());
    const lfLens* lens = nullptr;
    if (lenses != nullptr)
      lens = *lenses;

    bool result = false;
    if (camera == nullptr && lens == nullptr)
      this->warnIfDbOld(db);
    else
      result = cb(camera, lens);

    lf_free(lenses);
    lf_free(cameras);

    return result;
  }
};

LensfunMetadataExtractor::LensfunMetadataExtractor(
  const cras::LogHelperPtr& log, const std::weak_ptr<MetadataManager>& manager,
  const size_t width, const size_t height)
  : MetadataExtractor(log), data(new LensfunMetadataPrivate(log))
{
  this->data->manager = manager;
  this->data->width = width;
  this->data->height = height;
}

LensfunMetadataExtractor::~LensfunMetadataExtractor() = default;

std::string LensfunMetadataExtractor::getName() const
{
  return cras::getTypeName<std::remove_cv_t<std::remove_reference_t<decltype(*this)>>>();
}

int LensfunMetadataExtractor::getPriority() const
{
  return 60;
}

cras::optional<double> LensfunMetadataExtractor::getCropFactor()
{
  cras::optional<double> cropFactor;
  const auto cb = [&cropFactor](const lfCamera* camera, const lfLens* lens)
  {
    if (camera != nullptr)
      cropFactor = camera->CropFactor;
    else if (lens != nullptr)
      cropFactor = lens->CropFactor;
    return cropFactor.has_value();
  };

  if (this->data->cameraAndLensCallback(cb))
  {
    CRAS_DEBUG_NAMED("lensfun", "Crop factor %.2f was determined from lensfun database.", *cropFactor);
    return cropFactor;
  }

  return cras::nullopt;
}

cras::optional<std::pair<double, double>> LensfunMetadataExtractor::getSensorSizeMM()
{
  const auto cropFactor = this->getCropFactor();
  if (cropFactor == cras::nullopt)
    return cras::nullopt;

  const auto& w = this->data->width;
  const auto& h = this->data->height;
  const auto sensorWidthMM = 36.0 / *cropFactor;
  const auto sensorHeightMM = sensorWidthMM * std::min(w, h) / std::max(w, h);
  CRAS_DEBUG_NAMED("lensfun",
    "Sensor size %.1fx%1.f mm was determined from crop factor.", sensorWidthMM, sensorHeightMM);

  return std::pair<double, double>{sensorWidthMM, sensorHeightMM};
}

cras::optional<double> LensfunMetadataExtractor::getFocalLengthMM()
{
  cras::optional<double> focalLengthMM;
  const auto cb = [&focalLengthMM](const lfCamera* camera, const lfLens* lens)
  {
    if (lens != nullptr && lens->MinFocal == lens->MaxFocal)
      focalLengthMM = lens->MinFocal;
    return focalLengthMM.has_value();
  };

  if (this->data->cameraAndLensCallback(cb))
  {
    CRAS_DEBUG_NAMED("lensfun",
      "Real focal length %.1f mm determined from lensfun database as the lens is fixed.", *focalLengthMM);
    return focalLengthMM;
  }

  return cras::nullopt;
}

cras::optional<std::pair<CI::_distortion_model_type, CI::_D_type>> LensfunMetadataExtractor::getDistortion()
{
  const auto manager = this->data->manager.lock();
  if (manager == nullptr)
    return cras::nullopt;

  const auto focalLengthMM = manager->getFocalLengthMM();
  if (!focalLengthMM.has_value())
    return cras::nullopt;

  cras::optional<lfLensCalibDistortion> lensfunDist;
  const auto cb = [&lensfunDist, focalLengthMM](const lfCamera* camera, const lfLens* lens)
  {
    if (lens == nullptr)
      return false;
    lensfunDist.emplace();
    if (!lens->InterpolateDistortion(*focalLengthMM, *lensfunDist) || lensfunDist->Model == LF_DIST_MODEL_NONE)
      lensfunDist.reset();
    return lensfunDist.has_value();
  };

  if (!this->data->cameraAndLensCallback(cb))
    return cras::nullopt;

  auto camInfo = manager->getIntrinsicMatrix();
  if (!camInfo.has_value())
    return cras::nullopt;

  const auto rotation = manager->getRotation();
  const auto portrait = rotation.has_value() && (*rotation == 90 || *rotation == 270);

  auto& K = *camInfo;
  const auto& w = !portrait ? this->data->width : this->data->height;
  const auto& h = !portrait ? this->data->height : this->data->width;

  sensor_msgs::CameraInfo camInfoMsg;
  camInfoMsg.width = w;
  camInfoMsg.height = h;
  camInfoMsg.K = K;
  for (size_t row = 0; row < 3; ++row)
    std::copy_n(&camInfoMsg.K[row * 3], 3, &camInfoMsg.P[row * 4]);

  image_geometry::PinholeCameraModel cam;
  cam.fromCameraInfo(camInfoMsg);

  const auto tl = cam.projectPixelTo3dRay({0.0, 0.0});
  const auto tr = cam.projectPixelTo3dRay({w * 1.0, 0.0});
  const auto bl = cam.projectPixelTo3dRay({0.0, h * 1.0});

  const auto width3D = tr.x - tl.x;
  const auto height3D = bl.y - tl.y;

  std::vector<cv::Point3f> squareCorners;
  std::vector<cv::Point2f> rectified2dPoints;
  size_t numSquares = 16;
  const auto squareSize = std::min(width3D, height3D) / numSquares;
  for (size_t i = 0; i < numSquares; i++)
  {
    for (size_t j = 0; j < numSquares; j++)
    {
      squareCorners.emplace_back(j * squareSize, i * squareSize, 0.0f);
      rectified2dPoints.emplace_back(
        cam.project3dToPixel({squareCorners.back().x + tl.x, tl.y - squareCorners.back().y, tl.z}));
    }
  }

  std::vector<cv::Point2f> lfDistorted2dPoints;
  cv::Point2f center {w / 2.0f, h / 2.0f};
  const auto minDim = std::min(w, h);
  for (const auto& rectPoint : rectified2dPoints)
  {
    auto diff = rectPoint - center;
    // lensfun uses coordinates normalized by the smaller image dimension
    const auto r = cv::norm(diff) / minDim;
    const auto& d = lensfunDist->Terms;
    double r_d;
    switch (lensfunDist->Model)
    {
      case LF_DIST_MODEL_POLY3:
        r_d = r * (1 - d[0] + d[0] * std::pow(r, 2));
        break;
      case LF_DIST_MODEL_POLY5:
        r_d = r * (1 + d[0] * std::pow(r, 2) + d[1] * std::pow(r, 4));
        break;
      case LF_DIST_MODEL_PTLENS:
        r_d = r * (d[0] * std::pow(r, 3) + d[1] * std::pow(r, 2) + d[2] * r + 1 - d[0] - d[1] - d[2]);
        break;
      default:
        r_d = r;
    }
    diff *= r_d / r;
    lfDistorted2dPoints.emplace_back(center + diff);
  }

  cv::Mat camMatrix(3, 3, CV_64F, K.data());
  cv::Mat distCoeffs;

  double rms = cv::calibrateCamera(
    std::vector{{squareCorners}}, std::vector{{lfDistorted2dPoints}}, cv::Size(w, h),
    camMatrix, distCoeffs, cv::noArray(), cv::noArray(),
    cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_FIX_FOCAL_LENGTH | cv::CALIB_FIX_PRINCIPAL_POINT | cv::CALIB_RATIONAL_MODEL);  // NOLINT

  CRAS_DEBUG_NAMED("lensfun", "Distortion model estimated from lensfun DB with RMS error %f px.", rms);

  camInfoMsg.D.resize(8);
  for (size_t i = 0; i < 8; i++)
    camInfoMsg.D[i] = distCoeffs.at<double>(i);
  camInfoMsg.distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;

  return std::pair{camInfoMsg.distortion_model, camInfoMsg.D};
}

MetadataExtractor::Ptr LensfunMetadataExtractorPlugin::getExtractor(const MetadataExtractorParams& params)
{
  if (params.log == nullptr || params.manager.lock() == nullptr || params.width == 0 || params.height == 0)
    return nullptr;

  return std::make_shared<LensfunMetadataExtractor>(params.log, params.manager, params.width, params.height);
}

}

PLUGINLIB_EXPORT_CLASS(movie_publisher::LensfunMetadataExtractorPlugin, movie_publisher::MetadataExtractorPlugin)
