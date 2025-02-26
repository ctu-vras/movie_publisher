// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Metadata extractor using lensfun backend.
 * \author Martin Pecka
 */

#include "LensfunMetadataExtractor.h"

#include <sys/stat.h>

#include <lensfun/lensfun.h>
#include <opencv2/calib3d.hpp>

#include <cras_cpp_common/type_utils.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/distortion_models.h>

#if LF_VERSION_MAJOR > 0 || LF_VERSION_MINOR > 3 || LF_VERSION_MICRO > 2
#define lfDatabaseReadTimestamp lfDatabase::ReadTimestamp
lfError lfLoadFileOrDir(lfDatabase& db, const std::string& fileOrDir)
{
  return db.Load(fileOrDir.c_str());
}
#else
extern long int _lf_read_database_timestamp(char const*);  // NOLINT
#define lfDatabaseReadTimestamp _lf_read_database_timestamp
lfError lfLoadFileOrDir(lfDatabase& db, const std::string& fileOrDir)
{
  struct stat s;
  if (stat(fileOrDir.c_str(), &s) == 0 && s.st_mode & S_IFDIR)
    return db.LoadDirectory(fileOrDir.c_str()) ? LF_NO_ERROR : LF_NO_DATABASE;
  else
    return db.Load(fileOrDir.c_str());
}
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
  bool isStillImage {false};
  std::string extraDbFolder;

  lfDatabase* getDb()
  {
    if (this->db.has_value())
      return *this->db;

    this->dbPtr = std::make_unique<lfDatabase>();

    if (this->dbPtr->Load() == LF_NO_ERROR)
    {
      if (!this->extraDbFolder.empty())
      {
        if (lfLoadFileOrDir(*this->dbPtr, this->extraDbFolder) != LF_NO_ERROR)
          CRAS_ERROR_NAMED("lensfun",
            "Loading lensfun basic database succeeded, but loading the extra database from %s failed.",
            this->extraDbFolder.c_str());
        else
          CRAS_DEBUG_NAMED("lensfun", "Loaded data from extra lensfun database %s.", this->extraDbFolder.c_str());
      }

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

    // The static cast is needed for 32-bit builds
    if (maxStamp == -1 || std::abs(static_cast<int64_t>(maxStamp - ros::WallTime::now().sec)) > 60 * 60 * 24 * 30)
      CRAS_WARN_ONCE_NAMED("lensfun", "Lensfun database is old. Consider calling lensfun-update-data.");
  }

  /**
   * \brief Find a camera and lens that match the camera data provided by MetadataManager, and call a callback on them.
   * \param[in] cb Callback to be called for matching camera and lens (one of them can be nullptr).
   * \return The result of the callback, or false if no matching camera or lens was found.
   */
  bool cameraAndLensCallback(const std::function<bool(const lfCamera* camera, const lfLens* lens)>& cb)
  {
    const auto manager = this->manager.lock();
    if (manager == nullptr)
      return false;

    const auto db = this->getDb();
    if (db == nullptr)
      return false;

    const auto make = manager->getCameraMake().value_or("");
    const auto model = manager->getCameraModel().value_or("");
    const auto lensMake = manager->getLensMake().value_or("");
    const auto lensModel = manager->getLensModel().value_or("");

    if (make.empty() && model.empty() && lensMake.empty() && lensModel.empty())
      return false;

    const float aspect = static_cast<float>(this->width) / static_cast<float>(this->height);

    const auto cameras = db->FindCameras(
      make.empty() ? nullptr : make.c_str(), model.empty() ? nullptr : model.c_str());
    const lfCamera* camera = nullptr;
    for (size_t i = 0; cameras != nullptr && cameras[i] != nullptr; ++i)
    {
      CRAS_DEBUG_NAMED("lensfun", "Checking matching camera '%s %s'.", cameras[i]->Maker, cameras[i]->Model);
      if (cameras[i]->Variant != nullptr && strlen(cameras[i]->Variant) > 0)
      {
        const std::string variant = cameras[i]->Variant;
        CRAS_DEBUG_NAMED("lensfun", "Checking variant '%s'.", variant.c_str());
        const auto parts = cras::split(variant, " ", 1);
        std::string ratioStr, rest;
        if (cras::contains(parts[0], ':'))
        {
          ratioStr = parts[0];
          rest = parts.size() > 1 ? parts[1] : "";
        }
        else if (parts.size() > 1 && cras::contains(parts[1], ':'))
        {
          ratioStr = parts[1];
          rest = parts[0];
        }
        cras::strip(ratioStr);
        cras::strip(rest);
        if (this->isStillImage == cras::contains(rest, "video"))
        {
          CRAS_DEBUG_NAMED("lensfun", "Ignoring variant '%s' because it %s for still images.",
            variant.c_str(), this->isStillImage ? "is not" : "is");
          continue;
        }
        float ratio {0.0};
        const auto ratioParts = cras::split(ratioStr, ":");
        if (ratioParts.size() == 2)
        {
          try
          {
            ratio = cras::parseFloat(ratioParts[0]) / cras::parseFloat(ratioParts[1]);
          }
          catch (const std::invalid_argument&) {}
        }
        if (std::abs(ratio - aspect) > 1e-2 && std::abs(ratio - 1 / aspect) > 1e-2)
        {
          CRAS_DEBUG_NAMED("lensfun",
            "Ignoring variant '%s' because its aspect ratio does not match.", variant.c_str());
          continue;
        }
        camera = cameras[i];
        CRAS_DEBUG_NAMED("lensfun", "Variant '%s' matches.", variant.c_str());
        break;
      }
      else
      {
        // Camera has no variant, match any aspect ratio and only still images (because video has most probably a
        // different crop factor).
        if (this->isStillImage)
        {
          camera = cameras[i];
          CRAS_DEBUG_NAMED("lensfun", "Camera '%s %s' matches without a variant.", camera->Maker, camera->Model);
          // Do not stop iterations. If we found a more specific camera with a variant, it should win.
          continue;
        }
        else
        {
          CRAS_DEBUG_NAMED("lensfun", "Ignoring camera '%s %s' without a variant because a video profile is requested.",
            cameras[i]->Maker, cameras[i]->Model);
        }
      }
    }

    const auto lenses = db->FindLenses(
      camera, lensMake.empty() ? nullptr : lensMake.c_str(), lensModel.empty() ? nullptr : lensModel.c_str());
    const lfLens* lens = nullptr;
    if (lenses != nullptr)
    {
      size_t i;
      for (i = 0; lenses != nullptr && lenses[i] != nullptr; i++)
      {
        CRAS_DEBUG_NAMED("lensfun", "Checking matching lens '%s %s'.", lenses[i]->Maker, lenses[i]->Model);
        if (std::abs(lenses[i]->AspectRatio - aspect) < 1e-2 || std::abs(lenses[i]->AspectRatio - 1 / aspect) < 1e-2)
        {
          lens = *lenses;
          CRAS_DEBUG_NAMED("lensfun", "Lens '%s %s' matches aspect ratio.", lens->Maker, lens->Model);
          break;
        }
        else
        {
          CRAS_DEBUG_NAMED("lensfun", "Lens '%s %s' does not match the aspect ratio.",
            lenses[i]->Maker, lenses[i]->Model);
        }
      }
      if (lens == nullptr && camera != nullptr && (camera->Variant == nullptr || strlen(camera->Variant) == 0))
      {
        // When all lenses have wrong aspect ratio and the camera has no variant, then the crop factor of the camera is
        // probably wrong, so we rather give up the camera, too.
        camera = nullptr;
        CRAS_INFO_NAMED("lensfun",
          "%zu lenses matched in the lensfun database, but none had matching aspect ratio. "
          "Ignoring the lens and camera.", i);
      }
    }

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
  const size_t width, const size_t height, const bool isStillImage, const std::string& extraDb)
  : MetadataExtractor(log), data(new LensfunMetadataPrivate(log))
{
  this->data->manager = manager;
  this->data->width = width;
  this->data->height = height;
  this->data->isStillImage = isStillImage;
  this->data->extraDbFolder = extraDb;
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

  const auto flags =
    cv::CALIB_USE_INTRINSIC_GUESS |
    cv::CALIB_FIX_ASPECT_RATIO |
    cv::CALIB_FIX_FOCAL_LENGTH |
    cv::CALIB_FIX_PRINCIPAL_POINT |
    cv::CALIB_RATIONAL_MODEL;

  double rms = cv::calibrateCamera(
    std::vector<std::vector<cv::Point3f>>{{squareCorners}},
    std::vector<std::vector<cv::Point2f>>{{lfDistorted2dPoints}},
    cv::Size(w, h), camMatrix, distCoeffs, cv::noArray(), cv::noArray(), flags);

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

  std::string extraDb;
  if (params.params->hasParam("lensfun_extra_db"))
    extraDb = params.params->getParam("lensfun_extra_db", std::string{});

  return std::make_shared<LensfunMetadataExtractor>(
    params.log, params.manager, params.width, params.height, params.isStillImage, extraDb);
}

}

PLUGINLIB_EXPORT_CLASS(movie_publisher::LensfunMetadataExtractorPlugin, movie_publisher::MetadataExtractorPlugin)
