// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Metadata extractor from GPMF streams.
 * \author Martin Pecka
 */

#include "GPMFMetadataExtractor.h"

extern "C"
{
#include <libavformat/avformat.h>
}

#include <cras_cpp_common/type_utils.hpp>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/distortion_models.h>

namespace movie_publisher
{

struct GPMFMetadataPrivate : public cras::HasLogger
{
  explicit GPMFMetadataPrivate(const cras::LogHelperPtr& log) : cras::HasLogger(log) {}

  std::weak_ptr<MetadataManager> manager;

  const AVFormatContext* avFormatContext {nullptr};
  size_t videoStreamIndex {0u};
  AVStream* stream {nullptr};

  size_t width {0u};
  size_t height {0u};
  bool isStillImage {false};
};

GPMFMetadataExtractor::GPMFMetadataExtractor(
  const cras::LogHelperPtr& log, const std::weak_ptr<MetadataManager>& manager,
  const size_t width, const size_t height, const bool isStillImage,
  const AVFormatContext* avFormatContext, const size_t videoStreamIndex)
  : MetadataExtractor(log), data(new GPMFMetadataPrivate(log))
{
  this->data->manager = manager;
  this->data->width = width;
  this->data->height = height;
  this->data->isStillImage = isStillImage;
  this->data->avFormatContext = avFormatContext;
  this->data->videoStreamIndex = videoStreamIndex;
}

GPMFMetadataExtractor::~GPMFMetadataExtractor() = default;

std::string GPMFMetadataExtractor::getName() const
{
  return cras::getTypeName<std::remove_cv_t<std::remove_reference_t<decltype(*this)>>>();
}

int GPMFMetadataExtractor::getPriority() const
{
  return 5;
}

cras::optional<double> GPMFMetadataExtractor::getCropFactor()
{
  // TODO not sure where to get it; in the worst case, use a hard-coded table of GoPro models
  return cras::nullopt;
}

cras::optional<std::pair<double, double>> GPMFMetadataExtractor::getSensorSizeMM()
{
  // TODO not sure where to get it; in the worst case, hard-code a table from https://en.wikipedia.org/wiki/GoPro#HERO13
  return cras::nullopt;
}

cras::optional<double> GPMFMetadataExtractor::getFocalLengthMM()
{
  // TODO not sure where to get it, maybe a hard-coded table based on aspect ratio, lens mode and a static table
  //      per GoPro model from https://www.google.com/search?q=gopro+Digital+Lenses+FOV+Information ?
  //      Also consider DZOM+DZST, EISE+EISA+HCTL, ZFOV+VFOV, ARUW+ARWA
  return cras::nullopt;
}

cras::optional<std::pair<CI::_distortion_model_type, CI::_D_type>> GPMFMetadataExtractor::getDistortion()
{
  // TODO https://github.com/gopro/gpmf-parser?tab=readme-ov-file#dvid-fovl-large-fov---lens-distortion
  //      If VFOV is Linear, distortion is already corrected so this function should return all zeros
  //      SuperView and HyperView use a nonlinear horizontal stretching algorithm (MXCF, MYCF, MAPX, MAPY)
  //      (https://abekislevitz.com/43-gopro-footage-explained/), so there's no way to make them fully working with the
  //      current framework.
  //      GoPro with Wide FOV has fisheye lens, so use sensor_msgs::distortion_models::EQUIDISTANT
  return cras::nullopt;
}

cras::optional<std::string> GPMFMetadataExtractor::getCameraSerialNumber()
{
  // TODO CASN
  return cras::nullopt;
}

cras::optional<std::string> GPMFMetadataExtractor::getCameraMake()
{
  // TODO hard-code to "GoPro" ?
  return cras::nullopt;
}

cras::optional<std::string> GPMFMetadataExtractor::getCameraModel()
{
  // TODO MINF
  return cras::nullopt;
}

cras::optional<std::string> GPMFMetadataExtractor::getLensMake()
{
  // TODO hard-code to "GoPro" ?
  return cras::nullopt;
}

cras::optional<std::string> GPMFMetadataExtractor::getLensModel()
{
  // TODO LINF???
  return cras::nullopt;
}

cras::optional<int> GPMFMetadataExtractor::getRotation()
{
  // TODO OREN, maybe IORI?
  return cras::nullopt;
}

cras::optional<ros::Time>GPMFMetadataExtractor::getCreationTime()
{
  // TODO CDAT, maybe TZON
  return cras::nullopt;
}

cras::optional<double> GPMFMetadataExtractor::getFocalLength35MM()
{
  // TODO not sure where to get it; if it isn't anywhere, leave it out and let manager compute it from crop factor and
  //      focal length in mm
  return cras::nullopt;
}

cras::optional<double> GPMFMetadataExtractor::getFocalLengthPx()
{
  // TODO not sure where to get it; if it isn't anywhere, leave it out and let manager compute it from sensor size and
  //      focal length in mm
  return cras::nullopt;
}

cras::optional<CI::_K_type> GPMFMetadataExtractor::getIntrinsicMatrix()
{
  // TODO if neither calibration matrix K nor projection matrix P are defined, leave this out and let manager compute it
  //      from pixel focal length and image dimensions
  return cras::nullopt;
}

std::pair<cras::optional<sensor_msgs::NavSatFix>, cras::optional<gps_common::GPSFix>> GPMFMetadataExtractor::
getGNSSPosition()
{
  // TODO GPS5+GPSU+GPSF+GPSP or GPS9
  return {cras::nullopt, cras::nullopt};
}

cras::optional<compass_msgs::Azimuth> GPMFMetadataExtractor::getAzimuth()
{
  // TODO compute from MAGN
  return cras::nullopt;
}

cras::optional<std::pair<double, double>> GPMFMetadataExtractor::getRollPitch()
{
  // TODO compute from GRAV (better case) or ACCL
  return cras::nullopt;
}

cras::optional<geometry_msgs::Vector3> GPMFMetadataExtractor::getAcceleration()
{
  // TODO ACCL
  return cras::nullopt;
}

MetadataExtractor::Ptr GPMFMetadataExtractorPlugin::getExtractor(const MetadataExtractorParams& params)
{
  if (params.log == nullptr || params.manager.lock() == nullptr || params.width == 0 || params.height == 0)
    return nullptr;

  return std::make_shared<GPMFMetadataExtractor>(
    params.log, params.manager, params.width, params.height, params.isStillImage,
    params.avFormatContext, params.streamIndex);
}

}

PLUGINLIB_EXPORT_CLASS(movie_publisher::GPMFMetadataExtractorPlugin, movie_publisher::MetadataExtractorPlugin)
