// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Extractor of image or movie metadata.
 * \author Martin Pecka
 */

#pragma once

#include <memory>
#include <string>
#include <utility>

#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <geometry_msgs/Vector3.h>
#include <gps_common/GPSFix.h>
#include <ros/time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/NavSatFix.h>


class AVFormatContext;

namespace movie_publisher
{

using CI = sensor_msgs::CameraInfo;

class MetadataManager;

/**
 * \brief Parameters passed to the extractor plugins when initializing them.
 */
struct MetadataExtractorParams
{
  cras::LogHelperPtr log;  //!< Logger.
  std::weak_ptr<MetadataManager> manager;  //!< Weak pointer to the metadata manager. Use it to call other extractors.
  cras::BoundParamHelperPtr params;  //!< ROS parameters which can be used to configure the extractor plugin.
  std::string filename;  //!< Filename of the processed movie.
  size_t width;  //!< Width of the processed movie [px].
  size_t height;  //!< Height of the processed movie [px].
  const AVFormatContext* avFormatContext;  //!< Libav context of the open movie.
  int streamIndex;  //!< Index of the stream in avFormatContext containing the processed movie.
  bool isStillImage;  //!< Whether the movie is a still image (just one frame) or not.
};

/**
 * \brief Extractor of metadata about a movie or image.
 *
 * The extractors implementing this interface should only override methods in case they can directly provide the data.
 * If some data cannot be provided by an extractor, it is not a problem - MetadataManager will call other extractors
 * to get the information.
 *
 * If this extractor needs some metadata it cannot get itself, it can store a pointer to the manager and then call the
 * manager to ask other extractors for the needed data. In this case, use StackGuard to protect against infinite
 * recursion.
 */
class MetadataExtractor : public cras::HasLogger
{
public:
  /**
   * \brief Constructor.
   * \param[in] log Logger.
   */
  explicit MetadataExtractor(const cras::LogHelperPtr& log) : HasLogger(log) {}
  virtual ~MetadataExtractor() = default;

  /**
   * \brief Return the name of the extractor.
   * \return The name.
   */
  virtual std::string getName() const = 0;

  /**
   * \brief Return the priority of the extractor (for ordering in MetadataManager).
   * \return The priority. Lower values have more priority. Usually between 0 and 100.
   */
  virtual int getPriority() const = 0;

  /**
   * \brief Get a string describing the complete camera model.
   * \return Camera name.
   */
  virtual cras::optional<std::string> getCameraGeneralName() { return cras::nullopt; }
  /**
   * \brief Get a string describing the camera in a unique way, utilizing serial numbers etc.
   * \return The unique camera name. If unique identification is not possible, nothing is returned.
   */
  virtual cras::optional<std::string> getCameraUniqueName() { return cras::nullopt; }
  /**
   * \brief Get the camera's serial number.
   * \return The serial number.
   */
  virtual cras::optional<std::string> getCameraSerialNumber() { return cras::nullopt; }
  /**
   * \brief Get the camera manufacturer.
   * \return The manufacturer name.
   */
  virtual cras::optional<std::string> getCameraMake() { return cras::nullopt; }
  /**
   * \brief Get the camera model name (just the model, without manufacturer).
   * \return The camera model.
   */
  virtual cras::optional<std::string> getCameraModel() { return cras::nullopt; }
  /**
   * \brief Get the lens manufacturer.
   * \return Lens manufacturer.
   */
  virtual cras::optional<std::string> getLensMake() { return cras::nullopt; }
  /**
   * \brief Get the lens model.
   * \return Lens model.
   */
  virtual cras::optional<std::string> getLensModel() { return cras::nullopt; }
  /**
   * \brief Get the rotation of the image.
   * \return Rotation of the image in degrees. Only values 0, 90, 180 and 270 are supported.
   */
  virtual cras::optional<int> getRotation() { return cras::nullopt; }
  /**
   * \brief Get the global time corresponding to the first frame of the movie.
   * \return Time of movie start.
   */
  virtual cras::optional<ros::Time> getCreationTime() { return cras::nullopt; }
  /**
   * \brief Get crop factor of the camera (i.e. how many times is the sensing area smaller than 36x24 mm).
   * \return The crop factor.
   */
  virtual cras::optional<double> getCropFactor() { return cras::nullopt; }
  /**
   * \brief Get the sensor physical size.
   * \return Sensor size in mm (width, height).
   */
  virtual cras::optional<std::pair<double, double>> getSensorSizeMM() { return cras::nullopt; }
  /**
   * \brief Get the focal length recomputed to an equivalent 35 mm system.
   * \return The focal length 35 mm equivalent.
   */
  virtual cras::optional<double> getFocalLength35MM() { return cras::nullopt; }
  /**
   * \brief Get the focal length in mm.
   * \return The focal length [mm].
   */
  virtual cras::optional<double> getFocalLengthMM() { return cras::nullopt; }
  /**
   * \brief Get the focal length expressed in pixels.
   * \return The focal length [px].
   */
  virtual cras::optional<double> getFocalLengthPx() { return cras::nullopt; }
  /**
   * \brief Get the intrinsic calibration matrix of the camera.
   * \return The intrinsic matrix.
   */
  virtual cras::optional<CI::_K_type> getIntrinsicMatrix() { return cras::nullopt; }
  /**
   * \brief Get the camera distortion coefficients (corresponding to the OpenCV calibration model).
   * \return The distortion coefficients (5 or 8 elements).
   */
  virtual cras::optional<std::pair<CI::_distortion_model_type, CI::_D_type>> getDistortion() { return cras::nullopt; }
  /**
   * \brief Get the GNSS position of the camera when capturing the frame.
   * \return The GNSS position.
   */
  virtual std::pair<cras::optional<sensor_msgs::NavSatFix>, cras::optional<gps_common::GPSFix>> getGNSSPosition()
  {
    return std::make_pair(cras::nullopt, cras::nullopt);
  }
  /**
   * \brief Get the azimuth describing global camera heading when capturing the frame.
   * \return The azimuth.
   */
  virtual cras::optional<compass_msgs::Azimuth> getAzimuth() { return cras::nullopt; }
  /**
   * \brief Get gravity-aligned roll and pitch of the camera when capturing the frame.
   * \return Roll, pitch [rad].
   */
  virtual cras::optional<std::pair<double, double>> getRollPitch() { return cras::nullopt; }
  /**
   * \brief Get the acceleration vector acting on the camera when capturing the frame (including gravity).
   * \return The acceleration vector [m/s^2].
   */
  virtual cras::optional<geometry_msgs::Vector3> getAcceleration() { return cras::nullopt; }

  typedef std::shared_ptr<MetadataExtractor> Ptr;
  typedef std::shared_ptr<const MetadataExtractor> ConstPtr;
};

/**
 * \brief Helper structure that handles instantiation of an extractor.
 */
struct MetadataExtractorPlugin
{
  virtual ~MetadataExtractorPlugin() = default;

  /**
   * \brief Instantiate the extractor with the given parameters.
   * \param[in] params Parameters that configure the extractor.
   * \return An instance of the extractor.
   */
  virtual MetadataExtractor::Ptr getExtractor(const MetadataExtractorParams& params) = 0;
};

}
