// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Types of metadata.
 * \author Martin Pecka
 */

#pragma once

#include <utility>

#include <gps_common/GPSFix.h>
#include <movie_publisher/types.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>

namespace movie_publisher
{

/**
 * \brief Enum for supported metadata.
 */
enum class MetadataType
{
  CAMERA_GENERAL_NAME,
  CAMERA_UNIQUE_NAME,
  CAMERA_SERIAL_NUMBER,
  CAMERA_MAKE,
  CAMERA_MODEL,
  LENS_MAKE,
  LENS_MODEL,
  CREATION_TIME,
  ROTATION,
  CROP_FACTOR,
  SENSOR_SIZE_MM,
  FOCAL_LENGTH_35MM,
  FOCAL_LENGTH_MM,
  FOCAL_LENGTH_PX,
  INTRINSIC_MATRIX,
  DISTORTION,
  GNSS_POSITION,
  AZIMUTH,
  MAGNETIC_FIELD,
  ROLL_PITCH,
  ACCELERATION,
  ANGULAR_VELOCITY,
  FACES,
  CAMERA_INFO,
  IMU,
  OPTICAL_FRAME_TF,
  ZERO_ROLL_PITCH_TF,
};

using IntrinsicMatrix = sensor_msgs::CameraInfo::_K_type;
using DistortionType = sensor_msgs::CameraInfo::_distortion_model_type;
using Distortion = sensor_msgs::CameraInfo::_D_type;
using DistortionData = std::pair<DistortionType, Distortion>;
using GNSSFixAndDetail = std::pair<cras::optional<sensor_msgs::NavSatFix>, cras::optional<gps_common::GPSFix>>;
using SensorSize = std::pair<double, double>;
using RollPitch = std::pair<double, double>;



/**
 * \brief Timestamping wrapper for any kind of metadata.
 * \tparam T Type of metadata.
 */
template<typename T>
struct TimedMetadata
{
  using value_type = T;  //!< Type of the metadata values.
  StreamTime stamp;  //!< Timestamp of the data (in stream time).
  T value;  //!< The stored metadata.
};

}
