// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Base for nodes that process movie files.
 * \author Martin Pecka
 */

#pragma once

#include <memory>
#include <string>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/param_utils.hpp>
#include <geometry_msgs/Transform.h>
#include <movie_publisher/movie_reader_ros.h>
#include <sensor_msgs/Image.h>

namespace movie_publisher
{
/**
 * \brief Base for nodes that process movie files.
 */
class MovieProcessorBase : public cras::HasLogger
{
public:
  /**
   * \param[in] log Logger.
   */
  explicit MovieProcessorBase(const cras::LogHelperPtr& log);
  virtual ~MovieProcessorBase();

protected:
  /**
   * \brief Create an instance of the movie reader.
   * \param[in] params ROS parameters.
   * \return The movie reader instance.
   */
  virtual std::unique_ptr<MovieReaderRos> createReader(const cras::BoundParamHelperPtr& params);

  /**
   * \brief Open the given movie file for reading.
   * \param[in] movieFilename The movie file to open.
   * \param[in] params ROS parameters.
   * \return Nothing or error.
   *
   * Processed ROS parameters:
   * - `verbose` (bool, default false): Whether to print progress messages.
   */
  virtual cras::expected<void, std::string> open(
    const std::string& movieFilename, const cras::BoundParamHelperPtr& params);

  /**
   * \brief Process the frame loaded with this->reader->nextFrame() and all relevant metadata.
   * \param[in] image The decoded movie frame.
   * \param[in] pts The corresponding PTS value (timestamp relative to movie start).
   * \return Nothing or error.
   */
  virtual cras::expected<void, std::string> processFrame(
    const sensor_msgs::ImageConstPtr& image, const ros::Time& pts);

  /**
   * \brief Process the image and its camera info.
   * \param[in] image The decoded movie frame.
   * \param[in] cameraInfoMsg The corresponding camera info (if present).
   * \return Nothing or error.
   */
  virtual cras::expected<void, std::string> processImage(
    const sensor_msgs::ImageConstPtr& image, const cras::optional<sensor_msgs::CameraInfo>& cameraInfoMsg);

  /**
   * \brief Process the azimuth message.
   * \param[in] azimuthMsg Azimuth message.
   */
  virtual void processAzimuth(const compass_msgs::Azimuth& azimuthMsg);

  /**
   * \brief Process the NavSatFix message.
   * \param[in] navSatFixMsg NavSatFix message.
   */
  virtual void processNavSatFix(const sensor_msgs::NavSatFix& navSatFixMsg);

  /**
   * \brief Process the GPSFix message.
   * \param[in] gpsMsg GPSFix message.
   */
  virtual void processGps(const gps_common::GPSFix& gpsMsg);

  /**
   * \brief Process the IMU message.
   * \param[in] imuMsg IMU message.
   */
  virtual void processImu(const sensor_msgs::Imu& imuMsg);

  /**
   * \brief Process the zero roll/pitch TF message.
   * \param[in] zeroRollPitchTfMsg Dynamic TF message.
   */
  virtual void processZeroRollPitchTf(const geometry_msgs::TransformStamped& zeroRollPitchTfMsg);

  /**
   * \brief Process the optical frame TF message.
   * \param[in] opticalTfMsg Static TF message.
   */
  virtual void processOpticalTf(const geometry_msgs::TransformStamped& opticalTfMsg);

  std::unique_ptr<MovieReaderRos> reader;  //!< Movie reader instance.

  //! Last determined optical TF (transform from geometrical to optical frame).
  geometry_msgs::Transform lastOpticalTf;

  size_t frameNum {0};  //!< The number of the currently processed frame.

  bool verbose {false};  //!< Whether to print detailed info about playback.
};

}
