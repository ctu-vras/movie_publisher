// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Base for consumers of movie metadata.
 * \author Martin Pecka
 */

#pragma once

#include <memory>
#include <string>

#include <cras_cpp_common/expected.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <gps_common/GPSFix.h>
#include <movie_publisher/metadata_extractor.h>
#include <movie_publisher/movie_info.h>
#include <movie_publisher/movie_open_config.h>
#include <movie_publisher/movie_playback_state.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <vision_msgs/Detection2DArray.h>

namespace movie_publisher
{
/**
 * \brief Base for consumers of movie metadata.
 *
 * Callbacks for each type of metadata will be called when the metadata is available and the movie advances.
 *
 * \note The difference between TimedMetadataListener and MovieMetadataProcessor is the level of abstraction on which
 *       they work. TimedMetadataListener connects Movie and TimedMetadataExtractors so that the Movie class gets info
 *       about the metadata it has. MovieMetadataProcessor is then the interface between Movie and "the outer world",
 *       i.e. users of the Movie class. Usually, TimedMetadataListener will be mostly used internally in this library,
 *       while MovieMetadataProcessor will be used publicly by other classes outside this package. Also, the timestamps
 *       in TimedMetadataListener are always stream timestamps, while in MovieMetadataProcessor they should already
 *       correspond to ROS time according to the defined conversion function.
 */
class MovieMetadataProcessor
{
public:
  virtual ~MovieMetadataProcessor();

  /**
   * \brief Callback telling the processor that a movie has been opened for reading.
   * \param[in] info Information about the open movie.
   * \param[in] config Configuration with which the movie has been opened.
   * \return Nothing or error.
   */
  virtual cras::expected<void, std::string> onOpen(const MovieInfo::ConstPtr& info, const MovieOpenConfig& config);

  /**
   * \brief Callback telling the processor that static metadata have been extracted and timed metadata are ready.
   * \param[in] metadataExtractor The object via which metadata can be retrieved.
   * \return Nothing or error.
   */
  virtual cras::expected<void, std::string> onMetadataReady(
    const std::shared_ptr<TimedMetadataExtractor>& metadataExtractor);

  /**
   * \brief Callback telling the processor that a movie has been closed.
   * \return Nothing or error.
   */
  virtual cras::expected<void, std::string> onClose();

  /**
   * \brief Callback telling the processor that a movie has been seeked to the given time.
   * \param[in] time The stream time the movie is seeked to.
   * \return Nothing or error.
   */
  virtual cras::expected<void, std::string> onSeek(const StreamTime& time) {return {};}

  /**
   * \brief Process the frame read by MovieReader::nextFrame().
   * \param[in] image The decoded movie frame.
   * \param[in] playbackState Current state of movie playback.
   * \return Nothing or error.
   */
  virtual cras::expected<void, std::string> processFrame(
    const sensor_msgs::ImageConstPtr& image, const MoviePlaybackState& playbackState);

  /**
   * \brief Process the image and its camera info.
   * \param[in] image The decoded movie frame.
   * \param[in] cameraInfoMsg The corresponding camera info (if present).
   * \return Nothing or error.
   * \note This function is not called directly by the metadata manager (because it doesn't have access to the image),
   *       but is instead called from processFrame(). If you override processFrame(), make sure the call to
   *       processImage() is still propagated.
   */
  virtual cras::expected<void, std::string> processImage(
    const sensor_msgs::ImageConstPtr& image, const cras::optional<sensor_msgs::CameraInfo>& cameraInfoMsg) {return {};}

  /**
   * \brief Process camera info.
   * \param[in] cameraInfoMsg The camera info.
   * \return Nothing or error.
   */
  virtual cras::expected<void, std::string> processCameraInfo(const sensor_msgs::CameraInfo& cameraInfoMsg) {return {};}

  /**
   * \brief Process the azimuth message.
   * \param[in] azimuthMsg Azimuth message.
   */
  virtual cras::expected<void, std::string> processAzimuth(const compass_msgs::Azimuth& azimuthMsg) {return {};}

  /**
   * \brief Process the NavSatFix message.
   * \param[in] navSatFixMsg NavSatFix message.
   */
  virtual cras::expected<void, std::string> processNavSatFix(const sensor_msgs::NavSatFix& navSatFixMsg) {return {};}

  /**
   * \brief Process the GPSFix message.
   * \param[in] gpsMsg GPSFix message.
   */
  virtual cras::expected<void, std::string> processGps(const gps_common::GPSFix& gpsMsg) {return {};}

  /**
   * \brief Process the IMU message.
   * \param[in] imuMsg IMU message.
   */
  virtual cras::expected<void, std::string> processImu(const sensor_msgs::Imu& imuMsg) {return {};}

  /**
   * \brief Process the zero roll/pitch TF message.
   * \param[in] zeroRollPitchTfMsg Dynamic TF message.
   */
  virtual cras::expected<void, std::string> processZeroRollPitchTf(
    const geometry_msgs::TransformStamped& zeroRollPitchTfMsg) {return {};}

  /**
   * \brief Process the optical frame TF message.
   * \param[in] opticalTfMsg Static TF message.
   */
  virtual cras::expected<void, std::string> processOpticalTf(const geometry_msgs::TransformStamped& opticalTfMsg)
  {
    return {};
  }

  /**
   * \brief Process the magnetic field message.
   * \param[in] magneticFieldMsg Magnetic field message.
   */
  virtual cras::expected<void, std::string> processMagneticField(const sensor_msgs::MagneticField& magneticFieldMsg)
  {
    return {};
  }

  /**
   * \brief Process the face detections message.
   * \param[in] facesMsg Message with face detections.
   */
  virtual cras::expected<void, std::string> processFaces(const vision_msgs::Detection2DArray& facesMsg) {return {};}

protected:
  MovieInfo::ConstPtr info;  //!< Information about the last opened movie.
  cras::optional<MovieOpenConfig> config;  //!< Configuration of the last opened movie.
  std::shared_ptr<MetadataExtractor> metadataExtractor;  //!< Accessor to static metadata of the last opened movie.
  bool verbose {false};  //!< Whether the processor should be verbose.
};

}
