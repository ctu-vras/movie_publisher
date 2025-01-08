// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Convert movie files and their metadata to ROS bag file.
 * \author Martin Pecka
 */

#pragma once

#include <memory>
#include <string>

#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/node_utils/node_with_optional_master.h>
#include <cras_cpp_common/param_utils.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <gps_common/GPSFix.h>
#include <image_transport_codecs/image_transport_codecs.h>
#include <movie_publisher/movie_processor_base.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

namespace movie_publisher
{
/**
 * \brief Convert movie files and their metadata to ROS bag file.
 *
 * \par Stored topics
 *
 * - `${~topic}` (`sensor_msgs/Image`): The published movie (if raw transport is used).
 * - `${~topic}/${~transport}` (*): The published movie compressed stream (if raw transport is not used).
 * - `${~topic}/camera_info` (`sensor_msgs/CameraInfo`): Camera info.
 * - `${~topic}/azimuth` (`compass_msgs/Azimuth`): Georeferenced heading of the camera.
 * - `${~topic}/fix` (`sensor_msgs/NavSatFix`): GNSS position of the camera.
 * - `${~topic}/fix_detail` (`gps_common/GPSFix`): GNSS position of the camera.
 * - `${~topic}/imu` (`sensor_msgs/Imu`): Orientation and acceleration of the camera.
 *
 * To extract the additional topics except `movie`, the node uses instances of MetadataExtractor.
 *
 * To change the prefix of all topics, set `~topic` parameter. To change the name of a single topic, remap it.
 *
 * \par Parameters
 *
 * Parameters `~start`, `~end` and `~duration` can be expressed in seconds `(15.35)`, in `(min, sec)`,
 * in `(hour, min, sec)`, or as a string: `'01:03:05.35'`.
 *
 * - `~overwrite_bag` (bool, default false): If true and the bag file exists, it will be overwritten. Otherwise, it will
 *                                           be appended (and created if needed).
 * - `~start` (float|tuple|string, optional): If set, the movie will be read from the specified time.
 *                                            Cannot be set together with `~end` and `~duration`.
 * - `~end` (float|tuple|string, optional): If set, the movie will be read up to the specified time (not affected by
 *                                          start). Cannot be set together with `~start` and `~duration`.
 * - `~duration` (float|tuple|string, optional): If set, playback will have this duration. If end is also set, the
 *                                               duration is counted from the end of the clip, otherwise, it is the
 *                                               duration from the start of the clip. Cannot be set together with
 *                                               `~start` and `~end`.
 * - `~timestamp_offset` (int|float|string, default 0.0): Adjustment of timestamps determined by `~timestamp_source`.
 *                                                        If given as string, it can be a simple mathematical expression
 *                                                        that can also resolve several variables:
 *                                                        `ros_time` (current ROS time),
 *                                                        `wall_time` (current wall time),
 *                                                        `metadata_start` (start time from metadata),
 *                                                        `bag_start` (start time of the bag file),
 *                                                        `bag_end` (end time of the metadata),
 *                                                        `bag_duration` (duration of the bag file in s).
 * - `~timestamp_source` (str, default `metadata`): How to determine timestamps of the movie frames. Options are:
 *   - `metadata`: Extract absolute time when the movie was recorded and use that time as timestamps.
 *   - `all_zeros`: Use zero timestamps. Please note that time 0.0 cannot be stored in bag files. Use
 *                  `~timestamp_offset` to make the time valid.
 *   - `absolute_timecode`: Use the absolute timecode as timestamps (i.e. time since start of movie file).
 *   - `relative_timecode`: Use the relative timecode as timestamps (i.e. time since `~start`).
 *   - `ros_time`: Timestamp the frames with current ROS time. Note that this mode is not very useful for movie_to_bag.
 * - `~frame_id` (string, default ""): The frame_id used in the geometrical messages' headers.
 * - `~optical_frame_id` (string, default `${frame_id}_optical_frame`): The frame_id used in the image messages'
 *                                                                      headers.
 * - `~verbose` (bool, default False): If True, logs info about every frame played.
 * - `~allow_yuv_fallback` (bool, default False): Set whether `YUV***` formats should be decoded to YUV422, or whether
 *                                                the default encoding should be used.
 * - `~default_encoding` (string, optional): Set the default encoding which should be used for output frames if there is
 *                                           no direct match between the libav pixel format and ROS image encodings.
 * - `~encoding` (string, optional): Set the encoding which should be used for output frames regardless of their source
 *                                   encoding (one of sensor_msgs::image_encodings constants).
 */
class MovieToBag : public cras::NodeWithOptionalMaster, protected MovieProcessorBase
{
public:
  /**
   * \param[in] log Logger.
   */
  explicit MovieToBag(const cras::LogHelperPtr& log);

  /**
   * \brief Open the given movie file for reading and bag file for writing.
   * \param[in] bagFilename The bag filename.
   * \param[in] transport The image transport to use for saving movie frames.
   * \param[in] movieFilename The movie file.
   * \param[in] params ROS parameters.
   * \return Nothing or error.
   *
   * Processed ROS parameters:
   * - `verbose` (bool, default false): Whether to print progress messages.
   * - `topic` (string, default 'movie'): Base name of the topic with movie frames (without transport suffix).
   */
  virtual cras::expected<void, std::string> open(
    const std::string& bagFilename, const std::string& transport, const std::string& movieFilename,
    const cras::BoundParamHelperPtr& params);

  /**
   * \brief Run the conversion.
   * \return Nothing or error.
   */
  virtual cras::expected<void, std::string> run();

protected:
  cras::LogHelperConstPtr getCrasLogger() const;

  std::unique_ptr<MovieReaderRos> createReader(const cras::BoundParamHelperPtr& params) override;

  virtual std::string getImageTopic() const;
  virtual std::string getCameraInfoTopic() const;
  virtual std::string getAzimuthTopic() const;
  virtual std::string getNavSatFixTopic() const;
  virtual std::string getGpsTopic() const;
  virtual std::string getImuTopic() const;
  virtual std::string getTfTopic() const;
  virtual std::string getStaticTfTopic() const;

  /**
   * \brief Prefix the given topic name with the base topic.
   * \param[in] topicName Suffix to add to the base topic name.
   * \return The prefixed topic name.
   */
  virtual std::string getPrefixedTopic(const std::string& topicName) const;

  cras::expected<void, std::string> processImage(
    const sensor_msgs::ImageConstPtr& image, const cras::optional<sensor_msgs::CameraInfo>& cameraInfoMsg) override;
  void processAzimuth(const compass_msgs::Azimuth& azimuthMsg) override;
  void processNavSatFix(const sensor_msgs::NavSatFix& navSatFixMsg) override;
  void processGps(const gps_common::GPSFix& gpsMsg) override;
  void processImu(const sensor_msgs::Imu& imuMsg) override;
  void processZeroRollPitchTf(const geometry_msgs::TransformStamped& zeroRollPitchTfMsg) override;
  void processOpticalTf(const geometry_msgs::TransformStamped& opticalTfMsg) override;

  std::unique_ptr<image_transport_codecs::ImageTransportCodecs> imageCodecs;  //!< Image transport codec instance.
  std::unique_ptr<rosbag::Bag> bag;  //!< The bag to write to.

  std::string topic;  //!< The base topic name.
  std::string transport;  //!< The transport to use.
};

}
