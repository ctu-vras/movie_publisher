// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Read a movie or image file.
 * \author Martin Pecka
 */

#pragma once

#include <memory>
#include <string>

#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <gps_common/GPSFix.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

namespace movie_publisher
{

struct MovieReaderPrivate;

/**
 * \brief  Read a movie or image file using libav (ffmpeg) and provide each frame separately by calling nextFrame()
 * generator.
 *
 * Typical usage:
 *
 * <pre>
 * MovieReader reader({new NodeLogHelper()});
 * reader.open("path/to/file.mp4").or_else([](const auto& error) {throw std::runtime_error(error);});
 * // You can call seek() here
 * while (true)
 * {
 *   const auto& [stamp, image] = *reader.nextFrame().or_else([](const auto& error) {throw std::runtime_error(error);});
 *   if (image == nullptr)
 *     break;  // or you can seek to the start if you want to loop
 *   // Process the image
 * }
 * </pre>
 */
class MovieReader : public cras::HasLogger
{
public:
  /**
   * \brief Create the movie reader instance.
   * \param [in] log cras_cpp_common logging helper.
   * \param [in] params ROS/YAML parameters.
   */
  explicit MovieReader(const cras::LogHelperPtr& log, const cras::BoundParamHelperPtr& params);
  virtual ~MovieReader();

  /**
   * \brief How to compute ROS timestamps from movie frame presentation timestamp (PTS).
   */
  enum class TimestampSource
  {
    AllZeros,  //!< ROS timestamp is always 0
    AbsoluteVideoTimecode,  //!< Just use the PTS as is.
    RelativeVideoTimecode,  //!< Use PTS, but make it relative to the time set by last seek() call.
    RosTime,  //!< Use current ROS time.
    FromMetadata,  //!< Use PTS and offset it by the start time recorded in movie metadata.
  };

  /**
   * \brief Open a movie in the referenced file.
   * \param[in] filename Path to the file with the movie.
   * \param[in] timestampSource How to compute timestamps for the frames of the movie.
   * \return On error, an error message is returned.
   */
  virtual cras::expected<void, std::string> open(const std::string& filename, TimestampSource timestampSource);

  /**
   * \brief Open a movie in the referenced file, timestamping the frames with current ROS time.
   * \param[in] filename Path to the file with the movie.
   * \return On error, an error message is returned.
   */
  virtual cras::expected<void, std::string> open(const std::string& filename);

  /**
   * \brief Generator returning next frame of the movie on each call.
   *
   * After open(), this returns the first frame of the movie. After seek(), this returns the first frame after the time
   * it was seeked to.
   *
   * \return The frame and its timestamp. The timestamp depends on what was set by setTimestampSource() and
   *         setTimestampOffset(). On error, an error message is returned. When a `nullptr` is returned for the image,
   *         the movie has reached its end and no more frames will be generated.
   * \note Has to be called after open().
   */
  virtual cras::expected<std::pair<ros::Time, sensor_msgs::ImageConstPtr>, std::string> nextFrame();

  /**
   * \brief Seek the movie to the given time (relative to the start of the movie).
   * \param[in] time The time to seek to.
   * \return Whether the movie can be efficiently seeked, or an error.
   * \note Has to be called after open().
   */
  cras::expected<void, std::string> seek(const ros::Time& time);

  /**
   * \brief Close the movie opened by open(), free all acquired resources.
   */
  virtual void close();

  /**
   * \brief Whether the movie can be efficiently seeked.
   * \return Whether the movie can be efficiently seeked.
   * \note Always returns false before open() is called.
   */
  bool isSeekable() const;

  /**
   * \brief Whether the movie is just a single still image.
   * \return Whether the movie is just a single still image.
   * \note Always returns false before open() is called.
   */
  bool isStillImage() const;

  /**
   * \brief Get framerate of the movie.
   * \return The frame rate.
   * \note Has to be called after open().
   */
  double getFrameRate() const;

  /**
   * \brief Get the duration of the stream.
   * \return The duration.
   * \note Has to be called after open().
   */
  ros::Duration getDuration() const;

  /**
   * \brief Get the start time read from metadata.
   * \return The time (zero if not found).
   */
  ros::Time getMetadataStartTime() const;

  /**
   * \brief Get the number of frames in the movie.
   * \return The number of frames (can be 0 for some formats).
   * \note Has to be called after open().
   */
  size_t getNumFrames() const;

  /**
   * \brief Return a ROS message representing the azimuth under which the shot was taken.
   * \return The azimuth message or nullopt if the information cannot be extracted.
   * \note Has to be called after open().
   * \note The information is valid for the last message for which nexFrame(), open() or seek() was called.
   */
  const cras::optional<compass_msgs::Azimuth>& getAzimuthMsg() const;

  /**
   * \brief Return a ROS message representing the camera info with which the shot was taken.
   * \return The camera info message or nullopt if the information cannot be extracted.
   * \note Has to be called after open().
   * \note The information is valid for the last message for which nexFrame(), open() or seek() was called.
   */
  const cras::optional<sensor_msgs::CameraInfo>& getCameraInfoMsg() const;

  /**
   * \brief Return a ROS message representing the GNSS position at which the shot was taken.
   * \return The GNSS position message or nullopt if the information cannot be extracted.
   * \note Has to be called after open().
   * \note The information is valid for the last message for which nexFrame(), open() or seek() was called.
   */
  const cras::optional<sensor_msgs::NavSatFix>& getNavSatFixMsg() const;

  /**
   * \brief Return a ROS message representing the GNSS position at which the shot was taken.
   * \return The GNSS position message or nullopt if the information cannot be extracted.
   * \note Has to be called after open().
   * \note The information is valid for the last message for which nexFrame(), open() or seek() was called.
   */
  const cras::optional<gps_common::GPSFix>& getGpsMsg() const;

  /**
   * \brief Return a ROS message representing the roll and pitch orientation and possibly acceleration.
   * \return The roll and pitch orientation and acceleration or nullopt if the information cannot be extracted.
   * \note Has to be called after open().
   * \note The information is valid for the last message for which nexFrame(), open() or seek() was called.
   */
  const cras::optional<sensor_msgs::Imu>& getImuMsg() const;

  /**
   * \brief Return a ROS message representing the transform from camera body frame to a gravity-aligned frame.
   * \return The gravity-aligned frame transform or nullopt if the information cannot be extracted.
   * \note Has to be called after open().
   * \note The information is valid for the last message for which nexFrame(), open() or seek() was called.
   */
  const cras::optional<geometry_msgs::TransformStamped>& getZeroRollPitchTF() const;

  /**
   * \brief Return a ROS message representing the transform from camera body frame to optical frame.
   * \return The optical frame transform or nullopt if the information cannot be extracted.
   * \note Has to be called after open().
   * \note The information is valid for the last message for which nexFrame(), open() or seek() was called.
   */
  const cras::optional<geometry_msgs::TransformStamped>& getOpticalFrameTF() const;

  /**
   * \brief Set whether YUV*** formats should be decoded to YUV422, or whether the default encoding should be used.
   * \param [in] allowYUVFallback When true, YUV*** formats will be converted to YUV422.
   */
  void setAllowYUVFallback(bool allowYUVFallback);

  /**
   * \brief Set the encoding which should be used for output frames regardless of their source encoding.
   * \param [in] encoding The forced encoding (one of sensor_msgs::image_encodings constants). If empty, no encoding
   *                      will be forced. If the encoding is not suitable (not color, mono or yuv422), nothing happens.
   */
  void forceEncoding(const std::string& encoding);

  /**
   * \brief Set the default encoding which should be used for output frames whose source encoding has no known mapping.
   * \param [in] encoding The default encoding (one of sensor_msgs::image_encodings constants). If the encoding is not
   *                      suitable (not color, mono or yuv422), nothing happens.
   */
  void setDefaultEncoding(const std::string& encoding);

  /**
   * \brief Set the index of the video stream that will be read from the movie file.
   * \param [in] index The index of the stream. If negative, the "best" stream will be automatically selected by libav.
   */
  void setStreamIndex(int index);

  /**
   * \brief Set the `header.frame_id` of the output images.
   * \param [in] frameId The frame ID of geometrical data.
   * \param [in] opticalFrameId The frame ID of image information. If empty, `frameId` will be used.
   */
  void setFrameId(const std::string& frameId, const std::string& opticalFrameId);

  /**
   * \brief Set the number of threads used for video decoding.
   *
   * There is no clear answer to what number will give the best results. You have to try. The default is 1.
   *
   * \param [in] numThreads The number of threads.
   */
  void setNumThreads(size_t numThreads);

  /**
   * \brief Set the method used for converting frame presentation timestamp into a ROS timestamp.
   * \param [in] source How to compute timestamps for the frames of the movie.
   */
  void setTimestampSource(const TimestampSource& source);

  /**
   * \brief Set the offset of the computed ROS timestamps relative to the reference time.
   * \param [in] offset The offset.
   */
  void setTimestampOffset(const ros::Duration& offset);

private:
  std::unique_ptr<MovieReaderPrivate> data;  //!< PIMPL
};

}
