// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief MovieReader preconfigured from ROS parameters.
 * \author Martin Pecka
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/param_utils.hpp>
#include <movie_publisher/movie_reader.h>
#include <ros/time.h>

namespace movie_publisher
{
/**
 * \brief MovieReader preconfigured from ROS parameters.
 *
 * \par Parameters
 *
 * Parameters `~start`, `~end` and `~duration` can be expressed in seconds `(15.35)`, in `(min, sec)`,
 * in `(hour, min, sec)`, or as a string: `'01:03:05.35'`.
 *
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
 *                                                        `metadata_start` (start time from metadata).
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
 * - `~allow_yuv_fallback` (bool, default False): Set whether `YUV***` formats should be decoded to YUV422, or whether
 *                                                the default encoding should be used.
 * - `~default_encoding` (string, optional): Set the default encoding which should be used for output frames if there is
 *                                           no direct match between the libav pixel format and ROS image encodings.
 * - `~encoding` (string, optional): Set the encoding which should be used for output frames regardless of their source
 *                                   encoding (one of sensor_msgs::image_encodings constants).
 */
class MovieReaderRos : public MovieReader
{
public:
  MovieReaderRos(const cras::LogHelperPtr& log, const cras::BoundParamHelperPtr& params);

  cras::expected<void, std::string> open(const std::string& filename, TimestampSource timestampSource) override;
  cras::expected<void, std::string> open(const std::string& filename) override;

  /**
   * \brief Add a variable that will be resolved when parsing timestamp_offset.
   * \param[in] var Name of the variable.
   * \param[in] val The value of the variable.
   * \remark This should be called before open().
   */
  void addTimestampOffsetVar(const std::string& var, double val);

  /**
   * \brief Start time (relative to movie start).
   */
  virtual ros::Time getStart() const;

  /**
   * \brief End time (relative to movie start).
   */
  virtual ros::Time getEnd() const;

  /**
   * \brief Geometrical frame_id.
   */
  virtual std::string getFrameId() const;

  /**
   * \brief Optical frame_id.
   */
  virtual std::string getOpticalFrameId() const;

protected:
  cras::BoundParamHelperPtr params;  //!< ROS parameters configuring the movie reader.
  //! Extra variables to resolve in the timestamp_offset expression.
  std::unordered_map<std::string, double> timestampOffsetVars;

  ros::Time start;  //!< Start time (relative to movie start).
  ros::Time end;  //!< End time (relative to movie start).

  std::string frameId;  //!< Geometrical frame_id.
  std::string opticalFrameId;  //!< Optical frame_id.
};

}
