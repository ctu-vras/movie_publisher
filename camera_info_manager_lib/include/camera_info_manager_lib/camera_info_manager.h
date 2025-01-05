// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Jack O'Quin
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief camera_info_manager API without an active ROS node.
 * \author Martin Pecka
 */

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010-2012 Jack O'Quin
*  Copyright (c) 2024 Czech Technical University in Prague
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#pragma once

#include <string>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/optional.hpp>
#include <sensor_msgs/CameraInfo.h>

namespace camera_info_manager_lib
{
/**
 * \brief Provides CameraInfo data for a calibrated camera. Different from camera_info_manager::CameraInfoManager,
 *        this class provides only the C++ API and not the ROS API (set_camera_info service).
 *
 * \par Camera Name
 *
 * The device driver sets a camera name via the CameraInfoManager() constructor or the setCameraName() method.
 * This name is written when CameraInfo is saved, and checked when data are loaded, with a warning logged if the name
 * read does not match.
 *
 * Syntax: a camera name contains any combination of alphabetic, numeric and '_' characters.  Case is significant.
 *
 * Camera drivers may use any syntactically valid name they please. Where possible, it is best for the name to be
 * unique to the device, such as a GUID, or the make, model and serial number. Any parameters that affect calibration,
 * such as resolution, focus, zoom, etc., may also be included in the name, uniquely identifying each CameraInfo file.
 *
 * The camera name can be resolved as part of the URL, allowing direct access to device-specific calibration
 * information.
 *
 * \par Uniform Resource Locator
 *
 * The location for getting and saving calibration data is expressed by Uniform Resource Locator. The driver defines a
 * URL via the CameraInfoManager() constructor or the loadCameraInfo() method. Many drivers provide a
 * \c ~camera_info_url parameter so users may customize this URL, but that is handled outside this class.
 *
 * Typically, cameras store calibration information in a file, which can be in any format supported by
 * \c camera_calibration_parsers . Currently, that includes YAML and Videre INI files, identified by their .yaml or
 * .ini extensions as shown in the examples. These file formats are described here:
 *
 * - http://www.ros.org/wiki/camera_calibration_parsers#File_formats
 *
 * Example URL syntax:
 *
 * - file:///full/path/to/local/file.yaml
 * - file:///full/path/to/videre/file.ini
 * - package://camera_info_manager/tests/test_calibration.yaml
 * - package://ros_package_name/calibrations/camera3.yaml
 *
 * The \c file: URL specifies a full path name in the local system. The \c package: URL is handled the same as
 * \c file:, except the path name is resolved relative to the location of the named ROS package, which \em must be
 * reachable via \c $ROS_PACKAGE_PATH.
 *
 * The URL may contain substitution variables delimited by <tt>${...}</tt>, including:
 *
 * - <tt>${NAME}</tt> resolved to the current camera name defined by the device driver.
 * - <tt>${ROS_HOME}</tt> resolved to the \c $ROS_HOME environment variable if defined, or <tt>~/.ros</tt> if not.
 * - <tt>${FOCAL_LENGTH}</tt> resolved to the focal length of the camera. By default, the focal length is formatted by
 *   string <tt>%0.01fmm</tt>, but it can be changed by specifying another format string, e.g.
 *   <tt>${FOCAL_LENGTH:%0.04f_mm}</tt>.
 *
 * Resolution is done in a single pass through the URL string. Variable values containing substitutable strings are not
 * resolved recursively. Unrecognized variable names are treated literally with no substitution, but an error is
 * logged.
 *
 * Examples with variable substitution:
 *
 * - package://my_cameras/calibrations/${NAME}.yaml
 * - file://${ROS_HOME}/camera_info/left_front_camera.yaml
 *
 * The default URL is:
 *
 * - file://${ROS_HOME}/camera_info/${NAME}.yaml.
 *
 * If that file exists, its contents are used. Any new calibration will be stored there, missing parent directories
 * being created if necessary and possible.
 *
 * \par Loading Calibration Data
 *
 * CameraInfoManager loads nothing until the &c loadCameraInfo(), &c isCalibrated() or &c getCameraInfo() method is
 * called. That suppresses bogus error messages, but allows (valid) load errors to occur during the first
 * \c getCameraInfo(), or \c isCalibrated(). To avoid that, do an explicit \c loadCameraInfo() first.
 */
class CameraInfoManager : public cras::HasLogger
{
public:
  /**
   * \brief Constructor. It does not load the calibration file, that is done lazily when needed.
   * \param[in] cname Name of the camera (used in URL).
   * \param[in] url The URL specifying path to the calibration file (substitution keywords can be used).
   */
  explicit CameraInfoManager(const std::string& cname = "camera", const std::string& url = "");

  /**
   * \brief Constructor. It does not load the calibration file, that is done lazily when needed.
   * \param[in] log Logger.
   * \param[in] cname Name of the camera (used in URL).
   * \param[in] url The URL specifying path to the calibration file (substitution keywords can be used).
   */
  explicit CameraInfoManager(
    const cras::LogHelperPtr& log, const std::string& cname = "camera", const std::string& url = "");

  virtual ~CameraInfoManager();

  /**
   * \brief Load and return the camera info assigned to the camera specified in constructor or by calls to
   *        setCameraName() and setFocalLength().
   * \return The camera info.
   */
  sensor_msgs::CameraInfo getCameraInfo();

  /**
   * \return Whether camera info is available and nonzero for the specified camera.
   */
  bool isCalibrated();

  /**
   * \brief Resolve substitution keywords in the given URL.
   * \param[in] log Logger.
   * \param[in] url The URL to resolve.
   * \param[in] cname Camera name.
   * \param[in] focalLength Focal length [mm].
   * \return The resolved URL.
   */
  static std::string resolveURL(const cras::LogHelperPtr& log,
    const std::string& url, const std::string& cname, const cras::optional<double>& focalLength);

  /**
   * \brief Validate a camera info URL.
   * \param[in] url The URL to validate.
   * \return True if URL syntax is supported by CameraInfoManager (although the resource need not actually exist).
   */
  static bool validateURL(const std::string& url);

  /**
   * \brief Set or change the name of the camera that should currently be represented by this class.
   * \param[in] cname New camera name.
   * \return Whether the new name is valid and was used.
   */
  virtual bool setCameraName(const std::string& cname);

  /**
   * \brief Set or change the focal length of the camera that should currently be represented by this class.
   * \param[in] focalLength New focal length [mm].
   * \return Whether the focal length is valid and was used (negative values are invalid).
   */
  virtual bool setFocalLength(double focalLength);

protected:
  //! recognized URL types
  enum class URL
  {
    EMPTY = 0,  //!< empty string
    FILE,  //!< file:
    PACKAGE,  //!< package:
    // URLs not supported
    INVALID,  //!< anything >= is invalid
    FLASH,  //!< flash:
  };

  /**
   * \brief Parse the type of the URL.
   * \param[in] url The URL to parse.
   * \return The URL type.
   */
  static URL parseURL(const std::string& url);

  /**
   * \brief Load calibration according to camera info URL.
   * \param[in] url Camera info URL (can contain substitution keywords).
   * \param[in] cname Name of the camera.
   * \param[in] focalLength Focal length of the camera [mm].
   * \return Whether the calibration has been found and loaded.
   */
  virtual bool loadCalibration(
    const std::string& url, const std::string& cname, const cras::optional<double>& focalLength);

  /**
   * \brief Load calibration from the given file.
   * \param[in] filename Calibration file.
   * \param[in] cname Camera name.
   * \return Whether the calibration has been found and loaded.
   */
  virtual bool loadCalibrationFile(const std::string& filename, const std::string& cname);

  /**
   * \brief Load calibration from camera flash (not supported, but subclasses can override).
   * \param[in] flashURL Flash URL.
   * \param[in] cname Camera name.
   * \return Whether the calibration has been found and loaded.
   */
  virtual bool loadCalibrationFlash(const std::string& flashURL, const std::string& cname);

private:
  /**
   * \brief Resolve the <tt>package://</tt> part of URL with a local file.
   * \param[in] url The <tt>package://</tt> URL.
   * \return The URL resolved to a local file.
   */
  std::string getPackageFileName(const std::string& url) const;

protected:
  std::string cameraName;  //!< Camera name.
  cras::optional<double> focalLength;  //!< Current focal length.
  std::string url;  //!< URL for calibration data
  sensor_msgs::CameraInfo camInfo {};  //!< Current CameraInfo.
  bool loadedCamInfo;  //!< Whether camInfo load has been attempted.
};

}
