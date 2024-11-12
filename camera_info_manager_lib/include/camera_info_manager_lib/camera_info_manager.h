// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Jack O'Quin
// SPDX-FileCopyrightText: Czech Technical University in Prague

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

class CameraInfoManager : public cras::HasLogger
{
 public:
  explicit CameraInfoManager(const std::string& cname = "camera", const std::string& url = "");
  explicit CameraInfoManager(
    const cras::LogHelperPtr& log, const std::string& cname = "camera", const std::string& url = "");

  sensor_msgs::CameraInfo getCameraInfo();
  bool isCalibrated();
  static std::string resolveURL(
    const std::string& url, const std::string& cname, const cras::optional<double>& focal_length);
  virtual bool setCameraName(const std::string& cname);
  virtual bool setFocalLength(double focal_length);
  static bool validateURL(const std::string& url);

 private:
  // recognized URL types
  typedef enum
    {
      // supported URLs
      URL_empty = 0,             // empty string
      URL_file,                  // file:
      URL_package,               // package:
      // URLs not supported
      URL_invalid,               // anything >= is invalid
      URL_flash,                 // flash:
    } url_type_t;

  // private methods
  std::string getPackageFileName(const std::string& url) const;
  bool loadCalibration(const std::string& url, const std::string& cname, const cras::optional<double>& focal_length);
  bool loadCalibrationFile(const std::string& filename, const std::string& cname);
  bool loadCalibrationFlash(const std::string& flashURL, const std::string& cname);
  static url_type_t parseURL(const std::string& url);

  // private data
  std::string camera_name_;             ///< camera name
  cras::optional<double> focal_length_;
  std::string url_;                     ///< URL for calibration data
  sensor_msgs::CameraInfo cam_info_;    ///< current CameraInfo
  bool loaded_cam_info_;                ///< cam_info_ load attempted
};

}
