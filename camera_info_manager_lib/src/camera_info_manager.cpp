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

#include <locale>
#include <memory>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>

#include <ros/package.h>
#include <boost/algorithm/string.hpp>

#include <camera_calibration_parsers/parse.h>
#include <camera_info_manager_lib/camera_info_manager.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/string_utils.hpp>

namespace camera_info_manager_lib
{

/** URL to use when no other is defined. */
const char default_camera_info_url[] = "file://${ROS_HOME}/camera_info/${NAME}.yaml";

CameraInfoManager::CameraInfoManager(const cras::LogHelperPtr& log, const std::string& cname, const std::string& url) :
  cras::HasLogger(log), camera_name_(cname), url_(url), loaded_cam_info_(false)
{
}

CameraInfoManager::CameraInfoManager(const std::string& cname, const std::string& url) :
  CameraInfoManager(std::make_shared<cras::NodeLogHelper>(), cname, url)
{
}

sensor_msgs::CameraInfo CameraInfoManager::getCameraInfo()
{
  if (!loaded_cam_info_)
  {
    loaded_cam_info_ = true;
    loadCalibration(url_, camera_name_, focal_length_);
  }

  return cam_info_;
}

std::string CameraInfoManager::getPackageFileName(const std::string& url) const
{
  CRAS_DEBUG_STREAM("camera calibration URL: " << url);

  // Scan URL from after "package://" until next '/' and extract
  // package name.  The parseURL() already checked that it's present.
  size_t prefix_len = std::string("package://").length();
  size_t rest = url.find('/', prefix_len);
  std::string package(url.substr(prefix_len, rest - prefix_len));

  // Look up the ROS package path name.
  std::string pkgPath(ros::package::getPath(package));
  if (pkgPath.empty())                  // package not found?
    {
      CRAS_WARN_STREAM("unknown package: " << package << " (ignored)");
      return pkgPath;
    }
  else
    {
      // Construct file name from package location and remainder of URL.
      return pkgPath + url.substr(rest);
    }
}

bool CameraInfoManager::isCalibrated()
{
  if (!loaded_cam_info_)
  {
    // load being attempted now
    loaded_cam_info_ = true;

    // attempt load without the lock, it is not recursive
    loadCalibration(url_, camera_name_, focal_length_);
  }

  return (cam_info_.K[0] != 0.0);
}

bool CameraInfoManager::loadCalibration(
  const std::string& url, const std::string& cname, const cras::optional<double>& focal_length)
{
  bool success = false;                 // return value

  const std::string resURL(resolveURL(url, cname, focal_length));
  url_type_t url_type = parseURL(resURL);

  if (url_type != URL_empty)
    {
      CRAS_DEBUG_STREAM("camera calibration URL: " << resURL);
    }

  switch (url_type)
    {
    case URL_empty:
      {
        CRAS_DEBUG("using default calibration URL");
        success = loadCalibration(default_camera_info_url, cname, focal_length);
        break;
      }
    case URL_file:
      {
        success = loadCalibrationFile(resURL.substr(7), cname);
        break;
      }
    case URL_flash:
      {
        success = loadCalibrationFlash(resURL.substr(8), cname);
        break;
      }
    case URL_package:
      {
        std::string filename(getPackageFileName(resURL));
        if (!filename.empty())
          success = loadCalibrationFile(filename, cname);
        break;
      }
    default:
      {
        CRAS_ERROR_STREAM("Invalid camera calibration URL: " << resURL);
        break;
      }
    }

  return success;
}

bool CameraInfoManager::loadCalibrationFile(const std::string& filename, const std::string& cname)
{
  bool success = false;

  CRAS_DEBUG_STREAM("reading camera calibration from " << filename);
  std::string cam_name;
  sensor_msgs::CameraInfo cam_info;

  if (camera_calibration_parsers::readCalibration(filename, cam_name, cam_info))
    {
      if (cname != cam_name)
        {
          CRAS_WARN_STREAM("[" << cname << "] does not match name " << cam_name << " in file " << filename);
        }
      success = true;
      {
        cam_info_ = cam_info;
      }
    }
  else
    {
      CRAS_DEBUG_STREAM("Camera calibration file " << filename << " not found.");
    }

  return success;
}

bool CameraInfoManager::loadCalibrationFlash(const std::string& flashURL, const std::string& cname)
{
  CRAS_WARN("[CameraInfoManager] reading from flash not implemented for this CameraInfoManager");
  return false;
}

std::string CameraInfoManager::resolveURL(
  const std::string& url, const std::string& cname, const cras::optional<double>& focal_length)
{
  std::string resolved;
  size_t rest = 0;

  while (true)
    {
      // find the next '$' in the URL string
      size_t dollar  = url.find('$', rest);

      if (dollar >= url.length())
        {
          // no more variables left in the URL
          resolved += url.substr(rest);
          break;
        }

      // copy characters up to the next '$'
      resolved += url.substr(rest, dollar-rest);

      if (url.substr(dollar+1, 1) != "{")
        {
          // no '{' follows, so keep the '$'
          resolved += "$";
        }
      else if (url.substr(dollar+1, 6) == "{NAME}")
        {
          // substitute camera name
          resolved += cname;
          dollar += 6;
        }
      else if (url.substr(dollar+1, 10) == "{ROS_HOME}")
        {
          // substitute $ROS_HOME
          std::string ros_home;
          char *ros_home_env;
          if ((ros_home_env = getenv("ROS_HOME")))
            {
              // use environment variable
              ros_home = ros_home_env;
            }
          else if ((ros_home_env = getenv("HOME")))
            {
              // use "$HOME/.ros"
              ros_home = ros_home_env;
              ros_home += "/.ros";
            }
          resolved += ros_home;
          dollar += 10;
        }
      else if (url.substr(dollar+1, 13) == "{FOCAL_LENGTH")
        {
          size_t end  = url.find('}', dollar+14);
          if (end >= url.length())
          {
            // no matching }, so just finish up
            resolved += url.substr(rest);
            break;
          }
          auto format = url.substr(dollar+14, end-dollar-14);
          if (format.empty())
            format = "%.01f mm";
          else
            format = format.substr(1);  // remove the :
          resolved += cras::format(format.c_str(), focal_length);
          dollar = end;
        }
      else
        {
          // not a valid substitution variable
          ROS_ERROR_STREAM("[CameraInfoManager]"
                           " invalid URL substitution (not resolved): "
                           << url);
          resolved += "$";            // keep the bogus '$'
        }

      // look for next '$'
      rest = dollar + 1;
    }

  return resolved;
}

CameraInfoManager::url_type_t CameraInfoManager::parseURL(const std::string& url)
{
  if (url == "")
    {
      return URL_empty;
    }
  if (boost::iequals(url.substr(0, 7), "file://"))
    {
      return URL_file;
    }
  if (boost::iequals(url.substr(0, 8), "flash://"))
    {
      return URL_flash;
    }
  if (boost::iequals(url.substr(0, 10), "package://"))
    {
      // look for a '/' following the package name, make sure it is
      // there, the name is not empty, and something follows it
      size_t rest = url.find('/', 10);
      if (rest < url.length()-1 && rest > 10)
        return URL_package;
    }
  return URL_invalid;
}

bool CameraInfoManager::setCameraName(const std::string& cname)
{
  // the camera name may not be empty
  if (cname.empty())
    return false;

  if (cname == camera_name_)
    return true;

  // validate the camera name characters
  for (unsigned i = 0; i < cname.size(); ++i)
    {
      if (!isalnum(cname[i]) && cname[i] != '_')
        return false;
    }

  // The name is valid, so update our private copy.  Since the new
  // name might cause the existing URL to resolve somewhere else,
  // force @c cam_info_ to be reloaded before being used again.
  camera_name_ = cname;
  loaded_cam_info_ = false;

  return true;
}

bool CameraInfoManager::setFocalLength(double focal_length)
{
  if (focal_length <= 0)
    return false;

  if (focal_length_.has_value() && *focal_length_ == focal_length)
    return true;

  focal_length_ = focal_length;
  loaded_cam_info_ = false;

  return true;
}

/** Validate URL syntax.
 *
 * @param url Uniform Resource Locator to check
 *
 * @return true if URL syntax is supported by CameraInfoManager
 *              (although the resource need not actually exist)
 */
bool CameraInfoManager::validateURL(const std::string& url)
{
  url_type_t url_type = parseURL(url);
  return (url_type < URL_invalid);
}

}
