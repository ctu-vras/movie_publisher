// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Extractor of metadata from sidecar YAML files.
 * \author Martin Pecka
 */

#include <boost/array.hpp>
#include <yaml-cpp/yaml.h>

#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/string_utils/ros.hpp>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <gps_common/GPSFix.h>
#include <gps_common/GPSStatus.h>
#include <movie_publisher/metadata_type.h>
#include <ros/time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <vision_msgs/BoundingBox2D.h>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace YAML
{

template <typename T, typename YAMLType = T, typename std::enable_if_t<std::is_same<T, YAMLType>::value>* = nullptr>
void updateFromYAML(const Node& node, T& dest)
{
  if (!node)
    throw InvalidNode();
  if (!convert<YAMLType>::decode(node, dest))
    throw TypedBadConversion<T>(node.Mark());
}

template <typename T, typename YAMLType, typename std::enable_if_t<!std::is_same<T, YAMLType>::value>* = nullptr>
void updateFromYAML(const Node& node, T& dest)
{
  if (!node)
    throw InvalidNode();
  auto val = static_cast<YAMLType>(dest);
  if (!convert<YAMLType>::decode(node, val))
    throw TypedBadConversion<T>(node.Mark());
  dest = static_cast<T>(val);
}

template<typename T, typename YAMLType = T>
void fromYAML(const Node& node, const std::string& key, T& dest)
{
  if (node[key])
  {
    try
    {
      updateFromYAML<T, YAMLType>(node[key], dest);
    }
    catch (const YAML::Exception& e)
    {
      ROS_ERROR_NAMED("yaml", "Error parsing YAML: %s", e.what());
    }
  }
}

template<typename T, typename YAMLType = T>
void fromYAMLScalar(const Node& node, const std::string& key, T& dest)
{
  if (node[key] && node[key].IsScalar())
  {
    try
    {
      updateFromYAML<T, YAMLType>(node[key], dest);
    }
    catch (const YAML::Exception& e)
    {
      ROS_ERROR_NAMED("yaml", "Error parsing YAML: %s", e.what());
    }
  }
}

template<typename T, typename YAMLType = T>
void fromYAMLSequence(const Node& node, const std::string& key, T& dest)
{
  if (node[key] && node[key].IsSequence())
  {
    try
    {
      updateFromYAML<T, YAMLType>(node[key], dest);
    }
    catch (const YAML::Exception& e)
    {
      ROS_ERROR_NAMED("yaml", "Error parsing YAML: %s", e.what());
    }
  }
}

template<typename T, typename YAMLType = T>
void fromYAMLMap(const Node& node, const std::string& key, T& dest)
{
  if (node[key] && node[key].IsMap())
  {
    try
    {
      updateFromYAML<T, YAMLType>(node[key], dest);
    }
    catch (const YAML::Exception& e)
    {
      ROS_ERROR_NAMED("yaml", "Error parsing YAML: %s", e.what());
    }
  }
}

template<> struct convert<ros::Time>
{
  static bool decode(const Node& node, ros::Time& rhs)
  {
    if (!node.IsScalar())
      return false;

    try
    {
      rhs.fromSec(node.as<double>());
    }
    catch (const BadConversion&)
    {
      rhs = cras::parseTime(node.as<std::string>());
    }
    return true;
  }
};

template<> struct convert<movie_publisher::StreamTime>
{
  static movie_publisher::RationalNumber fps;

  static bool decode(const Node& node, movie_publisher::StreamTime& rhs)
  {
    if (!node.IsScalar())
      return false;

    try
    {
      rhs.fromSec(node.as<double>());
      return true;
    }
    catch (const YAML::Exception&)
    {
      const auto timeText = node.as<std::string>();
      if (timeText.empty())
        return false;

      movie_publisher::StreamDuration frameSecs;
      const auto timeTextParts = cras::split(timeText, "/", 1);
      if (timeTextParts.size() == 2 && fps.numerator != 0 && fps.denominator != 0)
      {
        try
        {
          const auto frames = cras::parseInt64(cras::strip(timeTextParts[1]), 10);
          frameSecs.fromNSec((frames * fps.denominator * 1000000000LL) / fps.numerator);
        }
        catch (const std::invalid_argument&)
        {
        }
      }

      try
      {
        rhs = movie_publisher::StreamTime(cras::parseDouble(timeTextParts[0])) + frameSecs;
        return true;
      }
      catch (const std::invalid_argument&)
      {
        rhs = movie_publisher::StreamTime(cras::parseTime(timeTextParts[0])) + frameSecs;
        return true;
      }
    }
  }
};

movie_publisher::RationalNumber convert<movie_publisher::StreamTime>::fps = {};

template<typename T, std::size_t N> struct convert<boost::array<T, N>>
{
  static bool decode(const Node& node, boost::array<T, N>& rhs)
  {
    if (!node.IsSequence() || node.size() != boost::array<T, N>::size())
      return false;

    for (size_t i = 0; i < boost::array<T, N>::size(); ++i)
      updateFromYAML<T>(node[i], rhs[i]);

    return true;
  }
};

template<> struct convert<compass_msgs::Azimuth>
{
  static bool decode(const Node& node, compass_msgs::Azimuth& rhs)
  {
    if (!node.IsMap() || !node["azimuth"] || !node["azimuth"].IsScalar())
      return false;

    fromYAMLScalar(node, "azimuth", rhs.azimuth);
    fromYAMLScalar(node, "variance", rhs.variance);
    fromYAMLScalar<uint8_t, uint16_t>(node, "unit", rhs.unit);
    fromYAMLScalar<uint8_t, uint16_t>(node, "orientation", rhs.orientation);
    fromYAMLScalar<uint8_t, uint16_t>(node, "reference", rhs.reference);
    return true;
  }
};

template<> struct convert<geometry_msgs::Vector3>
{
  static bool decode(const Node& node, geometry_msgs::Vector3& rhs)
  {
    if (!node.IsSequence() || node.size() != 3)
      return false;

    const auto data = node.as<std::vector<double>>();
    rhs.x = data[0];
    rhs.y = data[1];
    rhs.z = data[2];
    return true;
  }
};

template<> struct convert<geometry_msgs::Quaternion>
{
  static bool decode(const Node& node, geometry_msgs::Quaternion& rhs)
  {
    if (!node.IsSequence() || (node.size() != 3 && node.size() != 4))
      return false;

    const auto data = node.as<std::vector<double>>();
    if (data.size() == 4)
    {
      rhs.x = data[0];
      rhs.y = data[1];
      rhs.z = data[2];
      rhs.w = data[3];
    }
    else
    {
      tf2::Quaternion q;
      q.setRPY(data[0], data[1], data[2]);
      tf2::convert(q, rhs);
    }
    return true;
  }
};

template<> struct convert<sensor_msgs::MagneticField>
{
  static bool decode(const Node& node, sensor_msgs::MagneticField& rhs)
  {
    if (!node.IsMap() || !node["magnetic_field"] || !node["magnetic_field"].IsSequence())
      return false;

    fromYAMLSequence(node, "magnetic_field", rhs.magnetic_field);
    fromYAMLSequence(node, "magnetic_field_covariance", rhs.magnetic_field_covariance);
    return true;
  }
};

template<> struct convert<geometry_msgs::Pose2D>
{
  static bool decode(const Node& node, geometry_msgs::Pose2D& rhs)
  {
    if (!node.IsSequence() || (node.size() != 2 && node.size() != 3))
      return false;

    const auto data = node.as<std::vector<double>>();
    rhs.x = data[0];
    rhs.y = data[1];
    rhs.theta = data.size() == 3 ? data[2] : 0.0;
    return true;
  }
};

template<> struct convert<vision_msgs::BoundingBox2D>
{
  static bool decode(const Node& node, vision_msgs::BoundingBox2D& rhs)
  {
    if (!node.IsMap() || !node["center"] || !node["size_x"] || !node["size_y"])
      return false;

    rhs.center = node["center"].as<decltype(rhs.center)>();
    rhs.size_x = node["size_x"].as<decltype(rhs.size_x)>();
    rhs.size_y = node["size_y"].as<decltype(rhs.size_y)>();
    return true;
  }
};

template<> struct convert<vision_msgs::Detection2D>
{
  static bool decode(const Node& node, vision_msgs::Detection2D& rhs)
  {
    if (!node.IsMap() || !node["bbox"] || !node["bbox"].IsMap())
      return false;

    rhs.bbox = node["bbox"].as<decltype(rhs.bbox)>();
    if (node["score"] && node["score"].IsScalar())
    {
      vision_msgs::ObjectHypothesisWithPose hypot;
      hypot.score = node["score"].as<decltype(hypot.score)>();
      rhs.results.push_back(hypot);
    }
    return true;
  }
};

template<> struct convert<vision_msgs::Detection2DArray>
{
  static bool decode(const Node& node, vision_msgs::Detection2DArray& rhs)
  {
    if (!node.IsSequence())
      return false;

    for (const auto& face : node)
      rhs.detections.push_back(face.as<vision_msgs::Detection2D>());

    return true;
  }
};

template<> struct convert<geometry_msgs::Transform>
{
  static bool decode(const Node& node, geometry_msgs::Transform& rhs)
  {
    if (!node.IsMap())
      return false;

    if (rhs.rotation.x == 0 && rhs.rotation.y == 0 && rhs.rotation.z == 0 && rhs.rotation.w == 0)
      rhs.rotation.w = 1.0;
    fromYAMLSequence(node, "translation", rhs.translation);
    fromYAMLSequence(node, "rotation", rhs.rotation);

    return true;
  }
};

template<> struct convert<sensor_msgs::RegionOfInterest>
{
  static bool decode(const Node& node, sensor_msgs::RegionOfInterest& rhs)
  {
    if (!node.IsMap())
      return false;

    fromYAMLScalar(node, "x_offset", rhs.x_offset);
    fromYAMLScalar(node, "y_offset", rhs.y_offset);
    fromYAMLScalar(node, "height", rhs.height);
    fromYAMLScalar(node, "width", rhs.width);
    fromYAMLScalar<uint8_t, uint16_t>(node, "do_rectify", rhs.do_rectify);

    return true;
  }
};

template<> struct convert<sensor_msgs::CameraInfo>
{
  static bool decode(const Node& node, sensor_msgs::CameraInfo& rhs)
  {
    if (!node.IsMap())
      return false;

    fromYAMLScalar(node, "width", rhs.width);
    fromYAMLScalar(node, "height", rhs.height);
    fromYAMLScalar(node, "distortion_model", rhs.distortion_model);
    fromYAMLSequence(node, "D", rhs.D);
    fromYAMLSequence(node, "K", rhs.K);
    fromYAMLSequence(node, "R", rhs.R);
    fromYAMLSequence(node, "P", rhs.P);
    fromYAMLScalar(node, "binning_x", rhs.binning_x);
    fromYAMLScalar(node, "binning_y", rhs.binning_y);
    fromYAMLMap(node, "roi", rhs.roi);

    return true;
  }
};

template<> struct convert<sensor_msgs::Imu>
{
  static bool decode(const Node& node, sensor_msgs::Imu& rhs)
  {
    if (!node.IsMap())
      return false;

    if (rhs.orientation.x == 0 && rhs.orientation.y == 0 && rhs.orientation.z == 0 && rhs.orientation.w == 0)
      rhs.orientation.w = 1.0;
    fromYAMLSequence(node, "orientation", rhs.orientation);
    fromYAMLSequence(node, "orientation_covariance", rhs.orientation_covariance);
    fromYAMLSequence(node, "angular_velocity", rhs.angular_velocity);
    fromYAMLSequence(node, "angular_velocity_covariance", rhs.angular_velocity_covariance);
    fromYAMLSequence(node, "linear_acceleration", rhs.linear_acceleration);
    fromYAMLSequence(node, "linear_acceleration_covariance", rhs.linear_acceleration_covariance);

    return true;
  }
};

template<> struct convert<sensor_msgs::NavSatStatus>
{
  static bool decode(const Node& node, sensor_msgs::NavSatStatus& rhs)
  {
    if (!node.IsMap())
      return false;

    fromYAMLScalar<int8_t, int16_t>(node, "status", rhs.status);
    fromYAMLScalar(node, "service", rhs.service);

    return true;
  }
};

template<> struct convert<gps_common::GPSStatus>
{
  static bool decode(const Node& node, gps_common::GPSStatus& rhs)
  {
    if (!node.IsMap())
      return false;

    fromYAMLScalar(node, "status", rhs.status);
    fromYAMLScalar(node, "satellites_used", rhs.satellites_used);
    fromYAMLSequence(node, "satellite_used_prn", rhs.satellite_used_prn);
    fromYAMLScalar(node, "satellites_visible", rhs.satellites_visible);
    fromYAMLSequence(node, "satellite_visible_prn", rhs.satellite_visible_prn);
    fromYAMLSequence(node, "satellite_visible_z", rhs.satellite_visible_z);
    fromYAMLSequence(node, "satellite_visible_azimuth", rhs.satellite_visible_azimuth);
    fromYAMLSequence(node, "satellite_visible_snr", rhs.satellite_visible_snr);
    fromYAMLScalar(node, "position_source", rhs.position_source);
    fromYAMLScalar(node, "orientation_source", rhs.orientation_source);
    fromYAMLScalar(node, "motion_source", rhs.motion_source);

    return true;
  }
};

template<> struct convert<sensor_msgs::NavSatFix>
{
  static bool decode(const Node& node, sensor_msgs::NavSatFix& rhs)
  {
    if (!node.IsMap())
      return false;

    fromYAMLScalar(node, "latitude", rhs.latitude);
    fromYAMLScalar(node, "longitude", rhs.longitude);
    fromYAMLScalar(node, "altitude", rhs.altitude);
    fromYAMLSequence(node, "position_covariance", rhs.position_covariance);
    fromYAMLScalar<uint8_t, uint16_t>(node, "position_covariance_type", rhs.position_covariance_type);
    fromYAMLMap(node, "status", rhs.status);

    return true;
  }
};

template<> struct convert<gps_common::GPSFix>
{
  static bool decode(const Node& node, gps_common::GPSFix& rhs)
  {
    if (!node.IsMap())
      return false;

    fromYAMLScalar(node, "latitude", rhs.latitude);
    fromYAMLScalar(node, "longitude", rhs.longitude);
    fromYAMLScalar(node, "altitude", rhs.altitude);
    fromYAMLScalar(node, "track", rhs.track);
    fromYAMLScalar(node, "speed", rhs.speed);
    fromYAMLScalar(node, "climb", rhs.climb);
    fromYAMLScalar(node, "pitch", rhs.pitch);
    fromYAMLScalar(node, "roll", rhs.roll);
    fromYAMLScalar(node, "dip", rhs.dip);
    fromYAMLScalar(node, "time", rhs.time);
    fromYAMLScalar(node, "gdop", rhs.gdop);
    fromYAMLScalar(node, "pdop", rhs.pdop);
    fromYAMLScalar(node, "hdop", rhs.hdop);
    fromYAMLScalar(node, "vdop", rhs.vdop);
    fromYAMLScalar(node, "tdop", rhs.tdop);
    fromYAMLScalar(node, "err", rhs.err);
    fromYAMLScalar(node, "err_horz", rhs.err_horz);
    fromYAMLScalar(node, "err_vert", rhs.err_vert);
    fromYAMLScalar(node, "err_track", rhs.err_track);
    fromYAMLScalar(node, "err_speed", rhs.err_speed);
    fromYAMLScalar(node, "err_climb", rhs.err_climb);
    fromYAMLScalar(node, "err_time", rhs.err_time);
    fromYAMLScalar(node, "err_pitch", rhs.err_pitch);
    fromYAMLScalar(node, "err_roll", rhs.err_roll);
    fromYAMLScalar(node, "err_dip", rhs.err_dip);
    fromYAMLSequence(node, "position_covariance", rhs.position_covariance);
    fromYAMLScalar<uint8_t, uint16_t>(node, "position_covariance_type", rhs.position_covariance_type);
    fromYAMLMap(node, "status", rhs.status);

    return true;
  }
};

template<> struct convert<movie_publisher::GNSSFixAndDetail>
{
  static bool decode(const Node& node, movie_publisher::GNSSFixAndDetail& rhs)
  {
    if (!node.IsMap())
      return false;

    try
    {
      movie_publisher::GNSSFixAndDetail::first_type::value_type fix;
      if (rhs.first.has_value())
        fix = *rhs.first;
      updateFromYAML(node, fix);
      rhs.first.emplace() = fix;
    }
    catch (const YAML::Exception& e)
    {
      ROS_ERROR_NAMED("yaml", "Error parsing YAML: %s", e.what());
    }

    try
    {
      movie_publisher::GNSSFixAndDetail::second_type::value_type gps;
      if (rhs.second.has_value())
        gps = *rhs.second;
      updateFromYAML(node, gps);
      rhs.second.emplace() = gps;
    }
    catch (const YAML::Exception& e)
    {
      ROS_ERROR_NAMED("yaml", "Error parsing YAML: %s", e.what());
    }

    return true;
  }
};

template<typename T> struct convert<movie_publisher::TimedMetadata<T>>
{
  static bool decode(const Node& node, movie_publisher::TimedMetadata<T>& rhs)
  {
    if (!node.IsMap() || !node["stamp"])
      return false;

    fromYAMLScalar(node, "stamp", rhs.stamp);
    fromYAML(node, "value", rhs.value);

    return true;
  }
};

}
