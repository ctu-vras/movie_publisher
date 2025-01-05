// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Parameter parsing helpers.
 * \author Martin Pecka
 */


#include <list>
#include <memory>
#include <stdexcept>
#include <string>
#include <regex>
#include <unordered_map>
#include <vector>

#include <muParser.h>

#include <cras_cpp_common/string_utils.hpp>
#include <movie_publisher/movie_reader.h>
#include <movie_publisher/parsing_utils.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace movie_publisher
{

std::regex timeRegex("(\\d+):(\\d+):(\\d+([,|.]\\d+)?)");

template<typename T>
T parseTimeParam(const XmlRpc::XmlRpcValue& param)
{
  if (param.getType() == XmlRpc::XmlRpcValue::TypeString)
  {
    auto time = static_cast<std::string>(param);
    if (!cras::contains(time, ',') && !cras::contains(time, '.'))
      time = time + ".0";
    std::smatch matches;
    if (!std::regex_match(time, matches, timeRegex))
      throw std::runtime_error(cras::format("Could not parse value '%s' as a time string.", time.c_str()));

    const int64_t hr = cras::parseInt64(matches[1].str().c_str(), 10);
    const int32_t mn = cras::parseInt32(matches[2].str().c_str(), 10);
    const auto secondsStr = cras::replace(matches[3].str(), ",", ".");
    const double sec = cras::parseDouble(secondsStr);
    return T().fromSec(3600.0 * hr + 60.0 * mn + sec);
  }
  else if (param.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    if (param.size() != 2 && param.size() != 3)
      throw std::runtime_error("Time parameter has to have 2 or 3 values when passed as tuple.");
    std::vector<double> values;
    std::list<std::string> errors;
    if (!cras::convert(param, values, false, &errors))
      throw std::runtime_error(cras::format("Wrong values for time parameter: %s", cras::to_string(errors).c_str()));
    double hr = 0, mn = 0, sec = 0;
    if (values.size() == 3)
    {
      hr = values[0];
      mn = values[1];
      sec = values[2];
    }
    else
    {
      mn = values[0];
      sec = values[1];
    }
    return T().fromSec(3600.0 * hr + 60.0 * mn + sec);
  }
  else
  {
    double value;
    std::list<std::string> errors;
    if (!cras::convert(param, value, false, &errors))
      throw std::runtime_error(cras::format("Wrong value for time parameter: %s", cras::to_string(errors).c_str()));
    return T().fromSec(value);
  }
}

template ros::Duration parseTimeParam(const XmlRpc::XmlRpcValue& param);
template ros::Time parseTimeParam(const XmlRpc::XmlRpcValue& param);

bool parseTimestampOffset(const std::unordered_map<std::string, double>& extraVars,
  const XmlRpc::XmlRpcValue& param, double& value, const bool skipNonConvertible, std::list<std::string>* errors)
{
  if (param.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    value = static_cast<int>(param);
    return true;
  }
  else if (param.getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    value = param;
    return true;
  }
  else if (param.getType() == XmlRpc::XmlRpcValue::TypeString)
  {
    mu::Parser parser;

    auto wallTime = ros::WallTime::now().toSec();
    parser.DefineVar("wall_time", &wallTime);

    double rosTime {0};
    try
    {
      rosTime = ros::Time::now().toSec();
    }
    catch (const ros::TimeNotInitializedException&) {}
    parser.DefineVar("ros_time", &rosTime);

    std::vector<double> vars;
    for (auto& [var, val] : extraVars)
    {
      vars.push_back(val);
      parser.DefineVar(var, &vars.back());
    }

    try
    {
      parser.SetExpr(param);
      value = parser.Eval();
      return true;
    }
    catch (const mu::Parser::exception_type& e)
    {
      if (errors)
      {
        errors->push_back(cras::format(
          "Error parsing expression '%s': %s", e.GetExpr().c_str(), e.GetMsg().c_str()));
      }
      return false;
    }
  }
  else
  {
    if (errors)
      errors->push_back(cras::format("Wrong type: %i", param.getType()));
    return false;
  }
}

MovieReader::TimestampSource parseTimestampSource(const std::string& param)
{
  const auto& p = cras::toLower(param);
  if (p == "all_zeros")
    return MovieReader::TimestampSource::AllZeros;
  else if (p == "absolute_timecode")
    return MovieReader::TimestampSource::AbsoluteVideoTimecode;
  else if (p == "relative_timecode")
    return MovieReader::TimestampSource::RelativeVideoTimecode;
  else if (p == "ros_time")
    return MovieReader::TimestampSource::RosTime;
  else if (p == "metadata")
    return MovieReader::TimestampSource::FromMetadata;
  throw std::runtime_error(cras::format("Value %s is not a valid timestamp_source value.", param.c_str()));
}

std::string timestampSourceToStr(const MovieReader::TimestampSource& source)
{
  switch (source)
  {
    case MovieReader::TimestampSource::AllZeros:
      return "all_zeros";
    case MovieReader::TimestampSource::AbsoluteVideoTimecode:
      return "absolute_timecode";
    case MovieReader::TimestampSource::RelativeVideoTimecode:
      return "relative_timecode";
    case MovieReader::TimestampSource::RosTime:
      return "ros_time";
    case MovieReader::TimestampSource::FromMetadata:
      return "metadata";
    default:
      throw std::runtime_error("Wrong value");
  }
}

}
