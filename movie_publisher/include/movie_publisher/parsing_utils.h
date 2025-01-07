// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Parameter parsing helpers.
 * \author Martin Pecka
 */

#pragma once

#include <list>
#include <memory>
#include <string>
#include <unordered_map>

#include <movie_publisher/movie_reader.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace movie_publisher
{
/**
 * \brief Parse time or duration in several formats.
 * \tparam T ros::Time or ros::Duration
 * \param[in] param The parameter to parse. Can be float seconds `(15.35)`, tuple `(min, sec)`, tuple `(hour, min, sec)`
 *                  or as a string:`'01:03:05.35'`.
 * \return The parsed time value.
 */
template<typename T>
T parseTimeParam(const XmlRpc::XmlRpcValue& param);

/**
 * \brief Parse the given timestamp offset.
 * \param[in] extraVars Extra variables that should be resolved by the expression parser.
 * \param[in] param The ROS parameter. Can be either int, double or string. If string, it can contain a mathematical
 *                  expression with several predefined values: `ros_time` (current ROS time), `wall_time` (current wall
 *                  time) and other variables defined in extraVars.
 * \param[out] value The output value.
 * \param[in] skipNonConvertible No effect.
 * \param[in] errors List of encountered errors. Can be nullptr.
 * \return Whether the conversion to `value` succeeded.
 */
bool parseTimestampOffset(const std::unordered_map<std::string, double>& extraVars,
  const XmlRpc::XmlRpcValue& param, double& value, bool skipNonConvertible, std::list<std::string>* errors);

/**
 * \brief Parse TimestampSource from text.
 * \param[in] param The text to parse.
 * \return The timestamp source.
 * \throws std::runtime_error On error.
 */
MovieReader::TimestampSource parseTimestampSource(const std::string& param);

/**
 * \brief Convert the given TimestampSource to text.
 * \param[in] source The timestamp source.
 * \return The text representing the timestamp source.
 */
std::string timestampSourceToStr(const MovieReader::TimestampSource& source);

}
