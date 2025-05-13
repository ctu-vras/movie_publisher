// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Read a movie or image file.
 * \author Martin Pecka
 */

#include <memory>
#include <stdexcept>
#include <string>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <movie_publisher/movie_reader.h>
#include <movie_publisher/parsing_utils.h>

extern "C" {
#include <libavformat/avformat.h>
}

namespace movie_publisher
{

struct MovieReaderPrivate
{
  cras::BoundParamHelperPtr params;
};

MovieReader::MovieReader(const cras::LogHelperPtr& log, const cras::BoundParamHelperPtr& params) :
  HasLogger(log), data(new MovieReaderPrivate())
{
  this->data->params = params;
}

MovieReader::~MovieReader() = default;

cras::expected<MovieOpenConfig, std::string> MovieReader::createDefaultConfig() const
{
  MovieOpenConfig config(this->data->params);
  if (auto result = config.setTimestampSource(TimestampSource::RosTime); !result.has_value())
    return cras::make_unexpected(result.error());
  return config;
}

cras::expected<MoviePtr, std::string> MovieReader::open(const std::string& filename, const MovieOpenConfig& config)
{
  auto configCopy = config;
  if (auto result = configCopy.setFilenameOrURL(filename); !result.has_value())
    return cras::make_unexpected(result.error());

  try
  {
    return std::make_shared<Movie>(this->log, configCopy);
  }
  catch (const std::runtime_error& e)
  {
    return cras::make_unexpected(cras::format("Error opening movie: %s", e.what()));
  }
}

}
