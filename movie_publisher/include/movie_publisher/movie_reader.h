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

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <movie_publisher/movie.h>
#include <movie_publisher/movie_open_config.h>

namespace movie_publisher
{

struct MovieReaderPrivate;

/**
 * \brief Helper class for opening movie files.
 *
 * MovieReader holds configuration that will be applied to all movies open using its open() method. Changing the
 * configuration does not affect any already opened movies.
 *
 * Typical usage:
 *
 * <pre>
 * MovieReader reader({new NodeLogHelper()});
 * // configuration for opening movies
 * auto config = reader.createDefaultConfig().or_else([](const auto& error) {throw std::runtime_error(error);});
 * config.setAllowYUVFallback(true);
 * auto movie = reader.open("path/file.mp4", config).or_else([](const auto& error) {throw std::runtime_error(error);});
 * // You can call seek() here
 * while (true)
 * {
 *   const auto& [state, image] = *movie->nextFrame().or_else([](const auto& error) {throw std::runtime_error(error);});
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
   * \param [in] params ROS/YAML parameters of movie opening and metadata extractors.
   */
  explicit MovieReader(const cras::LogHelperPtr& log, const cras::BoundParamHelperPtr& params);
  virtual ~MovieReader();

  /**
   * \brief Open a movie in the referenced file.
   * \param[in] filename Path to the file with the movie.
   * \param[in] config Extra configuration for the movie opening.
   * \return The movie object or an error message on failure.
   */
  virtual cras::expected<MoviePtr, std::string> open(const std::string& filename, const MovieOpenConfig& config);

  /**
   * \brief Create a default config for opening movie files.
   * \return The config object or error string.
   */
  virtual cras::expected<MovieOpenConfig, std::string> createDefaultConfig() const;

private:
  std::unique_ptr<MovieReaderPrivate> data;  //!< PIMPL
};

}
