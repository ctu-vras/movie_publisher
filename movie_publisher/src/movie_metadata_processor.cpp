// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Base for consumers of movie metadata.
 * \author Martin Pecka
 */

#include <memory>
#include <string>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/param_utils.hpp>
#include <movie_publisher/movie_info.h>
#include <movie_publisher/movie_metadata_processor.h>
#include <movie_publisher/movie_playback_state.h>
#include <sensor_msgs/Image.h>

namespace movie_publisher
{

MovieMetadataProcessor::~MovieMetadataProcessor() = default;

cras::expected<void, std::string> MovieMetadataProcessor::onOpen(
  const MovieInfo::ConstPtr& info, const MovieOpenConfig& config)
{
  this->info = info;
  this->config = config;
  this->verbose = config.rosParams()->getParam("verbose", false);
  return {};
}

cras::expected<void, std::string> MovieMetadataProcessor::onMetadataReady(
  const std::shared_ptr<TimedMetadataExtractor>& metadataExtractor)
{
  this->metadataExtractor = metadataExtractor;
  return {};
}

cras::expected<void, std::string> MovieMetadataProcessor::onClose()
{
  this->info.reset();
  this->metadataExtractor.reset();
  return {};
}

cras::expected<void, std::string> MovieMetadataProcessor::processFrame(
  const sensor_msgs::ImageConstPtr& image, const MoviePlaybackState& playbackState)
{
  if (this->verbose && this->info != nullptr)
  {
    const auto numFrames = this->info->subclipNumFrames();

    CRAS_INFO("Frame %zu/%zu, stream time %s, subclip time %s, ROS stamp %s",
      playbackState.subclipFrameNum(), numFrames, cras::to_string(playbackState.streamTime()).c_str(),
      cras::to_string(playbackState.subclipTime()).c_str(), cras::to_string(playbackState.rosTime()).c_str());
  }

  auto cameraInfoMsg = this->metadataExtractor->getCameraInfo();
  if (cameraInfoMsg.has_value())
    cameraInfoMsg->header = image->header;
  const auto processImageResult = this->processImage(image, cameraInfoMsg);
  if (!processImageResult.has_value())
    return cras::make_unexpected(processImageResult.error());

  return {};
}

}
