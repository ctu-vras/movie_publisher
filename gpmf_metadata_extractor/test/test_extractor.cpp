// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for GPMFMetadataExtractor.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <memory>
#include <string>

extern "C"
{
#include <libavformat/avformat.h>
}

#include <cras_cpp_common/log_utils/memory.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/param_utils/param_helper.hpp>
#include <cras_cpp_common/param_utils/get_param_adapters/xmlrpc_value.hpp>
#include <cras_cpp_common/string_utils/ros.hpp>
#include <movie_publisher/metadata_manager.h>

#include "GPMFMetadataExtractor.h"

std::pair<movie_publisher::MetadataManager::Ptr, AVFormatContext*> getExtractor(
  const std::string& filename, const size_t width, const size_t height, const bool isStillImage,
  const size_t videoStreamIndex)
{
  // auto log = std::make_shared<cras::MemoryLogHelper>();
  auto log = std::make_shared<cras::NodeLogHelper>();

  auto* formatContext = avformat_alloc_context();

  if (avformat_open_input(&formatContext, filename.c_str(), nullptr, nullptr) != 0)
    return {nullptr, nullptr};

  int res = avformat_find_stream_info(formatContext, nullptr);
  if (res < 0)
  {
    avformat_close_input(&formatContext);
    return {nullptr, nullptr};
  }

  if (videoStreamIndex >= formatContext->nb_streams)
  {
    avformat_close_input(&formatContext);
    return {nullptr, nullptr};
  }

  const auto& stream = formatContext->streams[videoStreamIndex];
  if (stream->codecpar->codec_type != AVMEDIA_TYPE_VIDEO)
  {
    avformat_close_input(&formatContext);
    return {nullptr, nullptr};
  }

  XmlRpc::XmlRpcValue paramsXml;
  paramsXml.begin();
  auto adapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(paramsXml, "");
  auto params = std::make_shared<cras::BoundParamHelper>(log, adapter);
  movie_publisher::MovieOpenConfig config(params);
  auto info = std::make_shared<movie_publisher::MovieInfo>();
  info->setWidth(width);
  info->setHeight(height);

  auto manager = std::make_shared<movie_publisher::MetadataManager>(log, config, info);
  auto extractor = std::make_shared<movie_publisher::GPMFMetadataExtractor>(
    log, manager, width, height, formatContext, 10);
  manager->addExtractor(extractor);

  // Add camera intrinsics composer
  pluginlib::ClassLoader<movie_publisher::MetadataExtractorPlugin> loader(
    "movie_publisher", "movie_publisher::MetadataExtractorPlugin", "metadata_plugins");
  const auto cl = loader.createUniqueInstance("movie_publisher/camera_intrinsics_composer");
  movie_publisher::MetadataExtractorParams extractorParams {log, manager, config, info, nullptr, manager->getCache()};
  manager->addExtractor(cl->getExtractor(extractorParams));

  return {manager, formatContext};
}

TEST(GPMFMetadataExtractor, FairphoneStill)  // NOLINT
{
  auto [m, c] = getExtractor(std::string(TEST_DATA_DIR) + "/fairphone/IMG_20241125_024757.jpg", 4000, 3000, true, 0);

  EXPECT_FALSE(m->getRotation());
  EXPECT_FALSE(m->getCreationTime());
  EXPECT_FALSE(m->getCameraSerialNumber());
  EXPECT_FALSE(m->getCameraMake());
  EXPECT_FALSE(m->getCameraModel());
  EXPECT_FALSE(m->getLensMake());
  EXPECT_FALSE(m->getLensModel());
  EXPECT_FALSE(m->getCropFactor());
  EXPECT_FALSE(m->getSensorSizeMM());
  EXPECT_FALSE(m->getFocalLength35MM());
  EXPECT_FALSE(m->getFocalLengthMM());
  EXPECT_FALSE(m->getFocalLengthPx());
  EXPECT_FALSE(m->getRollPitch());
  EXPECT_FALSE(m->getAcceleration());
  EXPECT_FALSE(m->getAzimuth());
  auto [nav, gps] = m->getGNSSPosition();
  EXPECT_FALSE(nav);
  EXPECT_FALSE(gps);
}

TEST(GPMFMetadataExtractor, FairphoneMovie)  // NOLINT
{
  auto [m, c] = getExtractor(std::string(TEST_DATA_DIR) + "/fairphone/VID_20240815_143536.mp4", 1920, 1080, false, 0);

  EXPECT_FALSE(m->getRotation());
  EXPECT_FALSE(m->getCreationTime());
  EXPECT_FALSE(m->getCameraSerialNumber());
  EXPECT_FALSE(m->getCameraMake());
  EXPECT_FALSE(m->getCameraModel());
  EXPECT_FALSE(m->getLensMake());
  EXPECT_FALSE(m->getLensModel());
  EXPECT_FALSE(m->getCropFactor());
  EXPECT_FALSE(m->getSensorSizeMM());
  EXPECT_FALSE(m->getFocalLength35MM());
  EXPECT_FALSE(m->getFocalLengthMM());
  EXPECT_FALSE(m->getFocalLengthPx());
  EXPECT_FALSE(m->getRollPitch());
  EXPECT_FALSE(m->getAcceleration());
  EXPECT_FALSE(m->getAzimuth());
  auto [nav, gps] = m->getGNSSPosition();
  EXPECT_FALSE(nav); EXPECT_FALSE(gps);
}

TEST(GPMFMetadataExtractor, LumixStill)  // NOLINT
{
  auto [m, c] = getExtractor(std::string(TEST_DATA_DIR) + "/lumix/P1260334.JPG", 4592, 3448, true, 0);

  EXPECT_FALSE(m->getRotation());
  EXPECT_FALSE(m->getCreationTime());
  EXPECT_FALSE(m->getCameraSerialNumber());
  EXPECT_FALSE(m->getCameraMake());
  EXPECT_FALSE(m->getCameraModel());
  EXPECT_FALSE(m->getLensMake());
  EXPECT_FALSE(m->getLensModel());
  EXPECT_FALSE(m->getCropFactor());
  EXPECT_FALSE(m->getSensorSizeMM());
  EXPECT_FALSE(m->getFocalLength35MM());
  EXPECT_FALSE(m->getFocalLengthMM());
  EXPECT_FALSE(m->getFocalLengthPx());
  EXPECT_FALSE(m->getRollPitch());
  EXPECT_FALSE(m->getAcceleration());
  EXPECT_FALSE(m->getAzimuth());
  auto [nav, gps] = m->getGNSSPosition();
  EXPECT_FALSE(nav);
  EXPECT_FALSE(gps);
}

TEST(GPMFMetadataExtractor, LumixMovie)  // NOLINT
{
  auto [m, c] = getExtractor(std::string(TEST_DATA_DIR) + "/lumix/P1260657.MP4", 1920, 1080, false, 0);

  EXPECT_FALSE(m->getRotation());
  EXPECT_FALSE(m->getCreationTime());
  EXPECT_FALSE(m->getCameraSerialNumber());
  EXPECT_FALSE(m->getCameraMake());
  EXPECT_FALSE(m->getCameraModel());
  EXPECT_FALSE(m->getLensMake());
  EXPECT_FALSE(m->getLensModel());
  EXPECT_FALSE(m->getCropFactor());
  EXPECT_FALSE(m->getSensorSizeMM());
  EXPECT_FALSE(m->getFocalLength35MM());
  EXPECT_FALSE(m->getFocalLengthMM());
  EXPECT_FALSE(m->getFocalLengthPx());
  EXPECT_FALSE(m->getRollPitch());
  EXPECT_FALSE(m->getAcceleration());
  EXPECT_FALSE(m->getAzimuth());
  auto [nav, gps] = m->getGNSSPosition();
  EXPECT_FALSE(nav); EXPECT_FALSE(gps);
}

TEST(GPMFMetadataExtractor, FfmpegProcessed)  // NOLINT
{
  auto [m, c] = getExtractor(std::string(TEST_DATA_DIR) + "/ffmpeg_processed/P1320029.MP4.mp4", 1920, 1080, false, 0);

  EXPECT_FALSE(m->getRotation());
  EXPECT_FALSE(m->getCreationTime());
  EXPECT_FALSE(m->getCameraSerialNumber());
  EXPECT_FALSE(m->getCameraMake());
  EXPECT_FALSE(m->getCameraModel());
  EXPECT_FALSE(m->getLensMake());
  EXPECT_FALSE(m->getLensModel());
  EXPECT_FALSE(m->getCropFactor());
  EXPECT_FALSE(m->getSensorSizeMM());
  EXPECT_FALSE(m->getFocalLength35MM());
  EXPECT_FALSE(m->getFocalLengthMM());
  EXPECT_FALSE(m->getFocalLengthPx());
  EXPECT_FALSE(m->getRollPitch());
  EXPECT_FALSE(m->getAcceleration());
  EXPECT_FALSE(m->getAzimuth());
  auto [nav, gps] = m->getGNSSPosition();
  EXPECT_FALSE(nav); EXPECT_FALSE(gps);
}

TEST(GPMFMetadataExtractor, IphoneStill)  // NOLINT
{
  auto [m, c] = getExtractor(std::string(TEST_DATA_DIR) + "/iphone/20241005_160034_IMG_4998.jpg", 4032, 3024, true, 0);

  EXPECT_FALSE(m->getRotation());
  EXPECT_FALSE(m->getCreationTime());
  EXPECT_FALSE(m->getCameraSerialNumber());
  EXPECT_FALSE(m->getCameraMake());
  EXPECT_FALSE(m->getCameraModel());
  EXPECT_FALSE(m->getLensMake());
  EXPECT_FALSE(m->getLensModel());
  EXPECT_FALSE(m->getCropFactor());
  EXPECT_FALSE(m->getSensorSizeMM());
  EXPECT_FALSE(m->getFocalLength35MM());
  EXPECT_FALSE(m->getFocalLengthMM());
  EXPECT_FALSE(m->getFocalLengthPx());
  EXPECT_FALSE(m->getRollPitch());
  EXPECT_FALSE(m->getAcceleration());
  EXPECT_FALSE(m->getAzimuth());
  auto [nav, gps] = m->getGNSSPosition();
  EXPECT_FALSE(nav);
  EXPECT_FALSE(gps);
}

TEST(GPMFMetadataExtractor, IphoneMovie)  // NOLINT
{
  auto [m, c] = getExtractor(std::string(TEST_DATA_DIR) + "/iphone/IMG_2585.MOV", 1920, 1080, false, 0);

  EXPECT_FALSE(m->getRotation());
  EXPECT_FALSE(m->getCreationTime());
  EXPECT_FALSE(m->getCameraSerialNumber());
  EXPECT_FALSE(m->getCameraMake());
  EXPECT_FALSE(m->getCameraModel());
  EXPECT_FALSE(m->getLensMake());
  EXPECT_FALSE(m->getLensModel());
  EXPECT_FALSE(m->getCropFactor());
  EXPECT_FALSE(m->getSensorSizeMM());
  EXPECT_FALSE(m->getFocalLength35MM());
  EXPECT_FALSE(m->getFocalLengthMM());
  EXPECT_FALSE(m->getFocalLengthPx());
  EXPECT_FALSE(m->getRollPitch());
  EXPECT_FALSE(m->getAcceleration());
  EXPECT_FALSE(m->getAzimuth());
  auto [nav, gps] = m->getGNSSPosition();
  EXPECT_FALSE(nav); EXPECT_FALSE(gps);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::console::initialize();
  ros::console::set_logger_level("ros.gpmf_metadata_extractor", ros::console::Level::Debug);
  ros::console::set_logger_level("ros.movie_publisher", ros::console::Level::Debug);
  ros::console::set_logger_level("ros.movie_publisher.pluginlib", ros::console::Level::Info);
  ros::console::notifyLoggerLevelsChanged();
  return RUN_ALL_TESTS();
}
