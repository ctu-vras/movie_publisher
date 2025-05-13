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
#include <movie_publisher/movie_reader.h>

#include "GPMFMetadataExtractor.h"

namespace movie_publisher
{
class TestProcessor : public MovieMetadataProcessor
{
public:
  TimedMetadataCache cache;
  std::vector<sensor_msgs::ImageConstPtr> images;
protected:
  cras::expected<void, std::string> processFrame(const sensor_msgs::ImageConstPtr& image,
    const MoviePlaybackState& playbackState) override
  {
    this->images.push_back(image);
    return {};
  }
  cras::expected<void, std::string> processImage(const sensor_msgs::ImageConstPtr& image,
    const cras::optional<sensor_msgs::CameraInfo>& cameraInfoMsg) override
  {
    this->images.push_back(image);
    if (cameraInfoMsg.has_value())
      this->cache.cameraInfo().emplace_back(TimedMetadata<sensor_msgs::CameraInfo>{{0, 0}, *cameraInfoMsg});
    return {};
  }
  cras::expected<void, std::string> processCameraInfo(const sensor_msgs::CameraInfo& cameraInfoMsg) override
  {
    this->cache.cameraInfo().emplace_back(TimedMetadata<sensor_msgs::CameraInfo>{{0, 0}, cameraInfoMsg});
    return {};
  }
  cras::expected<void, std::string> processAzimuth(const compass_msgs::Azimuth& azimuthMsg) override
  {
    this->cache.azimuth().emplace_back(TimedMetadata<compass_msgs::Azimuth>{{0, 0}, azimuthMsg});
    return {};
  }
  cras::expected<void, std::string> processNavSatFix(const sensor_msgs::NavSatFix& navSatFixMsg) override
  {
    this->cache.gnssPosition().emplace_back(TimedMetadata<GNSSFixAndDetail>{{0, 0}, {navSatFixMsg, {}}});
    return {};
  }
  cras::expected<void, std::string> processGps(const gps_common::GPSFix& gpsMsg) override
  {
    this->cache.gnssPosition().emplace_back(TimedMetadata<GNSSFixAndDetail>{{0, 0}, {{}, gpsMsg}});
    return {};
  }
  cras::expected<void, std::string> processImu(const sensor_msgs::Imu& imuMsg) override
  {
    this->cache.imu().emplace_back(TimedMetadata<sensor_msgs::Imu>{{0, 0}, imuMsg});
    return {};
  }
  cras::expected<void, std::string>
  processZeroRollPitchTf(const geometry_msgs::TransformStamped& zeroRollPitchTfMsg) override
  {
    this->cache.zeroRollPitchTF().emplace_back(
      TimedMetadata<geometry_msgs::Transform>{{0, 0}, zeroRollPitchTfMsg.transform});
    return {};
  }
  cras::expected<void, std::string> processOpticalTf(const geometry_msgs::TransformStamped& opticalTfMsg) override
  {
    this->cache.opticalFrameTF().emplace_back(TimedMetadata<geometry_msgs::Transform>{{0, 0}, opticalTfMsg.transform});
    return {};
  }
  cras::expected<void, std::string> processMagneticField(const sensor_msgs::MagneticField& magneticFieldMsg) override
  {
    this->cache.magneticField().emplace_back(TimedMetadata<sensor_msgs::MagneticField>{{0, 0}, magneticFieldMsg});
    return {};
  }
  cras::expected<void, std::string> processFaces(const vision_msgs::Detection2DArray& facesMsg) override
  {
    this->cache.faces().emplace_back(TimedMetadata<vision_msgs::Detection2DArray>{{0, 0}, facesMsg});
    return {};
  }
};
}

std::pair<movie_publisher::MetadataExtractor::Ptr, movie_publisher::MoviePtr> getExtractor(
  const std::string& filename, const size_t width, const size_t height, const bool isStillImage,
  const size_t videoStreamIndex)
{
  // auto log = std::make_shared<cras::MemoryLogHelper>();
  auto log = std::make_shared<cras::NodeLogHelper>();

  XmlRpc::XmlRpcValue paramsXml;
  paramsXml["allowed_extractors"][0] = "movie_publisher/camera_info_composer";
  paramsXml["allowed_extractors"][1] = "movie_publisher/camera_intrinsics_composer";
  paramsXml["allowed_extractors"][2] = "movie_publisher/camera_name_composer";
  paramsXml["allowed_extractors"][3] = "movie_publisher/imu_composer";
  paramsXml["allowed_extractors"][4] = "movie_publisher/roll_pitch_composer";
  paramsXml["allowed_extractors"][5] = "movie_publisher/optical_frame_tf_composer";
  paramsXml["allowed_extractors"][6] = "movie_publisher/zero_roll_pitch_tf_composer";
  paramsXml["allowed_extractors"][7] = "movie_publisher/gpmf_metadata_extractor";
  auto adapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(paramsXml, "");
  auto params = std::make_shared<cras::BoundParamHelper>(log, adapter);
  movie_publisher::MovieOpenConfig config(params);
  auto processor = std::make_shared<movie_publisher::TestProcessor>();
  config.metadataProcessors().push_back(processor);

  movie_publisher::MovieReader reader(log, params);
  const auto maybeMovie = reader.open(filename, config);
  if (!maybeMovie.has_value())
    throw std::runtime_error("Failed to open file " + filename);

  auto extractor = maybeMovie.value()->staticMetadata();
  return {extractor, *maybeMovie};
}

TEST(GPMFMetadataExtractor, FairphoneStill)  // NOLINT
{
  auto [m, f] = getExtractor(std::string(TEST_DATA_DIR) + "/fairphone/IMG_20241125_024757.jpg", 4000, 3000, true, 0);

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
  auto [m, f] = getExtractor(std::string(TEST_DATA_DIR) + "/fairphone/VID_20240815_143536.mp4", 1920, 1080, false, 0);

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
  auto [m, f] = getExtractor(std::string(TEST_DATA_DIR) + "/lumix/P1260334.JPG", 4592, 3448, true, 0);

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
  auto [m, f] = getExtractor(std::string(TEST_DATA_DIR) + "/lumix/P1260657.MP4", 1920, 1080, false, 0);

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
  auto [m, f] = getExtractor(std::string(TEST_DATA_DIR) + "/ffmpeg_processed/P1320029.MP4.mp4", 1920, 1080, false, 0);

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
  auto [m, f] = getExtractor(std::string(TEST_DATA_DIR) + "/iphone/20241005_160034_IMG_4998.jpg", 4032, 3024, true, 0);

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
  auto [m, f] = getExtractor(std::string(TEST_DATA_DIR) + "/iphone/IMG_2585.MOV", 1920, 1080, false, 0);

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

TEST(GPMFMetadataExtractor, GoproMovie)  // NOLINT
{
  auto [m, f] = getExtractor(std::string(TEST_DATA_DIR) + "/gopro/GX010017.MP4", 1920, 1080, false, 0);

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

  for (size_t i = 0; i < 320; ++i)
    ASSERT_TRUE(f->nextFrame().has_value());


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
  // ASSERT_TRUE(m->getRollPitch());
  EXPECT_FALSE(m->getAcceleration());
  EXPECT_FALSE(m->getAzimuth());
  auto [nav2, gps2] = m->getGNSSPosition();
  EXPECT_FALSE(nav2); EXPECT_FALSE(gps2);

  auto proc = std::dynamic_pointer_cast<movie_publisher::TestProcessor>(f->config().metadataProcessors()[0]);
  EXPECT_EQ(proc->cache.faces().size(), 2);
  EXPECT_GT(proc->cache.imu().size(), 2000);
  EXPECT_GT(proc->cache.zeroRollPitchTF().size(), 400);
  EXPECT_GT(proc->cache.gnssPosition().size(), 90);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::console::initialize();
  ros::console::set_logger_level("ros.gpmf_metadata_extractor", ros::console::Level::Debug);
  ros::console::set_logger_level("ros.movie_publisher", ros::console::Level::Debug);
  ros::console::set_logger_level("ros.movie_publisher.pluginlib", ros::console::Level::Info);
  ros::console::notifyLoggerLevelsChanged();
  ros::Time::init();
  return RUN_ALL_TESTS();
}
