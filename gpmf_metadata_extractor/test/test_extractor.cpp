// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for GPMFMetadataExtractor.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"
#include "gmock/gmock.h"

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
#include <movie_publisher/movie_reader_ros.h>

#include "GPMFMetadataExtractor.h"

template<typename T>
auto matchesUpToStamp(const T& m)
{
  auto mCopy = m;
  mCopy.header.stamp = {};
  return testing::Eq(mCopy);
}

template<>
auto matchesUpToStamp(const gps_common::GPSFix& m)
{
  auto mCopy = m;
  mCopy.header.stamp = {};
  mCopy.status.header.stamp = {};
  return testing::Eq(mCopy);
}

template<>
auto matchesUpToStamp(const vision_msgs::Detection2DArray& m)
{
  auto mCopy = m;
  mCopy.header.stamp = {};
  for (auto& det : mCopy.detections)
    det.header.stamp = {};
  return testing::Eq(mCopy);
}

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
    if (this->cache.gnssPosition().empty() || !this->cache.gnssPosition().back().value.second.has_value() ||
      this->cache.gnssPosition().back().value.second->header.stamp != navSatFixMsg.header.stamp)
    {
      this->cache.gnssPosition().emplace_back(TimedMetadata<GNSSFixAndDetail>{{0, 0}, {{}, {}}});
    }
    this->cache.gnssPosition().back().value.first = navSatFixMsg;
    return {};
  }
  cras::expected<void, std::string> processGps(const gps_common::GPSFix& gpsMsg) override
  {
    if (this->cache.gnssPosition().empty() || !this->cache.gnssPosition().back().value.first.has_value() ||
      this->cache.gnssPosition().back().value.first->header.stamp != gpsMsg.header.stamp)
    {
      this->cache.gnssPosition().emplace_back(TimedMetadata<GNSSFixAndDetail>{{0, 0}, {{}, {}}});
    }
    this->cache.gnssPosition().back().value.second = gpsMsg;
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

std::tuple<
  movie_publisher::MetadataExtractor::Ptr, movie_publisher::MoviePtr, std::shared_ptr<movie_publisher::TestProcessor>>
getExtractor(const std::string& filename, const size_t width, const size_t height, const bool isStillImage,
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
  config.setFrameId("test");
  config.setOpticalFrameId("test_optical_frame");
  config.setTimestampSource(movie_publisher::TimestampSource::AbsoluteVideoTimecode);

  movie_publisher::MovieReader reader(log, params);
  const auto maybeMovie = reader.open(filename, config);
  if (!maybeMovie.has_value())
    throw std::runtime_error("Failed to open file " + filename);

  auto extractor = maybeMovie.value()->staticMetadata();
  return {extractor, *maybeMovie, processor};
}

TEST(GPMFMetadataExtractor, FairphoneStill)  // NOLINT
{
  auto [m, f, p] = getExtractor(std::string(TEST_DATA_DIR) + "/fairphone/IMG_20241125_024757.jpg", 4000, 3000, true, 0);

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
  auto [m, f, p] = getExtractor(
    std::string(TEST_DATA_DIR) + "/fairphone/VID_20240815_143536.mp4", 1920, 1080, false, 0);

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
  auto [m, f, p] = getExtractor(std::string(TEST_DATA_DIR) + "/lumix/P1260334.JPG", 4592, 3448, true, 0);

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
  auto [m, f, p] = getExtractor(std::string(TEST_DATA_DIR) + "/lumix/P1260657.MP4", 1920, 1080, false, 0);

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
  auto [m, f, p] = getExtractor(
    std::string(TEST_DATA_DIR) + "/ffmpeg_processed/P1320029.MP4.mp4", 1920, 1080, false, 0);

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
  auto [m, f, p] = getExtractor(
    std::string(TEST_DATA_DIR) + "/iphone/20241005_160034_IMG_4998.jpg", 4032, 3024, true, 0);

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
  auto [m, f, p] = getExtractor(std::string(TEST_DATA_DIR) + "/iphone/IMG_2585.MOV", 1920, 1080, false, 0);

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
  auto [m, f, p] = getExtractor(std::string(TEST_DATA_DIR) + "/gopro/GX010017.MP4", 1920, 1080, false, 0);
  // auto [m, f, p] = getExtractor("/media/data/bags/gopro/GX010074.MP4", 1920, 1080, false, 0);

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
  EXPECT_FALSE(m->getMagneticField());
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
  EXPECT_FALSE(m->getAzimuth());
  EXPECT_FALSE(m->getMagneticField());

  ASSERT_TRUE(m->getRollPitch());
  movie_publisher::RollPitch rollPitch = {0.0064857581138091508, -0.18201449062699399};
  EXPECT_EQ(rollPitch, *m->getRollPitch());

  ASSERT_TRUE(m->getAcceleration());
  geometry_msgs::Vector3 acceleration;
  acceleration.x = 0.96402877697841727;
  acceleration.y = 2.1822541966426861;
  acceleration.z = 10.810551558752998;
  EXPECT_EQ(acceleration, *m->getAcceleration());

  ASSERT_TRUE(m->getAngularVelocity());
  geometry_msgs::Vector3 angularVelocity;
  angularVelocity.x = 0.35569755058572949;
  angularVelocity.y = -0.043663471778487756;
  angularVelocity.z = 1.0372736954206603;
  EXPECT_EQ(angularVelocity, *m->getAngularVelocity());

  sensor_msgs::NavSatFix refNav2;
  refNav2.header.frame_id = "test";
  refNav2.header.stamp = {0, 253716000};
  refNav2.latitude = 50.075624599999998;
  refNav2.longitude = 14.4173721;
  refNav2.altitude = 149.727;
  refNav2.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
  refNav2.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

  gps_common::GPSFix refGps2;
  refGps2.header.frame_id = "test";
  refGps2.header.stamp = {0, 253716000};
  refGps2.status.header = refGps2.header;
  refGps2.latitude = 50.075624599999998;
  refGps2.longitude = 14.4173721;
  refGps2.altitude = 149.727;
  refGps2.speed = 0.30099999999999999;
  refGps2.gdop = 99.989999999999995;
  refGps2.status.status = gps_common::GPSStatus::STATUS_FIX;
  refGps2.status.position_source = gps_common::GPSStatus::SOURCE_GPS;

  auto [nav2, gps2] = m->getGNSSPosition();
  ASSERT_TRUE(nav2); ASSERT_TRUE(gps2);
  EXPECT_THAT(*nav2, matchesUpToStamp(refNav2));
  EXPECT_THAT(*gps2, matchesUpToStamp(refGps2));

  ASSERT_FALSE(p->cache.gnssPosition().empty());
  nav2 = p->cache.gnssPosition().back().value.first;
  gps2 = p->cache.gnssPosition().back().value.second;
  ASSERT_TRUE(nav2); ASSERT_TRUE(gps2);
  EXPECT_EQ(refNav2, *nav2);
  EXPECT_EQ(refGps2, *gps2);

  vision_msgs::Detection2D refFace;
  refFace.header.frame_id = "test_optical_frame";
  refFace.bbox.center.x = 1211.986266880293;
  refFace.bbox.center.y = 190.98374914167999;
  refFace.bbox.size_x = 129.99221789883268;
  refFace.bbox.size_y = 105.99771114671549;
  refFace.results.emplace_back();
  refFace.results.back().id = 1;
  refFace.results.back().score = 0.28;
  refFace.results.back().pose.pose.orientation.w = 1;
  ASSERT_TRUE(m->getFaces());
  EXPECT_EQ("test_optical_frame", m->getFaces()->header.frame_id);
  ASSERT_EQ(1u, m->getFaces()->detections.size());
  EXPECT_EQ(refFace, m->getFaces()->detections[0]);

  EXPECT_EQ(p->cache.faces().size(), 2);
  EXPECT_GT(p->cache.imu().size(), 2000);
  EXPECT_GT(p->cache.zeroRollPitchTF().size(), 290);
  EXPECT_GE(p->cache.gnssPosition().size(), 3);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::console::initialize();
  ros::console::set_logger_level("ros.gpmf_metadata_extractor", ros::console::Level::Debug);
  ros::console::set_logger_level("ros.gpmf_metadata_extractor.gpmf.dump", ros::console::Level::Info);
  ros::console::set_logger_level("ros.movie_publisher", ros::console::Level::Debug);
  ros::console::set_logger_level("ros.movie_publisher.pluginlib", ros::console::Level::Info);
  ros::console::notifyLoggerLevelsChanged();
  ros::Time::init();
  return RUN_ALL_TESTS();
}
