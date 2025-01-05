// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for LibexifMetadataExtractor.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <memory>
#include <string>

#include <cras_cpp_common/log_utils/memory.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/param_utils/param_helper.hpp>
#include <cras_cpp_common/string_utils/ros.hpp>
#include <movie_publisher/metadata_manager.h>

#include "LibexifMetadataExtractor.h"

movie_publisher::MetadataManager::Ptr getExtractor(const std::string& filename, const size_t width, const size_t height)
{
  // auto log = std::make_shared<cras::MemoryLogHelper>();
  auto log = std::make_shared<cras::NodeLogHelper>();

  auto manager = std::make_shared<movie_publisher::MetadataManager>(log, width, height);
  auto extractor = std::make_shared<movie_publisher::LibexifMetadataExtractor>(log, manager, filename, width, height);
  manager->addExtractor(extractor);
  return manager;
}

TEST(LibexifMetadataExtractor, FairphoneStill)  // NOLINT
{
  auto m = getExtractor(std::string(TEST_DATA_DIR) + "/fairphone/IMG_20241125_024757.jpg", 4000, 3000);

  ASSERT_TRUE(m->getRotation()); EXPECT_EQ(0, *m->getRotation());
  ASSERT_TRUE(m->getCreationTime()); EXPECT_EQ(cras::parseTime("2024-11-25 02:48:00.585"), *m->getCreationTime());
  EXPECT_FALSE(m->getCameraSerialNumber());
  ASSERT_TRUE(m->getCameraMake()); EXPECT_EQ("Fairphone", *m->getCameraMake());
  ASSERT_TRUE(m->getCameraModel()); EXPECT_EQ("FP4", *m->getCameraModel());
  EXPECT_FALSE(m->getLensMake());
  EXPECT_FALSE(m->getLensModel());
  EXPECT_FALSE(m->getCropFactor());
  EXPECT_FALSE(m->getSensorSizeMM());
  EXPECT_FALSE(m->getFocalLength35MM());
  ASSERT_TRUE(m->getFocalLengthMM()); EXPECT_NEAR(5.58, *m->getFocalLengthMM(), 1e-3);
  EXPECT_FALSE(m->getFocalLengthPx());
  EXPECT_FALSE(m->getRollPitch());
  EXPECT_FALSE(m->getAcceleration());
  EXPECT_FALSE(m->getAzimuth());
  auto [nav, gps] = m->getGNSSPosition();
  ASSERT_TRUE(nav);
  EXPECT_EQ("", nav->header.frame_id);
  EXPECT_EQ(ros::Time{}, nav->header.stamp);
  EXPECT_NEAR(50.132083, nav->latitude, 1e-6);
  EXPECT_NEAR(14.438111, nav->longitude, 1e-6);
  EXPECT_NEAR(350.571, nav->altitude, 1e-6);
  EXPECT_EQ(0, nav->status.status);
  ASSERT_TRUE(gps);
  EXPECT_EQ("", gps->header.frame_id);
  EXPECT_EQ(ros::Time{}, gps->header.stamp);
  EXPECT_NEAR(50.132083, gps->latitude, 1e-6);
  EXPECT_NEAR(14.438111, gps->longitude, 1e-6);
  EXPECT_NEAR(350.571, gps->altitude, 1e-6);
  EXPECT_EQ(0, gps->status.status);
}

TEST(LibexifMetadataExtractor, FairphoneMovie)  // NOLINT
{
  auto m = getExtractor(std::string(TEST_DATA_DIR) + "/fairphone/VID_20240815_143536.mp4", 1920, 1080);

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

TEST(LibexifMetadataExtractor, LumixStill)  // NOLINT
{
  auto m = getExtractor(std::string(TEST_DATA_DIR) + "/lumix/P1260334.JPG", 4592, 3448);

  ASSERT_TRUE(m->getRotation()); EXPECT_EQ(0, *m->getRotation());
  ASSERT_TRUE(m->getCreationTime()); EXPECT_EQ(cras::parseTime("2020-02-17 05:59:01.726"), *m->getCreationTime());
  ASSERT_TRUE(m->getCameraSerialNumber()); EXPECT_EQ("XGR1606080157-06FX2042462B", *m->getCameraSerialNumber());
  ASSERT_TRUE(m->getCameraMake()); EXPECT_EQ("Panasonic", *m->getCameraMake());
  ASSERT_TRUE(m->getCameraModel()); EXPECT_EQ("DMC-GX80", *m->getCameraModel());
  EXPECT_FALSE(m->getLensMake());
  ASSERT_TRUE(m->getLensModel()); EXPECT_EQ("LUMIX G VARIO 12-32/F3.5-5.6", *m->getLensModel());
  ASSERT_TRUE(m->getCropFactor()); EXPECT_NEAR(2.0, *m->getCropFactor(), 1e-6);
  ASSERT_TRUE(m->getSensorSizeMM());
  EXPECT_NEAR(18.0, m->getSensorSizeMM()->first, 1e-6); EXPECT_NEAR(13.515679, m->getSensorSizeMM()->second, 1e-6);
  ASSERT_TRUE(m->getFocalLength35MM()); EXPECT_NEAR(60.0, *m->getFocalLength35MM(), 1e-6);
  ASSERT_TRUE(m->getFocalLengthMM()); EXPECT_NEAR(30.0, *m->getFocalLengthMM(), 1e-3);
  ASSERT_TRUE(m->getFocalLengthPx()); EXPECT_NEAR(7653.3333333, *m->getFocalLengthPx(), 1e-3);
  ASSERT_TRUE(m->getRollPitch());
  EXPECT_NEAR(0.0244346, m->getRollPitch()->first, 1e-3); EXPECT_NEAR(0.1727875, m->getRollPitch()->second, 1e-3);
  ASSERT_TRUE(m->getAcceleration());
  EXPECT_NEAR(-2.261682, m->getAcceleration()->x, 1e-3);
  EXPECT_NEAR(0.173975, m->getAcceleration()->y, 1e-3);
  EXPECT_NEAR(9.533862, m->getAcceleration()->z, 1e-3);
  EXPECT_FALSE(m->getAzimuth());
  auto [nav, gps] = m->getGNSSPosition();
  EXPECT_FALSE(nav); EXPECT_FALSE(gps);
}

TEST(LibexifMetadataExtractor, LumixMovie)  // NOLINT
{
  auto m = getExtractor(std::string(TEST_DATA_DIR) + "/lumix/P1260657.MP4", 1920, 1080);

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

TEST(LibexifMetadataExtractor, FfmpegProcessed)  // NOLINT
{
  auto m = getExtractor(std::string(TEST_DATA_DIR) + "/ffmpeg_processed/P1320029.MP4.mp4", 1920, 1080);

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

TEST(LibexifMetadataExtractor, IphoneStill)  // NOLINT
{
  auto m = getExtractor(std::string(TEST_DATA_DIR) + "/iphone/20241005_160034_IMG_4998.jpg", 4032, 3024);

  ASSERT_TRUE(m->getRotation()); EXPECT_EQ(0, *m->getRotation());
  ASSERT_TRUE(m->getCreationTime()); EXPECT_EQ(cras::parseTime("2024-10-05 16:00:34.359+0200"), *m->getCreationTime());
  EXPECT_FALSE(m->getCameraSerialNumber());
  ASSERT_TRUE(m->getCameraMake()); EXPECT_EQ("Apple", *m->getCameraMake());
  ASSERT_TRUE(m->getCameraModel()); EXPECT_EQ("iPhone 12 mini", *m->getCameraModel());
  ASSERT_TRUE(m->getLensMake()); EXPECT_EQ("Apple", *m->getLensMake());
  ASSERT_TRUE(m->getLensModel()); EXPECT_EQ("iPhone 12 mini back dual wide camera 4.2mm f/1.6", *m->getLensModel());
  ASSERT_TRUE(m->getCropFactor()); EXPECT_NEAR(6.2, *m->getCropFactor(), 1e-2);
  ASSERT_TRUE(m->getSensorSizeMM());
  EXPECT_NEAR(5.8153846, m->getSensorSizeMM()->first, 1e-6); EXPECT_NEAR(4.361538, m->getSensorSizeMM()->second, 1e-6);
  ASSERT_TRUE(m->getFocalLength35MM()); EXPECT_NEAR(26.0, *m->getFocalLength35MM(), 1e-6);
  ASSERT_TRUE(m->getFocalLengthMM()); EXPECT_NEAR(4.2, *m->getFocalLengthMM(), 1e-3);
  ASSERT_TRUE(m->getFocalLengthPx()); EXPECT_NEAR(2912, *m->getFocalLengthPx(), 1e-3);
  EXPECT_FALSE(m->getRollPitch());
  ASSERT_TRUE(m->getAcceleration());
  EXPECT_NEAR(-9.7978364, m->getAcceleration()->x, 1e-3);
  EXPECT_NEAR(-0.01354990, m->getAcceleration()->y, 1e-3);
  EXPECT_NEAR(0.4519192, m->getAcceleration()->z, 1e-3);
  ASSERT_TRUE(m->getAzimuth());
  const auto azimuth = *m->getAzimuth();
  EXPECT_EQ("", azimuth.header.frame_id);
  EXPECT_EQ(ros::Time{}, azimuth.header.stamp);
  EXPECT_NEAR(269.5859069, azimuth.azimuth, 1e-6);
  EXPECT_NEAR(0, azimuth.variance, 1e-6);
  EXPECT_EQ(compass_msgs::Azimuth::UNIT_DEG, azimuth.unit);
  EXPECT_EQ(compass_msgs::Azimuth::ORIENTATION_NED, azimuth.orientation);
  EXPECT_EQ(compass_msgs::Azimuth::REFERENCE_GEOGRAPHIC, azimuth.reference);
  const auto [nav, gps] = m->getGNSSPosition();
  ASSERT_TRUE(nav);
  EXPECT_EQ("", nav->header.frame_id);
  EXPECT_EQ(ros::Time{}, nav->header.stamp);
  EXPECT_NEAR(50.083869, nav->latitude, 1e-6);
  EXPECT_NEAR(14.3915167, nav->longitude, 1e-6);
  EXPECT_NEAR(336.083355, nav->altitude, 1e-6);
  EXPECT_EQ(0, nav->status.status);
  ASSERT_TRUE(gps);
  EXPECT_EQ("", gps->header.frame_id);
  EXPECT_EQ(ros::Time{}, gps->header.stamp);
  EXPECT_NEAR(50.083869, gps->latitude, 1e-6);
  EXPECT_NEAR(14.3915167, gps->longitude, 1e-6);
  EXPECT_NEAR(336.083355, gps->altitude, 1e-6);
  EXPECT_EQ(0, gps->status.status);
  EXPECT_NEAR(5.688000, gps->speed, 1e-6);
  EXPECT_NEAR(12.44612669, gps->err_horz, 1e-6);
  EXPECT_NEAR(154.9060696, gps->position_covariance[0 * 3 + 0], 1e-6);
  EXPECT_NEAR(154.9060696, gps->position_covariance[1 * 3 + 1], 1e-6);
  EXPECT_NEAR(10000 * 10000, gps->position_covariance[2 * 3 + 2], 1e-6);
  EXPECT_EQ(gps_common::GPSFix::COVARIANCE_TYPE_APPROXIMATED, gps->position_covariance_type);
}

TEST(LibexifMetadataExtractor, IphoneMovie)  // NOLINT
{
  auto m = getExtractor(std::string(TEST_DATA_DIR) + "/iphone/IMG_2585.MOV", 1920, 1080);

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
  ros::console::set_logger_level("ros.libexif_metadata_extractor", ros::console::Level::Debug);
  ros::console::set_logger_level("ros.libexif_metadata_extractor.libexif.dump", ros::console::Level::Info);
  ros::console::set_logger_level("ros.movie_publisher", ros::console::Level::Debug);
  ros::console::set_logger_level("ros.movie_publisher.pluginlib", ros::console::Level::Info);
  ros::console::notifyLoggerLevelsChanged();
  return RUN_ALL_TESTS();
}
