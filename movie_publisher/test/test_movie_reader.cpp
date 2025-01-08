// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for MovieReader.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <list>
#include <memory>
#include <string>

#include <angles/angles.h>
#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/memory.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/string_utils/ros.hpp>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <cras_cpp_common/param_utils/get_param_adapters/xmlrpc_value.hpp>
#include <gps_common/GPSFix.h>
#include <movie_publisher/movie_reader.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <XmlRpcValue.h>
#include <sensor_msgs/image_encodings.h>

using Az = compass_msgs::Azimuth;
using Imu = sensor_msgs::Imu;
using Fix = sensor_msgs::NavSatFix;

constexpr uint32_t align(const uint32_t x, const uint32_t a)
{
  return ((x) + (a) - 1) & ~((a) - 1);
}

auto alignedStep(const uint32_t width, const uint32_t channels, const uint32_t planes = 1)
{
  const auto step = width * channels;
  return testing::AnyOf(
    testing::Eq(planes * step),
    testing::Eq(planes * align(step, 8)),
    testing::Eq(planes * align(step, 16)),
    testing::Eq(planes * align(step, 32)),
    testing::Eq(planes * align(step, 64)));
}

TEST(MovieReader, TestEncoding)  // NOLINT
{
  // auto log = std::make_shared<cras::MemoryLogHelper>();
  auto log = std::make_shared<cras::NodeLogHelper>();

  XmlRpc::XmlRpcValue paramsXml;
  paramsXml.begin();
  auto adapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(paramsXml, "");
  auto params = std::make_shared<cras::BoundParamHelper>(log, adapter);

  auto m = movie_publisher::MovieReader(log, params);
  m.setFrameId("test", "test_optical_frame");
  m.setAllowYUVFallback(false);
  auto openOk = m.open(std::string(TEST_DATA_DIR) + "/fairphone/VID_20240815_143536.mp4",
    movie_publisher::MovieReader::TimestampSource::FromMetadata);
  ASSERT_TRUE(openOk.has_value());
  EXPECT_FALSE(m.isStillImage());
  EXPECT_TRUE(m.isSeekable());
  EXPECT_TRUE(m.getOpticalFrameTF().has_value());
  EXPECT_EQ(363, m.getNumFrames());

  auto maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  auto image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(1080, image->width);
  EXPECT_EQ(1920, image->height);
  EXPECT_EQ("bgr8", image->encoding);
  EXPECT_EQ(false, image->is_bigendian);
  EXPECT_THAT(image->step, alignedStep(1080, 3));
}

TEST(MovieReader, FairphoneStill)  // NOLINT
{
  // auto log = std::make_shared<cras::MemoryLogHelper>();
  auto log = std::make_shared<cras::NodeLogHelper>();

  XmlRpc::XmlRpcValue paramsXml;
  paramsXml.begin();
  auto adapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(paramsXml, "");
  auto params = std::make_shared<cras::BoundParamHelper>(log, adapter);

  auto m = movie_publisher::MovieReader(log, params);
  m.setFrameId("test", "test_optical_frame");
  auto openOk = m.open(std::string(TEST_DATA_DIR) + "/fairphone/IMG_20241125_024757.jpg",
    movie_publisher::MovieReader::TimestampSource::FromMetadata);
  ASSERT_TRUE(openOk.has_value());
  EXPECT_TRUE(m.isStillImage());
  EXPECT_FALSE(m.isSeekable());
  EXPECT_TRUE(m.getOpticalFrameTF().has_value());
  EXPECT_EQ(1, m.getNumFrames());
  EXPECT_FALSE(m.getCameraInfoMsg().has_value());
  EXPECT_FALSE(m.getImuMsg().has_value());
  EXPECT_FALSE(m.getAzimuthMsg().has_value());
  EXPECT_FALSE(m.getZeroRollPitchTF().has_value());

  auto maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  auto stamp = maybeNextFrame->first;
  auto image = maybeNextFrame->second;
  EXPECT_EQ(ros::Time(0, 0), stamp);
  EXPECT_NEAR(cras::parseTime("2024-11-25 02:48:00.585").toSec(), image->header.stamp.toSec(), 5.0);
  EXPECT_EQ("test_optical_frame", image->header.frame_id);
  EXPECT_EQ(4000, image->width);
  EXPECT_EQ(3000, image->height);
  EXPECT_EQ("yuv422", image->encoding);
  EXPECT_EQ(false, image->is_bigendian);
  EXPECT_THAT(image->step, alignedStep(4000, 1, 2));

  maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  image = maybeNextFrame->second;
  EXPECT_EQ(nullptr, image);
}

TEST(MovieReader, FairphoneMovie)  // NOLINT
{
  // auto log = std::make_shared<cras::MemoryLogHelper>();
  auto log = std::make_shared<cras::NodeLogHelper>();

  XmlRpc::XmlRpcValue paramsXml;
  paramsXml.begin();
  auto adapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(paramsXml, "");
  auto params = std::make_shared<cras::BoundParamHelper>(log, adapter);

  auto m = movie_publisher::MovieReader(log, params);
  m.setFrameId("test", "test_optical_frame");
  auto openOk = m.open(std::string(TEST_DATA_DIR) + "/fairphone/VID_20240815_143536.mp4",
    movie_publisher::MovieReader::TimestampSource::FromMetadata);
  ASSERT_TRUE(openOk.has_value());
  EXPECT_FALSE(m.isStillImage());
  EXPECT_TRUE(m.isSeekable());
  EXPECT_TRUE(m.getOpticalFrameTF().has_value());
  EXPECT_EQ(363, m.getNumFrames());
  EXPECT_FALSE(m.getCameraInfoMsg().has_value());
  EXPECT_FALSE(m.getImuMsg().has_value());
  EXPECT_FALSE(m.getAzimuthMsg().has_value());
  EXPECT_FALSE(m.getZeroRollPitchTF().has_value());

  auto maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  auto stamp = maybeNextFrame->first;
  auto image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(ros::Time(0, 38400000), stamp);
  EXPECT_NEAR(cras::parseTime("2024-08-15 12:35:51").toSec(), image->header.stamp.toSec(), 5.0);
  EXPECT_EQ("test_optical_frame", image->header.frame_id);
  EXPECT_EQ(1080, image->width);
  EXPECT_EQ(1920, image->height);
  EXPECT_EQ("yuv422", image->encoding);
  EXPECT_EQ(false, image->is_bigendian);
  EXPECT_THAT(image->step, alignedStep(1080, 1, 2));

  maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  stamp = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(ros::Time(0, 71722222), stamp);

  maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  stamp = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(ros::Time(0, 105044444), stamp);

  EXPECT_TRUE(m.seek(ros::Time(2.5)).has_value());

  maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  stamp = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(ros::Time(2, 504288889), stamp);

  EXPECT_TRUE(m.seek(ros::Time(0, 0)).has_value());

  maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  stamp = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(ros::Time(0, 38400000), stamp);
}

TEST(MovieReader, LumixStill)  // NOLINT
{
  // auto log = std::make_shared<cras::MemoryLogHelper>();
  auto log = std::make_shared<cras::NodeLogHelper>();

  XmlRpc::XmlRpcValue paramsXml;
  paramsXml.begin();
  auto adapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(paramsXml, "");
  auto params = std::make_shared<cras::BoundParamHelper>(log, adapter);

  auto m = movie_publisher::MovieReader(log, params);
  m.setFrameId("test", "test_optical_frame");
  auto openOk = m.open(std::string(TEST_DATA_DIR) + "/lumix/P1260334.JPG",
    movie_publisher::MovieReader::TimestampSource::FromMetadata);
  ASSERT_TRUE(openOk.has_value());
  EXPECT_TRUE(m.isStillImage());
  EXPECT_FALSE(m.isSeekable());
  EXPECT_TRUE(m.getOpticalFrameTF().has_value());
  EXPECT_EQ(1, m.getNumFrames());
  EXPECT_FALSE(m.getAzimuthMsg().has_value());

  auto maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  auto stamp = maybeNextFrame->first;
  auto image = maybeNextFrame->second;
  EXPECT_EQ(ros::Time(0, 0), stamp);
  EXPECT_NEAR(cras::parseTime("2020-02-17 05:59:01.726").toSec(), image->header.stamp.toSec(), 5.0);
  EXPECT_EQ("test_optical_frame", image->header.frame_id);
  EXPECT_EQ(4592, image->width);
  EXPECT_EQ(3448, image->height);
  EXPECT_EQ("yuv422", image->encoding);
  EXPECT_EQ(false, image->is_bigendian);
  EXPECT_THAT(image->step, alignedStep(4592, 1, 2));

  maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  image = maybeNextFrame->second;
  EXPECT_EQ(nullptr, image);
}

TEST(MovieReader, LumixMovie)  // NOLINT
{
  // auto log = std::make_shared<cras::MemoryLogHelper>();
  auto log = std::make_shared<cras::NodeLogHelper>();

  XmlRpc::XmlRpcValue paramsXml;
  paramsXml.begin();
  auto adapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(paramsXml, "");
  auto params = std::make_shared<cras::BoundParamHelper>(log, adapter);

  auto m = movie_publisher::MovieReader(log, params);
  m.setFrameId("test", "test_optical_frame");
  auto openOk = m.open(std::string(TEST_DATA_DIR) + "/lumix/P1260657.MP4",
    movie_publisher::MovieReader::TimestampSource::FromMetadata);
  ASSERT_TRUE(openOk.has_value());
  EXPECT_FALSE(m.isStillImage());
  EXPECT_TRUE(m.isSeekable());
  EXPECT_TRUE(m.getOpticalFrameTF().has_value());
  EXPECT_EQ(132, m.getNumFrames());
  EXPECT_FALSE(m.getAzimuthMsg().has_value());

  auto maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  auto stamp = maybeNextFrame->first;
  auto image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(ros::Time(0, 0), stamp);
  EXPECT_NEAR(cras::parseTime("2020-02-20 04:35:42.953").toSec(), image->header.stamp.toSec(), 5.0);
  EXPECT_EQ("test_optical_frame", image->header.frame_id);
  EXPECT_EQ(1920, image->width);
  EXPECT_EQ(1080, image->height);
  EXPECT_EQ("yuv422", image->encoding);
  EXPECT_EQ(false, image->is_bigendian);
  EXPECT_THAT(image->step, alignedStep(1920, 1, 2));

  maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  stamp = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(ros::Time(0, 40000000), stamp);

  maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  stamp = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(ros::Time(0, 80000000), stamp);

  EXPECT_TRUE(m.seek(ros::Time(2.5)).has_value());

  maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  stamp = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(ros::Time(2, 520000000), stamp);

  EXPECT_TRUE(m.seek(ros::Time(0, 0)).has_value());

  maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  stamp = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(ros::Time(0, 0), stamp);
}

TEST(MovieReader, FfmpegProcessed)  // NOLINT
{
  // auto log = std::make_shared<cras::MemoryLogHelper>();
  auto log = std::make_shared<cras::NodeLogHelper>();

  XmlRpc::XmlRpcValue paramsXml;
  paramsXml.begin();
  auto adapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(paramsXml, "");
  auto params = std::make_shared<cras::BoundParamHelper>(log, adapter);

  auto m = movie_publisher::MovieReader(log, params);
  m.setFrameId("test", "test_optical_frame");
  auto openOk = m.open(std::string(TEST_DATA_DIR) + "/ffmpeg_processed/P1320029.MP4.mp4",
    movie_publisher::MovieReader::TimestampSource::FromMetadata);
  ASSERT_TRUE(openOk.has_value());
  EXPECT_FALSE(m.isStillImage());
  EXPECT_TRUE(m.isSeekable());
  EXPECT_TRUE(m.getOpticalFrameTF().has_value());
  EXPECT_EQ(984, m.getNumFrames());
  EXPECT_FALSE(m.getAzimuthMsg().has_value());

  auto maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  auto stamp = maybeNextFrame->first;
  auto image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(ros::Time(0, 0), stamp);
  // EXPECT_NEAR(cras::parseTime("2020-02-20 04:35:42.953").toSec(), image->header.stamp.toSec(), 5.0);
  EXPECT_EQ("test_optical_frame", image->header.frame_id);
  EXPECT_EQ(1920, image->width);
  EXPECT_EQ(1080, image->height);
  EXPECT_EQ("yuv422", image->encoding);
  EXPECT_EQ(false, image->is_bigendian);
  EXPECT_THAT(image->step, alignedStep(1920, 1, 2));

  maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  stamp = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(ros::Time(0, 20000000), stamp);

  maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  stamp = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(ros::Time(0, 40000000), stamp);

  EXPECT_TRUE(m.seek(ros::Time(2.5)).has_value());

  maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  stamp = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(ros::Time(2, 500000000), stamp);

  EXPECT_TRUE(m.seek(ros::Time(0, 0)).has_value());

  maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  stamp = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(ros::Time(0, 0), stamp);
}

TEST(MovieReader, IphoneStill)  // NOLINT
{
  // auto log = std::make_shared<cras::MemoryLogHelper>();
  auto log = std::make_shared<cras::NodeLogHelper>();

  XmlRpc::XmlRpcValue paramsXml;
  paramsXml.begin();
  auto adapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(paramsXml, "");
  auto params = std::make_shared<cras::BoundParamHelper>(log, adapter);

  auto m = movie_publisher::MovieReader(log, params);
  m.setFrameId("test", "test_optical_frame");
  auto openOk = m.open(std::string(TEST_DATA_DIR) + "/iphone/20241005_160034_IMG_4998.jpg",
    movie_publisher::MovieReader::TimestampSource::FromMetadata);
  ASSERT_TRUE(openOk.has_value());
  EXPECT_TRUE(m.isStillImage());
  EXPECT_FALSE(m.isSeekable());
  EXPECT_TRUE(m.getOpticalFrameTF().has_value());
  EXPECT_EQ(1, m.getNumFrames());

  auto maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  auto stamp = maybeNextFrame->first;
  auto image = maybeNextFrame->second;
  EXPECT_EQ(ros::Time(0, 0), stamp);
  EXPECT_NEAR(cras::parseTime("2024-10-05 16:00:34.359+0200").toSec(), image->header.stamp.toSec(), 5.0);
  EXPECT_EQ("test_optical_frame", image->header.frame_id);
  EXPECT_EQ(4032, image->width);
  EXPECT_EQ(3024, image->height);
  EXPECT_EQ("yuv422", image->encoding);
  EXPECT_EQ(false, image->is_bigendian);
  EXPECT_THAT(image->step, alignedStep(4032, 1, 2));

  maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  image = maybeNextFrame->second;
  EXPECT_EQ(nullptr, image);
}


TEST(MovieReader, IphoneMovie)  // NOLINT
{
  // auto log = std::make_shared<cras::MemoryLogHelper>();
  auto log = std::make_shared<cras::NodeLogHelper>();

  XmlRpc::XmlRpcValue paramsXml;
  paramsXml.begin();
  auto adapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(paramsXml, "");
  auto params = std::make_shared<cras::BoundParamHelper>(log, adapter);

  auto m = movie_publisher::MovieReader(log, params);
  m.setFrameId("test", "test_optical_frame");
  auto openOk = m.open(std::string(TEST_DATA_DIR) + "/iphone/IMG_2585.MOV",
    movie_publisher::MovieReader::TimestampSource::FromMetadata);
  ASSERT_TRUE(openOk.has_value());
  EXPECT_FALSE(m.isStillImage());
  EXPECT_TRUE(m.isSeekable());
  EXPECT_TRUE(m.getOpticalFrameTF().has_value());
  EXPECT_EQ(234, m.getNumFrames());
  EXPECT_FALSE(m.getAzimuthMsg().has_value());

  auto maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  auto stamp = maybeNextFrame->first;
  auto image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(ros::Time(0, 0), stamp);
  EXPECT_NEAR(cras::parseTime("2024-09-26 13:48:00+0200").toSec(), image->header.stamp.toSec(), 5.0);
  EXPECT_EQ("test_optical_frame", image->header.frame_id);
  EXPECT_EQ(1920, image->width);
  EXPECT_EQ(1080, image->height);
  EXPECT_EQ("yuv422", image->encoding);
  EXPECT_EQ(false, image->is_bigendian);
  EXPECT_THAT(image->step, alignedStep(1920, 1, 2));

  maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  stamp = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(ros::Time(0, 33333333), stamp);

  maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  stamp = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(ros::Time(0, 66666667), stamp);

  EXPECT_TRUE(m.seek(ros::Time(2.5)).has_value());

  maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  stamp = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(ros::Time(2, 500000000), stamp);

  EXPECT_TRUE(m.seek(ros::Time(0, 0)).has_value());

  maybeNextFrame = m.nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  stamp = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(ros::Time(0, 0), stamp);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::console::initialize();
  // ros::console::set_logger_level("ros.movie_publisher", ros::console::Level::Debug);
  ros::console::set_logger_level("ros.movie_publisher.pluginlib", ros::console::Level::Info);
  ros::console::notifyLoggerLevelsChanged();
  return RUN_ALL_TESTS();
}
