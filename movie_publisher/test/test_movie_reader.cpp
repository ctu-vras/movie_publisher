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
#include <movie_publisher/movie_info.h>
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

TEST(MovieReader, TestEncoding)  // NOLINT
{
  // auto log = std::make_shared<cras::MemoryLogHelper>();
  auto log = std::make_shared<cras::NodeLogHelper>();

  XmlRpc::XmlRpcValue paramsXml;
  paramsXml.begin();
  auto adapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(paramsXml, "");
  auto params = std::make_shared<cras::BoundParamHelper>(log, adapter);

  auto m = movie_publisher::MovieReader(log, params);
  movie_publisher::MovieOpenConfig config(params);
  config.setFrameId("test");
  config.setOpticalFrameId("test_optical_frame");
  config.setAllowYUVFallback(false);
  config.setTimestampSource(movie_publisher::TimestampSource::FromMetadata);
  auto maybeMovie = m.open(std::string(TEST_DATA_DIR) + "/fairphone/VID_20240815_143536.mp4", config);
  ASSERT_TRUE(maybeMovie.has_value());
  auto movie = maybeMovie.value();
  ASSERT_NE(nullptr, movie);
  ASSERT_NE(nullptr, movie->staticMetadata());
  EXPECT_FALSE(movie->info()->isStillImage());
  EXPECT_TRUE(movie->info()->isSeekable());
  EXPECT_EQ(363, movie->info()->streamNumFrames());
  EXPECT_TRUE(movie->staticMetadata()->getOpticalFrameTF());

  auto maybeNextFrame = movie->nextFrame();
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
  movie_publisher::MovieOpenConfig config(params);
  config.setFrameId("test");
  config.setOpticalFrameId("test_optical_frame");
  config.setTimestampSource(movie_publisher::TimestampSource::FromMetadata);
  auto maybeMovie = m.open(std::string(TEST_DATA_DIR) + "/fairphone/IMG_20241125_024757.jpg", config);
  ASSERT_TRUE(maybeMovie.has_value());
  auto movie = maybeMovie.value();
  ASSERT_NE(nullptr, movie);
  ASSERT_NE(nullptr, movie->staticMetadata());
  EXPECT_TRUE(movie->info()->isStillImage());
  EXPECT_FALSE(movie->info()->isSeekable());
  EXPECT_EQ(1, movie->info()->streamNumFrames());
  EXPECT_TRUE(movie->staticMetadata()->getOpticalFrameTF().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getCameraInfo().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getImu().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getAzimuth().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getZeroRollPitchTF().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getMagneticField().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getFaces().has_value());

  auto maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  auto playbackState = maybeNextFrame->first;
  auto image = maybeNextFrame->second;
  EXPECT_EQ(movie_publisher::StreamTime(0, 0), playbackState.streamTime());
  EXPECT_NEAR(cras::parseTime("2024-11-25 02:48:00.585").toSec(), image->header.stamp.toSec(), 5.0);
  EXPECT_EQ("test_optical_frame", image->header.frame_id);
  EXPECT_EQ(4000, image->width);
  EXPECT_EQ(3000, image->height);
  EXPECT_EQ("yuv422", image->encoding);
  EXPECT_EQ(false, image->is_bigendian);
  EXPECT_THAT(image->step, alignedStep(4000, 1, 2));

  maybeNextFrame = movie->nextFrame();
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
  movie_publisher::MovieOpenConfig config(params);
  config.setFrameId("test");
  config.setOpticalFrameId("test_optical_frame");
  config.setTimestampSource(movie_publisher::TimestampSource::FromMetadata);
  auto maybeMovie = m.open(std::string(TEST_DATA_DIR) + "/fairphone/VID_20240815_143536.mp4", config);
  ASSERT_TRUE(maybeMovie.has_value());
  auto movie = maybeMovie.value();
  ASSERT_NE(nullptr, movie);
  ASSERT_NE(nullptr, movie->staticMetadata());
  EXPECT_FALSE(movie->info()->isStillImage());
  EXPECT_TRUE(movie->info()->isSeekable());
  EXPECT_EQ(363, movie->info()->streamNumFrames());
  EXPECT_TRUE(movie->staticMetadata()->getOpticalFrameTF().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getCameraInfo().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getImu().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getAzimuth().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getZeroRollPitchTF().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getMagneticField().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getFaces().has_value());

  auto maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  auto playbackState = maybeNextFrame->first;
  auto image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(movie_publisher::StreamTime(0, 38400000), playbackState.streamTime());
  EXPECT_NEAR(cras::parseTime("2024-08-15 12:35:51").toSec(), image->header.stamp.toSec(), 5.0);
  EXPECT_EQ("test_optical_frame", image->header.frame_id);
  EXPECT_EQ(1080, image->width);
  EXPECT_EQ(1920, image->height);
  EXPECT_EQ("yuv422", image->encoding);
  EXPECT_EQ(false, image->is_bigendian);
  EXPECT_THAT(image->step, alignedStep(1080, 1, 2));

  maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  playbackState = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(movie_publisher::StreamTime(0, 71722222), playbackState.streamTime());

  maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  playbackState = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(movie_publisher::StreamTime(0, 105044444), playbackState.streamTime());

  EXPECT_TRUE(movie->seek(movie_publisher::StreamTime(2.5)).has_value());

  maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  playbackState = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(movie_publisher::StreamTime(2, 504288889), playbackState.streamTime());

  EXPECT_TRUE(movie->seek(movie_publisher::StreamTime(0, 0)).has_value());

  maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  playbackState = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(movie_publisher::StreamTime(0, 38400000), playbackState.streamTime());
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
  movie_publisher::MovieOpenConfig config(params);
  config.setFrameId("test");
  config.setOpticalFrameId("test_optical_frame");
  config.setTimestampSource(movie_publisher::TimestampSource::FromMetadata);
  auto maybeMovie = m.open(std::string(TEST_DATA_DIR) + "/lumix/P1260334.JPG", config);
  ASSERT_TRUE(maybeMovie.has_value());
  auto movie = maybeMovie.value();
  ASSERT_NE(nullptr, movie);
  ASSERT_NE(nullptr, movie->staticMetadata());
  EXPECT_TRUE(movie->info()->isStillImage());
  EXPECT_FALSE(movie->info()->isSeekable());
  EXPECT_EQ(1, movie->info()->streamNumFrames());
  EXPECT_TRUE(movie->staticMetadata()->getOpticalFrameTF().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getAzimuth().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getMagneticField().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getFaces().has_value());

  auto maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  auto playbackState = maybeNextFrame->first;
  auto image = maybeNextFrame->second;
  EXPECT_EQ(movie_publisher::StreamTime(0, 0), playbackState.streamTime());
  EXPECT_NEAR(cras::parseTime("2020-02-17 05:59:01.726").toSec(), image->header.stamp.toSec(), 5.0);
  EXPECT_EQ("test_optical_frame", image->header.frame_id);
  EXPECT_EQ(4592, image->width);
  EXPECT_EQ(3448, image->height);
  EXPECT_EQ("yuv422", image->encoding);
  EXPECT_EQ(false, image->is_bigendian);
  EXPECT_THAT(image->step, alignedStep(4592, 1, 2));

  maybeNextFrame = movie->nextFrame();
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
  movie_publisher::MovieOpenConfig config(params);
  config.setFrameId("test");
  config.setOpticalFrameId("test_optical_frame");
  config.setTimestampSource(movie_publisher::TimestampSource::FromMetadata);
  auto maybeMovie = m.open(std::string(TEST_DATA_DIR) + "/lumix/P1260657.MP4", config);
  ASSERT_TRUE(maybeMovie.has_value());
  auto movie = maybeMovie.value();
  ASSERT_NE(nullptr, movie);
  ASSERT_NE(nullptr, movie->staticMetadata());
  EXPECT_FALSE(movie->info()->isStillImage());
  EXPECT_TRUE(movie->info()->isSeekable());
  EXPECT_EQ(132, movie->info()->streamNumFrames());
  EXPECT_TRUE(movie->staticMetadata()->getOpticalFrameTF().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getAzimuth().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getMagneticField().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getFaces().has_value());

  auto maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  auto playbackState = maybeNextFrame->first;
  auto image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(movie_publisher::StreamTime(0, 0), playbackState.streamTime());
  EXPECT_NEAR(cras::parseTime("2020-02-20 04:35:42.953").toSec(), image->header.stamp.toSec(), 5.0);
  EXPECT_EQ("test_optical_frame", image->header.frame_id);
  EXPECT_EQ(1920, image->width);
  EXPECT_EQ(1080, image->height);
  EXPECT_EQ("yuv422", image->encoding);
  EXPECT_EQ(false, image->is_bigendian);
  EXPECT_THAT(image->step, alignedStep(1920, 1, 2));

  maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  playbackState = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(movie_publisher::StreamTime(0, 40000000), playbackState.streamTime());

  maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  playbackState = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(movie_publisher::StreamTime(0, 80000000), playbackState.streamTime());

  EXPECT_TRUE(movie->seek(movie_publisher::StreamTime(2.5)).has_value());

  maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  playbackState = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(movie_publisher::StreamTime(2, 520000000), playbackState.streamTime());

  EXPECT_TRUE(movie->seek(movie_publisher::StreamTime(0, 0)).has_value());

  maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  playbackState = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(movie_publisher::StreamTime(0, 0), playbackState.streamTime());
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
  movie_publisher::MovieOpenConfig config(params);
  config.setFrameId("test");
  config.setOpticalFrameId("test_optical_frame");
  config.setTimestampSource(movie_publisher::TimestampSource::FromMetadata);
  auto maybeMovie = m.open(std::string(TEST_DATA_DIR) + "/ffmpeg_processed/P1320029.MP4.mp4", config);
  ASSERT_TRUE(maybeMovie.has_value());
  auto movie = maybeMovie.value();
  ASSERT_NE(nullptr, movie);
  ASSERT_NE(nullptr, movie->staticMetadata());
  EXPECT_FALSE(movie->info()->isStillImage());
  EXPECT_TRUE(movie->info()->isSeekable());
  EXPECT_EQ(984, movie->info()->streamNumFrames());
  EXPECT_TRUE(movie->staticMetadata()->getOpticalFrameTF().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getAzimuth().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getMagneticField().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getFaces().has_value());

  auto maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  auto playbackState = maybeNextFrame->first;
  auto image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(movie_publisher::StreamTime(0, 0), playbackState.streamTime());
  // EXPECT_NEAR(cras::parseTime("2020-02-20 04:35:42.953").toSec(), image->header.stamp.toSec(), 5.0);
  EXPECT_EQ("test_optical_frame", image->header.frame_id);
  EXPECT_EQ(1920, image->width);
  EXPECT_EQ(1080, image->height);
  EXPECT_EQ("yuv422", image->encoding);
  EXPECT_EQ(false, image->is_bigendian);
  EXPECT_THAT(image->step, alignedStep(1920, 1, 2));

  maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  playbackState = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(movie_publisher::StreamTime(0, 20000000), playbackState.streamTime());

  maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  playbackState = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(movie_publisher::StreamTime(0, 40000000), playbackState.streamTime());

  EXPECT_TRUE(movie->seek(movie_publisher::StreamTime(2.5)).has_value());

  maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  playbackState = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(movie_publisher::StreamTime(2, 500000000), playbackState.streamTime());

  EXPECT_TRUE(movie->seek(movie_publisher::StreamTime(0, 0)).has_value());

  maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  playbackState = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(movie_publisher::StreamTime(0, 0), playbackState.streamTime());
}

class TestMetaProcessor : public movie_publisher::MovieMetadataProcessor
{
public:
  cras::expected<void, std::string> onOpen(const movie_publisher::MovieInfo::ConstPtr& info,
    const movie_publisher::MovieOpenConfig& config) override
  {
    this->info = info;
    this->config = config;
    return {};
  }
  cras::expected<void, std::string> onMetadataReady(
    const std::shared_ptr<movie_publisher::TimedMetadataExtractor>& metadataExtractor) override
  {
    this->extractor = metadataExtractor;
    return {};
  }
  cras::expected<void, std::string> onClose() override
  {
    this->wasClosed = true;
    return {};
  }
  cras::expected<void, std::string> onSeek(const movie_publisher::StreamTime& time) override
  {
    this->lastSeekTime = time;
    return {};
  }
  cras::expected<void, std::string> processFrame(const sensor_msgs::ImageConstPtr& image,
    const movie_publisher::MoviePlaybackState& playbackState) override
  {
    this->lastImage = image;
    this->lastPlaybackState = playbackState;
    return {};
  }
  cras::expected<void, std::string> processImage(const sensor_msgs::ImageConstPtr& image,
    const cras::optional<sensor_msgs::CameraInfo>& cameraInfoMsg) override
  {
    this->lastImage = image;
    this->lastImageCameraInfo = cameraInfoMsg;
    return {};
  }
  cras::expected<void, std::string> processOpticalTf(const geometry_msgs::TransformStamped& opticalTfMsg) override
  {
    this->lastOpticalTfMsgs.push_back(opticalTfMsg);
    return {};
  }

  cras::expected<void, std::string> processNavSatFix(const sensor_msgs::NavSatFix& navSatFixMsg) override
  {
    this->lastNavSatFixes.push_back(navSatFixMsg);
    return {};
  }
  cras::expected<void, std::string> processGps(const gps_common::GPSFix& gpsMsg) override
  {
    this->lastGpsFixes.push_back(gpsMsg);
    return {};
  }

  cras::expected<void, std::string> processCameraInfo(const sensor_msgs::CameraInfo& cameraInfoMsg) override
  {
    this->lastCameraInfos.push_back(cameraInfoMsg);
    return {};
  }
  cras::expected<void, std::string> processAzimuth(const compass_msgs::Azimuth& azimuthMsg) override
  {
    this->lastAzimuths.push_back(azimuthMsg);
    return {};
  }
  cras::expected<void, std::string> processImu(const sensor_msgs::Imu& imuMsg) override
  {
    this->lastImus.push_back(imuMsg);
    return {};
  }
  cras::expected<void, std::string> processZeroRollPitchTf(
    const geometry_msgs::TransformStamped& zeroRollPitchTfMsg) override
  {
    this->lastZeroRollPitchTFs.push_back(zeroRollPitchTfMsg);
    return {};
  }
  cras::expected<void, std::string> processMagneticField(const sensor_msgs::MagneticField& magneticFieldMsg) override
  {
    this->lastMagneticFields.push_back(magneticFieldMsg);
    return {};
  }
  cras::expected<void, std::string> processFaces(const vision_msgs::Detection2DArray& facesMsg) override
  {
    this->lastFaces.push_back(facesMsg);
    return {};
  }

  void reset()
  {
    this->wasClosed = false;
    this->lastSeekTime.reset();
    this->lastPlaybackState.reset();
    this->lastImage.reset();
    this->lastImageCameraInfo.reset();
    this->lastOpticalTfMsgs.clear();
    this->lastNavSatFixes.clear();
    this->lastGpsFixes.clear();
    this->lastCameraInfos.clear();
    this->lastAzimuths.clear();
    this->lastImus.clear();
    this->lastZeroRollPitchTFs.clear();
    this->lastMagneticFields.clear();
    this->lastFaces.clear();
  }

  movie_publisher::MovieInfo::ConstPtr info;
  cras::optional<movie_publisher::MovieOpenConfig> config;
  movie_publisher::TimedMetadataExtractor::Ptr extractor;
  bool wasClosed {false};
  cras::optional<movie_publisher::StreamTime> lastSeekTime;
  cras::optional<movie_publisher::MoviePlaybackState> lastPlaybackState;
  sensor_msgs::ImageConstPtr lastImage;
  cras::optional<sensor_msgs::CameraInfo> lastImageCameraInfo;
  std::vector<geometry_msgs::TransformStamped> lastOpticalTfMsgs;
  std::vector<sensor_msgs::NavSatFix> lastNavSatFixes;
  std::vector<gps_common::GPSFix> lastGpsFixes;
  std::vector<sensor_msgs::CameraInfo> lastCameraInfos;
  std::vector<compass_msgs::Azimuth> lastAzimuths;
  std::vector<sensor_msgs::Imu> lastImus;
  std::vector<geometry_msgs::TransformStamped> lastZeroRollPitchTFs;
  std::vector<sensor_msgs::MagneticField> lastMagneticFields;
  std::vector<vision_msgs::Detection2DArray> lastFaces;
};

TEST(MovieReader, FfmpegProcessedWithSidecar)  // NOLINT
{
  // auto log = std::make_shared<cras::MemoryLogHelper>();
  auto log = std::make_shared<cras::NodeLogHelper>();

  XmlRpc::XmlRpcValue paramsXml;
  paramsXml.begin();
  auto adapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(paramsXml, "");
  auto params = std::make_shared<cras::BoundParamHelper>(log, adapter);

  auto m = movie_publisher::MovieReader(log, params);
  movie_publisher::MovieOpenConfig config(params);
  config.setFrameId("test");
  config.setOpticalFrameId("test_optical_frame");
  config.setTimestampSource(movie_publisher::TimestampSource::FromMetadata);
  auto processor = std::make_shared<TestMetaProcessor>();
  config.metadataProcessors().push_back(processor);

  const auto startStamp = ros::Time(1735727696, 789000000);

  auto maybeMovie = m.open(std::string(TEST_DATA_DIR) + "/ffmpeg_processed/P1320029.MP4", config);
  ASSERT_TRUE(maybeMovie.has_value());
  auto movie = maybeMovie.value();
  ASSERT_NE(nullptr, movie);

  ASSERT_NE(nullptr, processor->info);
  ASSERT_TRUE(processor->config.has_value());
  EXPECT_FALSE(processor->wasClosed);

  const auto refCamInfo = []
  {
    sensor_msgs::CameraInfo msg;
    msg.header.frame_id = "test_optical_frame";
    msg.width = 800;
    msg.height = 600;
    msg.K = {1000.0, 0.0, 960.0, 0.0, 1000.0, 540.0, 0.0, 0.0, 1.0};
    msg.P = {
      msg.K[0], msg.K[1], msg.K[2], 0.0,
      msg.K[3], msg.K[4], msg.K[5], 0.0,
      msg.K[6], msg.K[7], msg.K[8], 0.0
    };
    msg.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    msg.distortion_model = "rational_polynomial";
    msg.D = {1.0, -1.0, 0.0, 1.0, 1.0};
    msg.binning_x = 1;
    msg.binning_y = 2;
    msg.roi.x_offset = 1;
    msg.roi.y_offset = 2;
    msg.roi.height = 400;
    msg.roi.width = 600;
    msg.roi.do_rectify = 1;
    return msg;
  }();

  const auto refNavSat = []
  {
    sensor_msgs::NavSatFix msg;
    msg.header.frame_id = "test";
    msg.latitude = 50.0;
    msg.longitude = 15.0;
    msg.altitude = 500.0;
    msg.position_covariance = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
    msg.position_covariance_type = 1;
    msg.status.status = 1;
    msg.status.service = 2;
    return msg;
  }();

  const auto refGpsFix = []
  {
    gps_common::GPSFix msg;
    msg.header.frame_id = "test";
    msg.latitude = 50.0;
    msg.longitude = 15.0;
    msg.altitude = 500.0;
    msg.position_covariance = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
    msg.position_covariance_type = 1;
    msg.track = 1.0;
    msg.speed = 2.0;
    msg.climb = 3.0;
    msg.pitch = 4.0;
    msg.roll = 5.0;
    msg.dip = 6.0;
    msg.time = 7.0;
    msg.gdop = 8.0;
    msg.pdop = 9.0;
    msg.hdop = 10.0;
    msg.vdop = 11.0;
    msg.tdop = 12.0;
    msg.err = 13.0;
    msg.err_horz = 14.0;
    msg.err_vert = 15.0;
    msg.err_track = 16.0;
    msg.err_speed = 17.0;
    msg.err_climb = 18.0;
    msg.err_time = 19.0;
    msg.err_pitch = 20.0;
    msg.err_roll = 21.0;
    msg.err_dip = 22.0;
    msg.status.header.frame_id = "test";
    msg.status.status = 1;
    msg.status.satellites_used = 2;
    msg.status.satellite_used_prn = {1, 2};
    msg.status.satellites_visible = 3;
    msg.status.satellite_visible_prn = {1, 2, 3};
    msg.status.satellite_visible_z = {4, 5, 6};
    msg.status.satellite_visible_azimuth = {7, 8, 9};
    msg.status.satellite_visible_snr = {10, 11, 12};
    msg.status.position_source = 1;
    msg.status.orientation_source = 2;
    msg.status.motion_source = 3;
    return msg;
  }();

  const auto refAzimuth = []
  {
    compass_msgs::Azimuth msg;
    msg.header.frame_id = "test";
    msg.azimuth = 2.0;
    msg.variance = 4.0;
    msg.unit = 1;
    msg.orientation = 2;
    msg.reference = 3;
    return msg;
  }();

  const auto refMagneticField = []
  {
    sensor_msgs::MagneticField msg;
    msg.header.frame_id = "test";
    msg.magnetic_field.x = 1e-5;
    msg.magnetic_field.y = 2e-5;
    msg.magnetic_field.z = 3e-5;
    msg.magnetic_field_covariance = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
    return msg;
  }();

  const auto refImu = []
  {
    sensor_msgs::Imu msg;
    msg.header.frame_id = "test";
    msg.orientation.x = 1.0;
    msg.orientation.y = 2.0;
    msg.orientation.z = 3.0;
    msg.orientation.w = 4.0;
    msg.orientation_covariance = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
    msg.angular_velocity.x = 1.0;
    msg.angular_velocity.y = 2.0;
    msg.angular_velocity.z = 3.0;
    msg.angular_velocity_covariance = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
    msg.linear_acceleration.x = 1.0;
    msg.linear_acceleration.y = 2.0;
    msg.linear_acceleration.z = 3.0;
    msg.linear_acceleration_covariance = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
    return msg;
  }();

  const auto refOpticalFrameTF = []
  {
    geometry_msgs::TransformStamped msg;
    msg.header.frame_id = "test";
    msg.child_frame_id = "test_optical_frame";
    msg.transform.rotation.x = 1.0;
    msg.transform.rotation.y = 2.0;
    msg.transform.rotation.z = 3.0;
    msg.transform.rotation.w = 4.0;
    return msg;
  }();

  const auto refOpticalFrameTF90 = []
  {
    geometry_msgs::TransformStamped msg;
    msg.header.frame_id = "test";
    msg.child_frame_id = "test_optical_frame";
    msg.transform.rotation.x = M_SQRT1_2;
    msg.transform.rotation.y = 0.0;
    msg.transform.rotation.z = M_SQRT1_2;
    msg.transform.rotation.w = 0.0;
    return msg;
  }();

  const auto refOpticalFrameTF0 = []
  {
    geometry_msgs::TransformStamped msg;
    msg.header.frame_id = "test";
    msg.child_frame_id = "test_optical_frame";
    msg.transform.rotation.x = -0.5;
    msg.transform.rotation.y = 0.5;
    msg.transform.rotation.z = -0.5;
    msg.transform.rotation.w = 0.5;
    return msg;
  }();

  const auto refZeroRollPitchTF = []
  {
    geometry_msgs::TransformStamped msg;
    msg.header.frame_id = "test";
    msg.child_frame_id = "test_zero_roll_pitch";
    msg.transform.translation.x = 1.0;
    msg.transform.translation.y = 2.0;
    msg.transform.translation.z = 3.0;
    msg.transform.rotation.w = 1.0;
    return msg;
  }();

  const auto refFaces = []
  {
    vision_msgs::Detection2DArray msg;
    msg.header.frame_id = "test_optical_frame";

    vision_msgs::Detection2D det1;
    det1.header.frame_id = "test_optical_frame";
    det1.bbox.center.x = 1.0;
    det1.bbox.center.y = 2.0;
    det1.bbox.size_x = 3.0;
    det1.bbox.size_y = 4.0;
    vision_msgs::ObjectHypothesisWithPose hyp1;
    hyp1.score = 1.0;
    det1.results.push_back(hyp1);
    msg.detections.push_back(det1);

    vision_msgs::Detection2D det2;
    det2.header.frame_id = "test_optical_frame";
    det2.bbox.center.x = 5.0;
    det2.bbox.center.y = 6.0;
    det2.bbox.size_x = 7.0;
    det2.bbox.size_y = 8.0;
    msg.detections.push_back(det2);
    return msg;
  }();

  ASSERT_NE(nullptr, movie->staticMetadata());
  EXPECT_FALSE(movie->info()->isStillImage());
  EXPECT_TRUE(movie->info()->isSeekable());
  EXPECT_EQ(984, movie->info()->streamNumFrames());
  auto& meta = *movie->staticMetadata();
  ASSERT_TRUE(meta.getCameraGeneralName().has_value()); EXPECT_EQ("Test camera", *meta.getCameraGeneralName());
  ASSERT_TRUE(meta.getCameraUniqueName().has_value());
  EXPECT_EQ("Test camera 1234-5678", *meta.getCameraUniqueName());
  ASSERT_TRUE(meta.getCameraSerialNumber().has_value()); EXPECT_EQ("1234-5678", *meta.getCameraSerialNumber());
  ASSERT_TRUE(meta.getCameraMake().has_value()); EXPECT_EQ("Test", *meta.getCameraMake());
  ASSERT_TRUE(meta.getCameraModel().has_value()); EXPECT_EQ("camera", *meta.getCameraModel());
  ASSERT_TRUE(meta.getLensMake().has_value()); EXPECT_EQ("Test", *meta.getLensMake());
  ASSERT_TRUE(meta.getLensModel().has_value()); EXPECT_EQ("lens", *meta.getLensModel());
  ASSERT_TRUE(meta.getRotation().has_value()); EXPECT_EQ(90, *meta.getRotation());
  ASSERT_TRUE(meta.getCreationTime().has_value());
  EXPECT_EQ(cras::parseTime("2025-01-01 12:34:56.789+0200"), *meta.getCreationTime());
  ASSERT_TRUE(meta.getCropFactor().has_value()); EXPECT_NEAR(2.0, *meta.getCropFactor(), 1e-9);
  ASSERT_TRUE(meta.getSensorSizeMM().has_value());
  EXPECT_NEAR(18.0, meta.getSensorSizeMM()->first, 1e-9); EXPECT_NEAR(12.0, meta.getSensorSizeMM()->second, 1e-9);
  ASSERT_TRUE(meta.getFocalLength35MM().has_value()); EXPECT_NEAR(72.0, *meta.getFocalLength35MM(), 1e-9);
  ASSERT_TRUE(meta.getFocalLengthMM().has_value()); EXPECT_NEAR(36.0, *meta.getFocalLengthMM(), 1e-9);
  ASSERT_TRUE(meta.getFocalLengthPx().has_value()); EXPECT_NEAR(1000.0, *meta.getFocalLengthPx(), 1e-9);
  ASSERT_TRUE(meta.getIntrinsicMatrix().has_value());
  EXPECT_EQ(refCamInfo.K, *meta.getIntrinsicMatrix());
  ASSERT_TRUE(meta.getDistortion().has_value());
  EXPECT_EQ(refCamInfo.distortion_model, meta.getDistortion()->first);
  ASSERT_EQ(5, meta.getDistortion()->second.size());
  EXPECT_EQ(refCamInfo.D, meta.getDistortion()->second);
  ASSERT_TRUE(meta.getGNSSPosition().first.has_value());
  EXPECT_EQ(refNavSat, *meta.getGNSSPosition().first);
  ASSERT_TRUE(meta.getGNSSPosition().second.has_value());
  EXPECT_EQ(refGpsFix, *meta.getGNSSPosition().second);
  ASSERT_TRUE(meta.getAzimuth().has_value());
  EXPECT_EQ(refAzimuth, *meta.getAzimuth());
  ASSERT_TRUE(meta.getMagneticField().has_value());
  EXPECT_EQ(refMagneticField, *meta.getMagneticField());
  ASSERT_TRUE(meta.getRollPitch().has_value());
  EXPECT_NEAR(1.0, meta.getRollPitch()->first, 1e-9);
  EXPECT_NEAR(2.0, meta.getRollPitch()->second, 1e-9);
  ASSERT_TRUE(meta.getAcceleration().has_value());
  EXPECT_EQ(refImu.linear_acceleration, *meta.getAcceleration());
  ASSERT_TRUE(meta.getAngularVelocity().has_value());
  EXPECT_EQ(refImu.angular_velocity, *meta.getAngularVelocity());
  ASSERT_TRUE(meta.getFaces().has_value());
  EXPECT_EQ(refFaces, *meta.getFaces());
  ASSERT_TRUE(meta.getCameraInfo().has_value());
  EXPECT_EQ(refCamInfo, *meta.getCameraInfo());
  ASSERT_TRUE(meta.getImu().has_value());
  EXPECT_EQ(refImu, *meta.getImu());
  ASSERT_TRUE(meta.getOpticalFrameTF().has_value());
  EXPECT_EQ(refOpticalFrameTF.transform, *meta.getOpticalFrameTF());
  ASSERT_TRUE(meta.getZeroRollPitchTF().has_value());
  EXPECT_EQ(refZeroRollPitchTF.transform, *meta.getZeroRollPitchTF());

  // Before the first frame, the timed callbacks have been called with static metadata
  EXPECT_EQ(nullptr, processor->lastImage);
  EXPECT_FALSE(processor->lastImageCameraInfo.has_value());
  EXPECT_FALSE(processor->lastPlaybackState.has_value());
  EXPECT_FALSE(processor->lastSeekTime.has_value());
  ASSERT_EQ(1u, processor->lastOpticalTfMsgs.size());
  EXPECT_THAT(refOpticalFrameTF, matchesUpToStamp(processor->lastOpticalTfMsgs[0]));
  EXPECT_EQ(startStamp, processor->lastOpticalTfMsgs[0].header.stamp);
  ASSERT_EQ(1u, processor->lastZeroRollPitchTFs.size());
  EXPECT_THAT(refZeroRollPitchTF, matchesUpToStamp(processor->lastZeroRollPitchTFs[0]));
  EXPECT_EQ(startStamp, processor->lastZeroRollPitchTFs[0].header.stamp);
  ASSERT_EQ(1u, processor->lastNavSatFixes.size());
  EXPECT_THAT(refNavSat, matchesUpToStamp(processor->lastNavSatFixes[0]));
  EXPECT_EQ(startStamp, processor->lastNavSatFixes[0].header.stamp);
  ASSERT_EQ(1u, processor->lastGpsFixes.size());
  EXPECT_THAT(refGpsFix, matchesUpToStamp(processor->lastGpsFixes[0]));
  EXPECT_EQ(startStamp, processor->lastGpsFixes[0].header.stamp);
  ASSERT_EQ(0u, processor->lastCameraInfos.size());
  ASSERT_EQ(1u, processor->lastAzimuths.size());
  EXPECT_THAT(refAzimuth, matchesUpToStamp(processor->lastAzimuths[0]));
  EXPECT_EQ(startStamp, processor->lastAzimuths[0].header.stamp);
  ASSERT_EQ(1u, processor->lastMagneticFields.size());
  EXPECT_THAT(refMagneticField, matchesUpToStamp(processor->lastMagneticFields[0]));
  EXPECT_EQ(startStamp, processor->lastMagneticFields[0].header.stamp);
  ASSERT_EQ(1u, processor->lastImus.size());
  EXPECT_THAT(refImu, matchesUpToStamp(processor->lastImus[0]));
  EXPECT_EQ(startStamp, processor->lastImus[0].header.stamp);
  ASSERT_EQ(1u, processor->lastFaces.size());
  EXPECT_THAT(refFaces, matchesUpToStamp(processor->lastFaces[0]));
  EXPECT_EQ(startStamp, processor->lastFaces[0].header.stamp);
  processor->reset();

  // Extract the first frame
  auto maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  ASSERT_NE(nullptr, processor->lastImage);
  ASSERT_TRUE(processor->lastPlaybackState.has_value());
  auto playbackState = processor->lastPlaybackState.value();
  auto image = processor->lastImage;
  EXPECT_EQ(movie_publisher::StreamTime(0, 0), playbackState.streamTime());
  EXPECT_EQ(startStamp, playbackState.rosTime());
  EXPECT_EQ("test_optical_frame", image->header.frame_id);
  EXPECT_EQ(startStamp, image->header.stamp);
  EXPECT_EQ(1080, image->width);
  EXPECT_EQ(1920, image->height);
  EXPECT_EQ("yuv422", image->encoding);
  EXPECT_EQ(false, image->is_bigendian);
  EXPECT_THAT(image->step, alignedStep(1080, 1, 2));
  ASSERT_EQ(1u, processor->lastOpticalTfMsgs.size());
  ASSERT_THAT(refOpticalFrameTF90, matchesUpToStamp(processor->lastOpticalTfMsgs[0]));

  auto refNavSat1 = refNavSat;
  refNavSat1.latitude = 50.1;
  refNavSat1.longitude = 15.1;
  refNavSat1.altitude = 500.1;
  auto refGpsFix1 = refGpsFix;
  refGpsFix1.latitude = 50.1;
  refGpsFix1.longitude = 15.1;
  refGpsFix1.altitude = 500.1;

  ASSERT_TRUE(meta.getRotation().has_value());
  EXPECT_EQ(90, *meta.getRotation());
  ASSERT_EQ(0u, processor->lastZeroRollPitchTFs.size());
  ASSERT_EQ(1u, processor->lastNavSatFixes.size());
  EXPECT_THAT(refNavSat1, matchesUpToStamp(processor->lastNavSatFixes[0]));
  EXPECT_EQ(startStamp, processor->lastNavSatFixes[0].header.stamp);
  ASSERT_EQ(1u, processor->lastGpsFixes.size());
  EXPECT_THAT(refGpsFix1, matchesUpToStamp(processor->lastGpsFixes[0]));
  EXPECT_EQ(startStamp, processor->lastGpsFixes[0].header.stamp);
  ASSERT_EQ(1u, processor->lastCameraInfos.size());
  ASSERT_EQ(0u, processor->lastAzimuths.size());
  ASSERT_EQ(0u, processor->lastMagneticFields.size());
  ASSERT_EQ(0u, processor->lastImus.size());
  ASSERT_EQ(0u, processor->lastFaces.size());

  processor->reset();

  // Extract the second frame
  auto stamp = startStamp + ros::Duration(0, 20000000);
  maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  ASSERT_NE(nullptr, processor->lastImage);
  ASSERT_TRUE(processor->lastPlaybackState.has_value());
  playbackState = processor->lastPlaybackState.value();
  image = processor->lastImage;
  EXPECT_EQ(movie_publisher::StreamTime(0, 20000000), playbackState.streamTime());
  EXPECT_EQ(stamp, playbackState.rosTime());
  EXPECT_EQ("test_optical_frame", image->header.frame_id);
  EXPECT_EQ(stamp, image->header.stamp);
  EXPECT_EQ(1080, image->width);
  EXPECT_EQ(1920, image->height);
  EXPECT_EQ("yuv422", image->encoding);
  EXPECT_EQ(false, image->is_bigendian);
  EXPECT_THAT(image->step, alignedStep(1080, 1, 2));

  auto refNavSat2 = refNavSat;
  refNavSat2.latitude = 50.2;
  refNavSat2.longitude = 15.2;
  refNavSat2.altitude = 500.2;
  auto refGpsFix2 = refGpsFix;
  refGpsFix2.latitude = 50.2;
  refGpsFix2.longitude = 15.2;
  refGpsFix2.altitude = 500.2;

  ASSERT_TRUE(meta.getRotation().has_value());
  EXPECT_EQ(90, *meta.getRotation());
  ASSERT_EQ(2u, processor->lastOpticalTfMsgs.size());
  EXPECT_THAT(refOpticalFrameTF90, matchesUpToStamp(processor->lastOpticalTfMsgs[0]));
  EXPECT_EQ(startStamp + ros::Duration(0, 10000000), processor->lastOpticalTfMsgs[0].header.stamp);
  EXPECT_THAT(refOpticalFrameTF90, matchesUpToStamp(processor->lastOpticalTfMsgs[1]));
  EXPECT_EQ(stamp, processor->lastOpticalTfMsgs[1].header.stamp);
  ASSERT_EQ(1u, processor->lastNavSatFixes.size());
  EXPECT_THAT(refNavSat2, matchesUpToStamp(processor->lastNavSatFixes[0]));
  EXPECT_EQ(stamp, processor->lastNavSatFixes[0].header.stamp);
  ASSERT_EQ(1u, processor->lastGpsFixes.size());
  EXPECT_THAT(refGpsFix2, matchesUpToStamp(processor->lastGpsFixes[0]));
  EXPECT_EQ(stamp, processor->lastGpsFixes[0].header.stamp);

  processor->reset();

  for (size_t i = 0; i < 48; ++i)
  {
    maybeNextFrame = movie->nextFrame();
    ASSERT_TRUE(maybeNextFrame.has_value());
    processor->reset();
  }

  // Extract the 51st frame
  stamp = startStamp + ros::Duration(1, 0);
  maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  ASSERT_NE(nullptr, processor->lastImage);
  ASSERT_TRUE(processor->lastPlaybackState.has_value());
  playbackState = processor->lastPlaybackState.value();
  image = processor->lastImage;
  EXPECT_EQ(movie_publisher::StreamTime(1, 0), playbackState.streamTime());
  EXPECT_EQ(stamp, playbackState.rosTime());
  EXPECT_EQ("test_optical_frame", image->header.frame_id);
  EXPECT_EQ(stamp, image->header.stamp);
  EXPECT_EQ(1920, image->width);
  EXPECT_EQ(1080, image->height);
  EXPECT_EQ("yuv422", image->encoding);
  EXPECT_EQ(false, image->is_bigendian);
  EXPECT_THAT(image->step, alignedStep(1920, 1, 2));
  ASSERT_TRUE(meta.getRotation().has_value());
  EXPECT_EQ(180, *meta.getRotation());
  ASSERT_EQ(1u, processor->lastOpticalTfMsgs.size());
  EXPECT_EQ("test", processor->lastOpticalTfMsgs[0].header.frame_id);
  EXPECT_EQ("test_optical_frame", processor->lastOpticalTfMsgs[0].child_frame_id);
  EXPECT_EQ(stamp, processor->lastOpticalTfMsgs[0].header.stamp);
  EXPECT_NEAR(0.0, processor->lastOpticalTfMsgs[0].transform.translation.x, 1e-9);
  EXPECT_NEAR(0.0, processor->lastOpticalTfMsgs[0].transform.translation.y, 1e-9);
  EXPECT_NEAR(0.0, processor->lastOpticalTfMsgs[0].transform.translation.z, 1e-9);
  EXPECT_NEAR(0.5, processor->lastOpticalTfMsgs[0].transform.rotation.x, 1e-9);
  EXPECT_NEAR(0.5, processor->lastOpticalTfMsgs[0].transform.rotation.y, 1e-9);
  EXPECT_NEAR(0.5, processor->lastOpticalTfMsgs[0].transform.rotation.z, 1e-9);
  EXPECT_NEAR(0.5, processor->lastOpticalTfMsgs[0].transform.rotation.w, 1e-9);
  processor->reset();

  for (size_t i = 0; i < 4; ++i)
  {
    maybeNextFrame = movie->nextFrame();
    ASSERT_TRUE(maybeNextFrame.has_value());
    processor->reset();
  }

  // Extract the 56th frame
  stamp = startStamp + ros::Duration(1, 100000000);
  maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  ASSERT_NE(nullptr, processor->lastImage);
  ASSERT_TRUE(processor->lastPlaybackState.has_value());
  playbackState = processor->lastPlaybackState.value();
  image = processor->lastImage;
  EXPECT_EQ(movie_publisher::StreamTime(1, 100000000), playbackState.streamTime());
  EXPECT_EQ(stamp, playbackState.rosTime());
  EXPECT_EQ("test_optical_frame", image->header.frame_id);
  EXPECT_EQ(stamp, image->header.stamp);
  EXPECT_EQ(1080, image->width);
  EXPECT_EQ(1920, image->height);
  EXPECT_EQ("yuv422", image->encoding);
  EXPECT_EQ(false, image->is_bigendian);
  EXPECT_THAT(image->step, alignedStep(1080, 1, 2));
  ASSERT_TRUE(meta.getRotation().has_value());
  EXPECT_EQ(90, *meta.getRotation());
  ASSERT_EQ(1u, processor->lastOpticalTfMsgs.size());
  EXPECT_EQ("test", processor->lastOpticalTfMsgs[0].header.frame_id);
  EXPECT_EQ("test_optical_frame", processor->lastOpticalTfMsgs[0].child_frame_id);
  EXPECT_EQ(stamp, processor->lastOpticalTfMsgs[0].header.stamp);
  EXPECT_NEAR(0.0, processor->lastOpticalTfMsgs[0].transform.translation.x, 1e-9);
  EXPECT_NEAR(0.0, processor->lastOpticalTfMsgs[0].transform.translation.y, 1e-9);
  EXPECT_NEAR(0.0, processor->lastOpticalTfMsgs[0].transform.translation.z, 1e-9);
  EXPECT_NEAR(M_SQRT1_2, processor->lastOpticalTfMsgs[0].transform.rotation.x, 1e-9);
  EXPECT_NEAR(0.0, processor->lastOpticalTfMsgs[0].transform.rotation.y, 1e-9);
  EXPECT_NEAR(M_SQRT1_2, processor->lastOpticalTfMsgs[0].transform.rotation.z, 1e-9);
  EXPECT_NEAR(0.0, processor->lastOpticalTfMsgs[0].transform.rotation.w, 1e-9);
  processor->reset();

  // Extract the 57th frame
  stamp = startStamp + ros::Duration(1, 120000000);
  maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  ASSERT_NE(nullptr, processor->lastImage);
  ASSERT_TRUE(processor->lastPlaybackState.has_value());
  playbackState = processor->lastPlaybackState.value();
  image = processor->lastImage;
  EXPECT_EQ(movie_publisher::StreamTime(1, 120000000), playbackState.streamTime());
  EXPECT_EQ(stamp, playbackState.rosTime());
  EXPECT_EQ("test_optical_frame", image->header.frame_id);
  EXPECT_EQ(stamp, image->header.stamp);
  EXPECT_EQ(1080, image->width);
  EXPECT_EQ(1920, image->height);
  EXPECT_EQ("yuv422", image->encoding);
  EXPECT_EQ(false, image->is_bigendian);
  EXPECT_THAT(image->step, alignedStep(1080, 1, 2));
  ASSERT_TRUE(meta.getRotation().has_value());
  EXPECT_EQ(90, *meta.getRotation());
  ASSERT_EQ(1u, processor->lastOpticalTfMsgs.size());
  EXPECT_EQ("test", processor->lastOpticalTfMsgs[0].header.frame_id);
  EXPECT_EQ("test_optical_frame", processor->lastOpticalTfMsgs[0].child_frame_id);
  EXPECT_EQ(stamp, processor->lastOpticalTfMsgs[0].header.stamp);
  EXPECT_NEAR(0.0, processor->lastOpticalTfMsgs[0].transform.translation.x, 1e-9);
  EXPECT_NEAR(0.0, processor->lastOpticalTfMsgs[0].transform.translation.y, 1e-9);
  EXPECT_NEAR(0.0, processor->lastOpticalTfMsgs[0].transform.translation.z, 1e-9);
  EXPECT_NEAR(M_SQRT1_2, processor->lastOpticalTfMsgs[0].transform.rotation.x, 1e-9);
  EXPECT_NEAR(0.0, processor->lastOpticalTfMsgs[0].transform.rotation.y, 1e-9);
  EXPECT_NEAR(M_SQRT1_2, processor->lastOpticalTfMsgs[0].transform.rotation.z, 1e-9);
  EXPECT_NEAR(0.0, processor->lastOpticalTfMsgs[0].transform.rotation.w, 1e-9);
  processor->reset();

  // Extract the 58th frame
  stamp = startStamp + ros::Duration(1, 140000000);
  maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  ASSERT_NE(nullptr, processor->lastImage);
  ASSERT_TRUE(processor->lastPlaybackState.has_value());
  playbackState = processor->lastPlaybackState.value();
  image = processor->lastImage;
  EXPECT_EQ(movie_publisher::StreamTime(1, 140000000), playbackState.streamTime());
  EXPECT_EQ(stamp, playbackState.rosTime());
  EXPECT_EQ("test_optical_frame", image->header.frame_id);
  EXPECT_EQ(stamp, image->header.stamp);
  EXPECT_EQ(1920, image->width);
  EXPECT_EQ(1080, image->height);
  EXPECT_EQ("yuv422", image->encoding);
  EXPECT_EQ(false, image->is_bigendian);
  EXPECT_THAT(image->step, alignedStep(1920, 1, 2));
  ASSERT_TRUE(meta.getRotation().has_value());
  EXPECT_EQ(180, *meta.getRotation());
  ASSERT_EQ(1u, processor->lastOpticalTfMsgs.size());
  EXPECT_EQ("test", processor->lastOpticalTfMsgs[0].header.frame_id);
  EXPECT_EQ("test_optical_frame", processor->lastOpticalTfMsgs[0].child_frame_id);
  EXPECT_EQ(stamp, processor->lastOpticalTfMsgs[0].header.stamp);
  EXPECT_NEAR(0.0, processor->lastOpticalTfMsgs[0].transform.translation.x, 1e-9);
  EXPECT_NEAR(0.0, processor->lastOpticalTfMsgs[0].transform.translation.y, 1e-9);
  EXPECT_NEAR(0.0, processor->lastOpticalTfMsgs[0].transform.translation.z, 1e-9);
  EXPECT_NEAR(0.5, processor->lastOpticalTfMsgs[0].transform.rotation.x, 1e-9);
  EXPECT_NEAR(0.5, processor->lastOpticalTfMsgs[0].transform.rotation.y, 1e-9);
  EXPECT_NEAR(0.5, processor->lastOpticalTfMsgs[0].transform.rotation.z, 1e-9);
  EXPECT_NEAR(0.5, processor->lastOpticalTfMsgs[0].transform.rotation.w, 1e-9);
  processor->reset();

  // Extract the 59th frame
  stamp = startStamp + ros::Duration(1, 160000000);
  maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  ASSERT_NE(nullptr, processor->lastImage);
  ASSERT_TRUE(processor->lastPlaybackState.has_value());
  playbackState = processor->lastPlaybackState.value();
  image = processor->lastImage;
  EXPECT_EQ(movie_publisher::StreamTime(1, 160000000), playbackState.streamTime());
  EXPECT_EQ(stamp, playbackState.rosTime());
  EXPECT_EQ("test_optical_frame", image->header.frame_id);
  EXPECT_EQ(stamp, image->header.stamp);
  EXPECT_EQ(1920, image->width);
  EXPECT_EQ(1080, image->height);
  EXPECT_EQ("yuv422", image->encoding);
  EXPECT_EQ(false, image->is_bigendian);
  EXPECT_THAT(image->step, alignedStep(1920, 1, 2));
  ASSERT_EQ(0u, processor->lastOpticalTfMsgs.size());
  processor->reset();

  ASSERT_TRUE(movie->seek({0, 0}).has_value());

  // Extract the first frame again after seek
  maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  ASSERT_NE(nullptr, processor->lastImage);
  ASSERT_TRUE(processor->lastPlaybackState.has_value());
  playbackState = processor->lastPlaybackState.value();
  image = processor->lastImage;
  EXPECT_EQ(movie_publisher::StreamTime(0, 0), playbackState.streamTime());
  EXPECT_EQ(startStamp, playbackState.rosTime());
  EXPECT_EQ("test_optical_frame", image->header.frame_id);
  EXPECT_EQ(startStamp, image->header.stamp);
  EXPECT_EQ(1080, image->width);
  EXPECT_EQ(1920, image->height);
  EXPECT_EQ("yuv422", image->encoding);
  EXPECT_EQ(false, image->is_bigendian);
  EXPECT_THAT(image->step, alignedStep(1080, 1, 2));
  ASSERT_TRUE(meta.getRotation().has_value());
  EXPECT_EQ(90, *meta.getRotation());
  ASSERT_EQ(1u, processor->lastOpticalTfMsgs.size());
  EXPECT_EQ("test", processor->lastOpticalTfMsgs[0].header.frame_id);
  EXPECT_EQ("test_optical_frame", processor->lastOpticalTfMsgs[0].child_frame_id);
  EXPECT_EQ(startStamp, processor->lastOpticalTfMsgs[0].header.stamp);
  EXPECT_NEAR(0.0, processor->lastOpticalTfMsgs[0].transform.translation.x, 1e-9);
  EXPECT_NEAR(0.0, processor->lastOpticalTfMsgs[0].transform.translation.y, 1e-9);
  EXPECT_NEAR(0.0, processor->lastOpticalTfMsgs[0].transform.translation.z, 1e-9);
  EXPECT_NEAR(M_SQRT1_2, processor->lastOpticalTfMsgs[0].transform.rotation.x, 1e-9);
  EXPECT_NEAR(0.0, processor->lastOpticalTfMsgs[0].transform.rotation.y, 1e-9);
  EXPECT_NEAR(M_SQRT1_2, processor->lastOpticalTfMsgs[0].transform.rotation.z, 1e-9);
  EXPECT_NEAR(0.0, processor->lastOpticalTfMsgs[0].transform.rotation.w, 1e-9);
  processor->reset();
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
  movie_publisher::MovieOpenConfig config(params);
  config.setFrameId("test");
  config.setOpticalFrameId("test_optical_frame");
  config.setTimestampSource(movie_publisher::TimestampSource::FromMetadata);
  auto maybeMovie = m.open(std::string(TEST_DATA_DIR) + "/iphone/20241005_160034_IMG_4998.jpg", config);
  ASSERT_TRUE(maybeMovie.has_value());
  auto movie = maybeMovie.value();
  ASSERT_NE(nullptr, movie);
  ASSERT_NE(nullptr, movie->staticMetadata());
  EXPECT_TRUE(movie->info()->isStillImage());
  EXPECT_FALSE(movie->info()->isSeekable());
  EXPECT_EQ(1, movie->info()->streamNumFrames());
  EXPECT_TRUE(movie->staticMetadata()->getOpticalFrameTF().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getMagneticField().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getFaces().has_value());

  auto maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  auto playbackState = maybeNextFrame->first;
  auto image = maybeNextFrame->second;
  EXPECT_EQ(movie_publisher::StreamTime(0, 0), playbackState.streamTime());
  EXPECT_NEAR(cras::parseTime("2024-10-05 16:00:34.359+0200").toSec(), image->header.stamp.toSec(), 5.0);
  EXPECT_EQ("test_optical_frame", image->header.frame_id);
  EXPECT_EQ(4032, image->width);
  EXPECT_EQ(3024, image->height);
  EXPECT_EQ("yuv422", image->encoding);
  EXPECT_EQ(false, image->is_bigendian);
  EXPECT_THAT(image->step, alignedStep(4032, 1, 2));

  maybeNextFrame = movie->nextFrame();
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
  movie_publisher::MovieOpenConfig config(params);
  config.setFrameId("test");
  config.setOpticalFrameId("test_optical_frame");
  config.setTimestampSource(movie_publisher::TimestampSource::FromMetadata);
  auto maybeMovie = m.open(std::string(TEST_DATA_DIR) + "/iphone/IMG_2585.MOV", config);
  ASSERT_TRUE(maybeMovie.has_value());
  auto movie = maybeMovie.value();
  ASSERT_NE(nullptr, movie);
  ASSERT_NE(nullptr, movie->staticMetadata());
  EXPECT_FALSE(movie->info()->isStillImage());
  EXPECT_TRUE(movie->info()->isSeekable());
  EXPECT_EQ(234, movie->info()->streamNumFrames());
  EXPECT_TRUE(movie->staticMetadata()->getOpticalFrameTF().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getAzimuth().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getMagneticField().has_value());
  EXPECT_FALSE(movie->staticMetadata()->getFaces().has_value());

  auto maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  auto playbackState = maybeNextFrame->first;
  auto image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(movie_publisher::StreamTime(0, 0), playbackState.streamTime());
  EXPECT_NEAR(cras::parseTime("2024-09-26 13:48:00+0200").toSec(), image->header.stamp.toSec(), 5.0);
  EXPECT_EQ("test_optical_frame", image->header.frame_id);
  EXPECT_EQ(1920, image->width);
  EXPECT_EQ(1080, image->height);
  EXPECT_EQ("yuv422", image->encoding);
  EXPECT_EQ(false, image->is_bigendian);
  EXPECT_THAT(image->step, alignedStep(1920, 1, 2));

  maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  playbackState = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(movie_publisher::StreamTime(0, 33333333), playbackState.streamTime());

  maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  playbackState = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(movie_publisher::StreamTime(0, 66666667), playbackState.streamTime());

  EXPECT_TRUE(movie->seek(movie_publisher::StreamTime(2.5)).has_value());

  maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  playbackState = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(movie_publisher::StreamTime(2, 500000000), playbackState.streamTime());

  EXPECT_TRUE(movie->seek(movie_publisher::StreamTime(0, 0)).has_value());

  maybeNextFrame = movie->nextFrame();
  ASSERT_TRUE(maybeNextFrame.has_value());
  playbackState = maybeNextFrame->first;
  image = maybeNextFrame->second;
  ASSERT_NE(nullptr, image);
  EXPECT_EQ(movie_publisher::StreamTime(0, 0), playbackState.streamTime());
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
