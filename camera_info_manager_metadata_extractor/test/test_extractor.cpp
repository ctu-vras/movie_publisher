// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for CameraInfoManagerMetadataExtractor.
 * \author Martin Pecka
 */
// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include "gtest/gtest.h"

#include <memory>
#include <string>

#include <cras_cpp_common/log_utils/memory.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/param_utils/param_helper.hpp>
#include <movie_publisher/metadata_manager.h>
#include <sensor_msgs/distortion_models.h>

#include "CamInfoManagerMetadataExtractor.h"

static const std::string TEST_URL_PREFIX = std::string("file://") + TEST_DATA_DIR;  // NOLINT

struct TestData
{
  cras::optional<std::string> make;
  cras::optional<std::string> model;
  cras::optional<std::string> lensMake;
  cras::optional<std::string> lensModel;
  cras::optional<std::string> cameraSerial;
  cras::optional<double> focalLengthMM;
  cras::optional<int> rotation;
};

class TestExtractor : public movie_publisher::MetadataExtractor
{
public:
  TestExtractor(const cras::LogHelperPtr& log, const TestData& data) : MetadataExtractor(log), data(data) {}
  std::string getName() const override { return "test"; }
  int getPriority() const override { return 100; }
  cras::optional<std::string> getCameraMake() override { return this->data.make; }
  cras::optional<std::string> getCameraModel() override { return this->data.model; }
  cras::optional<std::string> getLensMake() override { return this->data.lensMake; }
  cras::optional<std::string> getLensModel() override { return this->data.lensModel; }
  cras::optional<double> getFocalLengthMM() override { return this->data.focalLengthMM; }
  cras::optional<std::string> getCameraSerialNumber() override { return this->data.cameraSerial; }
  cras::optional<int> getRotation() override { return this->data.rotation; }
private:
  TestData data;
};

std::pair<movie_publisher::MetadataManager::Ptr, movie_publisher::MetadataExtractor::Ptr>
getExtractor(const TestData& data, const size_t width, const size_t height)
{
  // auto log = std::make_shared<cras::MemoryLogHelper>();
  auto log = std::make_shared<cras::NodeLogHelper>();

  auto manager = std::make_shared<movie_publisher::MetadataManager>(log, width, height);
  auto extractor = std::make_shared<movie_publisher::CamInfoManagerMetadataExtractor>(
    log, manager, width, height,
    std::list{TEST_URL_PREFIX + "/${NAME}-${FOCAL_LENGTH}.yaml", TEST_URL_PREFIX + "/${NAME}.yaml"});
  manager->addExtractor(extractor);
  auto extra = std::make_shared<TestExtractor>(log, data);
  manager->addExtractor(extra);
  return std::make_pair(manager, extractor);
}

TEST(CamInfoManagerMetadataExtractor, FairphoneStill)  // NOLINT
{
  const TestData data = {"Fairphone", "FP4", cras::nullopt, cras::nullopt, cras::nullopt, 5.58, 0};
  auto [m, e] = getExtractor(data, 4000, 3000);

  EXPECT_FALSE(e->getIntrinsicMatrix());
  EXPECT_FALSE(e->getDistortion());
}

TEST(CamInfoManagerMetadataExtractor, FairphoneMovie)  // NOLINT
{
  const TestData data = {"Fairphone", "FP4", cras::nullopt, cras::nullopt, cras::nullopt, cras::nullopt, 90};
  auto [m, e] = getExtractor(data, 1920, 1080);

  EXPECT_FALSE(e->getIntrinsicMatrix());
  EXPECT_FALSE(e->getDistortion());
}

TEST(CamInfoManagerMetadataExtractor, LumixStill)  // NOLINT
{
  const TestData data = {
    "Panasonic", "DMC-GX80", cras::nullopt, "LUMIX G VARIO 12-32/F3.5-5.6", "XGR1606080157-06FX2042462B", 30.0, 0
  };
  auto [m, e] = getExtractor(data, 4592, 3448);

  ASSERT_TRUE(e->getIntrinsicMatrix());
  const auto& K = *e->getIntrinsicMatrix();

  EXPECT_NEAR(907.5058667024193, K[0], 1e-6);
  EXPECT_NEAR(0.0, K[1], 1e-6);
  EXPECT_NEAR(1023.674064941674, K[2], 1e-6);
  EXPECT_NEAR(0.0, K[3], 1e-6);
  EXPECT_NEAR(910.4449275805216, K[4], 1e-6);
  EXPECT_NEAR(818.9322168491329, K[5], 1e-6);
  EXPECT_NEAR(0.0, K[6], 1e-6);
  EXPECT_NEAR(0.0, K[7], 1e-6);
  EXPECT_NEAR(1.0, K[8], 1e-6);

  ASSERT_TRUE(e->getDistortion());
  EXPECT_EQ(sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL, e->getDistortion()->first);
  ASSERT_EQ(8, e->getDistortion()->second.size());
  EXPECT_NEAR(0.0168137, e->getDistortion()->second[0], 1e-6);
  EXPECT_NEAR(-0.00191267, e->getDistortion()->second[1], 1e-6);
  EXPECT_NEAR(0.0, e->getDistortion()->second[2], 1e-6);
  EXPECT_NEAR(0.0, e->getDistortion()->second[3], 1e-6);
  EXPECT_NEAR(0.0408514, e->getDistortion()->second[4], 1e-6);
  EXPECT_NEAR(0.00559995, e->getDistortion()->second[5], 1e-6);
  EXPECT_NEAR(-0.000484464, e->getDistortion()->second[6], 1e-6);
  EXPECT_NEAR(0.0399399, e->getDistortion()->second[7], 1e-6);
}

TEST(CamInfoManagerMetadataExtractor, LumixMovie)  // NOLINT
{
  const TestData data = {
    "Panasonic", "DMC-GX80", cras::nullopt, "LUMIX G VARIO 12-32/F3.5-5.6", "XGR1606080157-06FX2042462B", 28.0, 0
  };
  auto [m, e] = getExtractor(data, 1920, 1080);

  EXPECT_FALSE(e->getIntrinsicMatrix());
  EXPECT_FALSE(e->getDistortion());
}

TEST(CamInfoManagerMetadataExtractor, FfmpegProcessed)  // NOLINT
{
  const TestData data = {cras::nullopt, cras::nullopt, cras::nullopt, cras::nullopt, cras::nullopt, cras::nullopt, 0};
  auto [m, e] = getExtractor(data, 1920, 1080);

  EXPECT_FALSE(e->getIntrinsicMatrix());
  EXPECT_FALSE(e->getDistortion());
}

TEST(CamInfoManagerMetadataExtractor, IphoneStill)  // NOLINT
{
  const TestData data = {
    "Apple", "iPhone 12 mini", "Apple", "iPhone 12 mini back dual wide camera 4.2mm f/1.6", cras::nullopt, 4.2, 0};
  auto [m, e] = getExtractor(data, 4032, 3024);

  EXPECT_FALSE(e->getIntrinsicMatrix());
  EXPECT_FALSE(e->getDistortion());
}

TEST(CamInfoManagerMetadataExtractor, IphoneMovie)  // NOLINT
{
  const TestData data = {
    "Apple", "iPhone SE (2nd generation)", cras::nullopt, "iPhone SE (2nd generation) back camera 3.99mm f/1.8",
    cras::nullopt, 4.2, 0
  };
  auto [m, e] = getExtractor(data, 1920, 1080);

  EXPECT_FALSE(e->getIntrinsicMatrix());
  EXPECT_FALSE(e->getDistortion());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::console::initialize();
  ros::console::set_logger_level(
    "ros.camera_info_manager_metadata_extractor.caminfo_manager", ros::console::Level::Debug);
  ros::console::set_logger_level("ros.movie_publisher", ros::console::Level::Debug);
  ros::console::set_logger_level("ros.movie_publisher.pluginlib", ros::console::Level::Info);
  ros::console::notifyLoggerLevelsChanged();
  return RUN_ALL_TESTS();
}
