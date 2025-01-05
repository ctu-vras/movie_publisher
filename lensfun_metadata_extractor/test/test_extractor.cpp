// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for LensfunMetadataExtractor.
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
#include <sensor_msgs/distortion_models.h>

#include "LensfunMetadataExtractor.h"

struct TestData
{
  cras::optional<std::string> make;
  cras::optional<std::string> model;
  cras::optional<std::string> lensMake;
  cras::optional<std::string> lensModel;
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
  cras::optional<int> getRotation() override { return this->data.rotation; }
private:
  TestData data;
};

std::pair<movie_publisher::MetadataManager::Ptr, movie_publisher::MetadataExtractor::Ptr>
getExtractor(const TestData& data, const size_t width, const size_t height, const bool isStillImage)
{
  // auto log = std::make_shared<cras::MemoryLogHelper>();
  auto log = std::make_shared<cras::NodeLogHelper>();

  auto manager = std::make_shared<movie_publisher::MetadataManager>(log, width, height);
  auto extractor = std::make_shared<movie_publisher::LensfunMetadataExtractor>(
    log, manager, width, height, isStillImage, TEST_DATA_DIR);
  manager->addExtractor(extractor);
  auto extra = std::make_shared<TestExtractor>(log, data);
  manager->addExtractor(extra);
  return std::make_pair(manager, extractor);
}

TEST(LensfunMetadataExtractor, FairphoneStill)  // NOLINT
{
  const TestData data = {"Fairphone", "FP4", cras::nullopt, cras::nullopt, 5.58, 0};
  auto [m, e] = getExtractor(data, 4000, 3000, true);

  EXPECT_FALSE(e->getCropFactor());
  EXPECT_FALSE(e->getSensorSizeMM());
  EXPECT_FALSE(e->getFocalLengthMM());
  EXPECT_FALSE(e->getDistortion());
}

TEST(LensfunMetadataExtractor, FairphoneMovie)  // NOLINT
{
  const TestData data = {"Fairphone", "FP4", cras::nullopt, cras::nullopt, cras::nullopt, 90};
  auto [m, e] = getExtractor(data, 1920, 1080, false);

  EXPECT_FALSE(e->getCropFactor());
  EXPECT_FALSE(e->getSensorSizeMM());
  EXPECT_FALSE(e->getFocalLengthMM());
  EXPECT_FALSE(e->getDistortion());
}

TEST(LensfunMetadataExtractor, LumixStill)  // NOLINT
{
  const TestData data = {"Panasonic", "DMC-GX80", cras::nullopt, "LUMIX G VARIO 12-32/F3.5-5.6", 30.0, 0};
  auto [m, e] = getExtractor(data, 4592, 3448, true);

  ASSERT_TRUE(e->getCropFactor()); EXPECT_NEAR(2.0, *e->getCropFactor(), 1e-6);
  ASSERT_TRUE(e->getSensorSizeMM());
  EXPECT_NEAR(18.0, e->getSensorSizeMM()->first, 1e-6); EXPECT_NEAR(13.515679, e->getSensorSizeMM()->second, 1e-6);
  EXPECT_FALSE(e->getFocalLengthMM());
  ASSERT_TRUE(e->getDistortion());
  EXPECT_EQ(sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL, e->getDistortion()->first);
  ASSERT_EQ(8, e->getDistortion()->second.size());
  EXPECT_NEAR(0.0168137, e->getDistortion()->second[0], 2e-3);
  EXPECT_NEAR(-0.00191267, e->getDistortion()->second[1], 1e-5);
  EXPECT_NEAR(0.0, e->getDistortion()->second[2], 1e-6);
  EXPECT_NEAR(0.0, e->getDistortion()->second[3], 1e-6);
  EXPECT_NEAR(0.0408514, e->getDistortion()->second[4], 1e-4);
  EXPECT_NEAR(0.00559995, e->getDistortion()->second[5], 2e-3);
  EXPECT_NEAR(-0.000484464, e->getDistortion()->second[6], 1e-4);
  EXPECT_NEAR(0.0399399, e->getDistortion()->second[7], 1e-4);
}

TEST(LensfunMetadataExtractor, LumixMovie)  // NOLINT
{
  const TestData data = {"Panasonic", "DMC-GX80", cras::nullopt, "LUMIX G VARIO 12-32/F3.5-5.6", 28.0, 0};
  auto [m, e] = getExtractor(data, 1920, 1080, false);

  // We add a special video profile in the test database, but it only contains the camera, not the lens.
  ASSERT_TRUE(e->getCropFactor()); EXPECT_NEAR(2.1785714, *e->getCropFactor(), 1e-6);
  ASSERT_TRUE(e->getSensorSizeMM());
  EXPECT_NEAR(16.5245901, e->getSensorSizeMM()->first, 1e-6);
  EXPECT_NEAR(9.295082, e->getSensorSizeMM()->second, 1e-6);
  EXPECT_FALSE(e->getFocalLengthMM());
  EXPECT_FALSE(e->getDistortion());
}

TEST(LensfunMetadataExtractor, FfmpegProcessed)  // NOLINT
{
  const TestData data = {cras::nullopt, cras::nullopt, cras::nullopt, cras::nullopt, cras::nullopt, 0};
  auto [m, e] = getExtractor(data, 1920, 1080, false);

  EXPECT_FALSE(e->getCropFactor());
  EXPECT_FALSE(e->getSensorSizeMM());
  EXPECT_FALSE(e->getFocalLengthMM());
  EXPECT_FALSE(e->getDistortion());
}

TEST(LensfunMetadataExtractor, IphoneStill)  // NOLINT
{
  const TestData data = {
    "Apple", "iPhone 12 mini", "Apple", "iPhone 12 mini back dual wide camera 4.2mm f/1.6", 4.2, 0};
  auto [m, e] = getExtractor(data, 4032, 3024, true);

  EXPECT_FALSE(e->getCropFactor());
  EXPECT_FALSE(e->getSensorSizeMM());
  EXPECT_FALSE(e->getFocalLengthMM());
  EXPECT_FALSE(e->getDistortion());
}

TEST(LensfunMetadataExtractor, IphoneMovie)  // NOLINT
{
  const TestData data = {
    "Apple", "iPhone SE (2nd generation)", cras::nullopt, "iPhone SE (2nd generation) back camera 3.99mm f/1.8", 4.2, 0
  };
  auto [m, e] = getExtractor(data, 1920, 1080, false);

  EXPECT_FALSE(e->getCropFactor());
  EXPECT_FALSE(e->getSensorSizeMM());
  EXPECT_FALSE(e->getFocalLengthMM());
  EXPECT_FALSE(e->getDistortion());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::console::initialize();
  ros::console::set_logger_level("ros.lensfun_metadata_extractor", ros::console::Level::Debug);
  ros::console::set_logger_level("ros.movie_publisher", ros::console::Level::Debug);
  ros::console::set_logger_level("ros.movie_publisher.pluginlib", ros::console::Level::Info);
  ros::console::notifyLoggerLevelsChanged();
  return RUN_ALL_TESTS();
}
