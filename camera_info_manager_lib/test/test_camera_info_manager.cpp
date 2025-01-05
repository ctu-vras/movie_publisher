// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for camera_info_manager_lib::CameraInfoManager.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <memory>
#include <string>

#include <cras_cpp_common/log_utils/memory.h>
#include <cras_cpp_common/log_utils/node.h>
#include <camera_info_manager_lib/camera_info_manager.h>
#include <sensor_msgs/distortion_models.h>

static const std::string TEST_URL_PREFIX = std::string("file://") + TEST_DATA_DIR;  // NOLINT

TEST(CameraInfoManagerLib, ConstructEmpty)  // NOLINT
{
  auto log = std::make_shared<cras::MemoryLogHelper>();
  // auto log = std::make_shared<cras::NodeLogHelper>();

  camera_info_manager_lib::CameraInfoManager manager(log, "", TEST_URL_PREFIX + "/${NAME}.yaml");
  EXPECT_FALSE(manager.isCalibrated());

  EXPECT_TRUE(manager.setCameraName("test"));
  EXPECT_TRUE(manager.isCalibrated());
}

TEST(CameraInfoManagerLib, ConstructNameDirect)  // NOLINT
{
  auto log = std::make_shared<cras::MemoryLogHelper>();
  // auto log = std::make_shared<cras::NodeLogHelper>();

  camera_info_manager_lib::CameraInfoManager manager(log, "", TEST_URL_PREFIX + "/test.yaml");
  EXPECT_TRUE(manager.isCalibrated());
}

TEST(CameraInfoManagerLib, ConstructNameSubstitution)  // NOLINT
{
  auto log = std::make_shared<cras::MemoryLogHelper>();
  // auto log = std::make_shared<cras::NodeLogHelper>();

  camera_info_manager_lib::CameraInfoManager manager(log, "test", TEST_URL_PREFIX + "/${NAME}.yaml");
  EXPECT_TRUE(manager.isCalibrated());
}

TEST(CameraInfoManagerLib, SetNameAndFocalLength)  // NOLINT
{
  // auto log = std::make_shared<cras::MemoryLogHelper>();
  auto log = std::make_shared<cras::NodeLogHelper>();

  camera_info_manager_lib::CameraInfoManager manager(log, "test", TEST_URL_PREFIX + "/${NAME}-${FOCAL_LENGTH}.yaml");
  EXPECT_FALSE(manager.isCalibrated());

  EXPECT_TRUE(manager.setFocalLength(4.2));
  EXPECT_TRUE(manager.isCalibrated());
  EXPECT_NEAR(4.2, manager.getCameraInfo().K[0], 1e-6);

  EXPECT_TRUE(manager.setFocalLength(8.4));
  EXPECT_TRUE(manager.isCalibrated());
  EXPECT_NEAR(8.4, manager.getCameraInfo().K[0], 1e-6);

  EXPECT_TRUE(manager.setFocalLength(10.0));
  EXPECT_FALSE(manager.isCalibrated());

  EXPECT_TRUE(manager.setFocalLength(8.4));
  EXPECT_TRUE(manager.isCalibrated());
  EXPECT_NEAR(8.4, manager.getCameraInfo().K[0], 1e-6);

  EXPECT_TRUE(manager.setCameraName("foo"));
  EXPECT_FALSE(manager.isCalibrated());

  EXPECT_TRUE(manager.setCameraName("test"));
  EXPECT_TRUE(manager.isCalibrated());
  EXPECT_NEAR(8.4, manager.getCameraInfo().K[0], 1e-6);
}

TEST(CameraInfoManagerLib, CheckValues)  // NOLINT
{
  auto log = std::make_shared<cras::MemoryLogHelper>();
  // auto log = std::make_shared<cras::NodeLogHelper>();

  camera_info_manager_lib::CameraInfoManager manager(log, "test", TEST_URL_PREFIX + "/${NAME}.yaml");
  EXPECT_TRUE(manager.isCalibrated());

  const auto& camInfo = manager.getCameraInfo();

  EXPECT_EQ("", camInfo.header.frame_id);
  EXPECT_EQ(ros::Time{}, camInfo.header.stamp);

  EXPECT_EQ(2048, camInfo.width);
  EXPECT_EQ(1536, camInfo.height);

  EXPECT_NEAR(907.5058667024193, camInfo.K[0], 1e-6);
  EXPECT_NEAR(0.0, camInfo.K[1], 1e-6);
  EXPECT_NEAR(1023.674064941674, camInfo.K[2], 1e-6);
  EXPECT_NEAR(0.0, camInfo.K[3], 1e-6);
  EXPECT_NEAR(910.4449275805216, camInfo.K[4], 1e-6);
  EXPECT_NEAR(818.9322168491329, camInfo.K[5], 1e-6);
  EXPECT_NEAR(0.0, camInfo.K[6], 1e-6);
  EXPECT_NEAR(0.0, camInfo.K[7], 1e-6);
  EXPECT_NEAR(1.0, camInfo.K[8], 1e-6);

  EXPECT_NEAR(521.3358764648438, camInfo.P[0], 1e-6);
  EXPECT_NEAR(0.0, camInfo.P[1], 1e-6);
  EXPECT_NEAR(1026.724493594556, camInfo.P[2], 1e-6);
  EXPECT_NEAR(0.0, camInfo.P[3], 1e-6);
  EXPECT_NEAR(0.0, camInfo.P[4], 1e-6);
  EXPECT_NEAR(650.6087036132812, camInfo.P[5], 1e-6);
  EXPECT_NEAR(862.4682443669153, camInfo.P[6], 1e-6);
  EXPECT_NEAR(0.0, camInfo.P[7], 1e-6);
  EXPECT_NEAR(0.0, camInfo.P[8], 1e-6);
  EXPECT_NEAR(0.0, camInfo.P[9], 1e-6);
  EXPECT_NEAR(1.0, camInfo.P[10], 1e-6);
  EXPECT_NEAR(0.0, camInfo.P[11], 1e-6);

  EXPECT_NEAR(1.0, camInfo.R[0], 1e-6);
  EXPECT_NEAR(0.0, camInfo.R[1], 1e-6);
  EXPECT_NEAR(0.0, camInfo.R[2], 1e-6);
  EXPECT_NEAR(0.0, camInfo.R[3], 1e-6);
  EXPECT_NEAR(1.0, camInfo.R[4], 1e-6);
  EXPECT_NEAR(0.0, camInfo.R[5], 1e-6);
  EXPECT_NEAR(0.0, camInfo.R[6], 1e-6);
  EXPECT_NEAR(0.0, camInfo.R[7], 1e-6);
  EXPECT_NEAR(1.0, camInfo.R[8], 1e-6);

  EXPECT_EQ(sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL, camInfo.distortion_model);
  ASSERT_EQ(8, camInfo.D.size());
  EXPECT_NEAR(-0.1245457038087884, camInfo.D[0], 1e-6);
  EXPECT_NEAR(-0.05369527340945016, camInfo.D[1], 1e-6);
  EXPECT_NEAR(0.0006351464093726904, camInfo.D[2], 1e-6);
  EXPECT_NEAR(0.0002099959376929684, camInfo.D[3], 1e-6);
  EXPECT_NEAR(-0.05956909417507483, camInfo.D[4], 1e-6);
  EXPECT_NEAR(0.1941877216925693, camInfo.D[5], 1e-6);
  EXPECT_NEAR(-0.08032867890341482, camInfo.D[6], 1e-6);
  EXPECT_NEAR(-0.12207514479882, camInfo.D[7], 1e-6);

  EXPECT_EQ(0, camInfo.binning_x);
  EXPECT_EQ(0, camInfo.binning_y);
  EXPECT_EQ(0, camInfo.roi.width);
  EXPECT_EQ(0, camInfo.roi.height);
  EXPECT_EQ(0, camInfo.roi.x_offset);
  EXPECT_EQ(0, camInfo.roi.y_offset);
  EXPECT_EQ(false, camInfo.roi.do_rectify);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
