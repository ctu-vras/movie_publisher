// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for magnetometer_compass.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include <Eigen/Core>

#include <class_loader/class_loader_core.hpp>
#include <cras_cpp_common/log_utils/memory.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/nodelet_utils.hpp>
#include <cras_cpp_common/param_utils/param_helper.hpp>
#include <cras_cpp_common/string_utils/ros.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <gps_common/GPSFix.h>
#include <nodelet/nodelet.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using CompressedImage = sensor_msgs::CompressedImage;
using Image = sensor_msgs::Image;
using Fix = sensor_msgs::NavSatFix;
using FixDetail = gps_common::GPSFix;

ros::V_string my_argv;

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

template<typename NodeletType = cras::Nodelet>
std::unique_ptr<NodeletType> createNodelet(const cras::LogHelperPtr& log,
  const ros::M_string& remaps = {},
  const std::shared_ptr<tf2_ros::Buffer>& tf = nullptr)
{
  // Declaration order of these variables is important to make sure they can be properly stopped and destroyed.
  auto nodelet = class_loader::impl::createInstance<nodelet::Nodelet>(
    "movie_publisher::MoviePublisherNodelet", nullptr);
  if (nodelet == nullptr)
    return nullptr;

  {
    const auto paramHelper = dynamic_cast<cras::ParamHelper*>(nodelet);
    if (paramHelper != nullptr)
      paramHelper->setLogger(log);
  }

  const auto targetNodelet = dynamic_cast<NodeletType*>(nodelet);
  if (targetNodelet == nullptr)
  {
    delete nodelet;
    return nullptr;
  }

  if (tf != nullptr)
    targetNodelet->setBuffer(tf);

  nodelet->init(ros::this_node::getName(), remaps, my_argv, nullptr, nullptr);

  return std::unique_ptr<NodeletType>(targetNodelet);
}

TEST(MoviePublisherNodelet, FairphoneStill)  // NOLINT
{
  // The values in this test are extracted from a real-world bag file recording.

  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("movie_file", std::string(TEST_DATA_DIR) + "/fairphone/IMG_20241125_024757.jpg");
  pnh.setParam("frame_id", "test");
  pnh.setParam("wait_after_publisher_created", 0.5);
  pnh.setParam("spin_after_end", true);
  // pnh.setParam("verbose", true);

  std::vector<Image> lastImages;
  auto imageCb = [&lastImages](const Image::ConstPtr& msg)
  {
    lastImages.push_back(*msg);
  };

  std::vector<CompressedImage> lastCompressedImages;
  auto compressedImageCb = [&lastCompressedImages](const CompressedImage::ConstPtr& msg)
  {
    lastCompressedImages.push_back(*msg);
  };

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  std::list<ros::Subscriber> subs;
  auto imageSub = nh.subscribe<Image>("movie", 1, imageCb); subs.push_back(imageSub);
  auto compressedImageSub = nh.subscribe<CompressedImage>("movie/compressed", 1, compressedImageCb);
  subs.push_back(compressedImageSub);

  const auto log = std::make_shared<cras::MemoryLogHelper>();
  // const auto log = std::make_shared<cras::NodeLogHelper>();

  auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, nodelet);

  const auto subTest = [](const ros::Subscriber& p) {return p.getNumPublishers() == 0;};
  for (size_t i = 0; i < 1000 && std::any_of(subs.begin(), subs.end(), subTest); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for subscriber connections.");
  }

  ASSERT_FALSE(std::any_of(subs.begin(), subs.end(), subTest));

  for (size_t i = 0; i < 50 && (lastImages.empty() || lastCompressedImages.empty()) && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }

  ASSERT_FALSE(lastImages.empty());
  ASSERT_FALSE(lastCompressedImages.empty());

  EXPECT_EQ("test_optical_frame", lastImages[0].header.frame_id);
  EXPECT_EQ(ros::Time(1732502880, 585000000), lastImages[0].header.stamp);
  EXPECT_EQ(4000, lastImages[0].width);
  EXPECT_EQ(3000, lastImages[0].height);
  EXPECT_EQ(sensor_msgs::image_encodings::BGR8, lastImages[0].encoding);
  EXPECT_THAT(lastImages[0].step, alignedStep(4000, 3));
  EXPECT_EQ(0, lastImages[0].is_bigendian);

  EXPECT_EQ("test_optical_frame", lastCompressedImages[0].header.frame_id);
  EXPECT_EQ(ros::Time(1732502880, 585000000), lastCompressedImages[0].header.stamp);
  EXPECT_EQ("bgr8; jpeg compressed bgr8", lastCompressedImages[0].format);

  ASSERT_TRUE(tfBuffer.canTransform("test", "test_optical_frame", lastImages[0].header.stamp));

  nodelet->requestStop();
}

TEST(MoviePublisherNodelet, FairphoneMovie)  // NOLINT
{
  // The values in this test are extracted from a real-world bag file recording.

  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("movie_file", std::string(TEST_DATA_DIR) + "/fairphone/VID_20240815_143536.mp4");
  pnh.setParam("frame_id", "test");
  pnh.setParam("wait_after_publisher_created", 0.5);
  // pnh.setParam("verbose", true);

  std::vector<Image> lastImages;
  auto imageCb = [&lastImages](const Image::ConstPtr& msg)
  {
    lastImages.push_back(*msg);
  };

  std::vector<CompressedImage> lastCompressedImages;
  auto compressedImageCb = [&lastCompressedImages](const CompressedImage::ConstPtr& msg)
  {
    lastCompressedImages.push_back(*msg);
  };

  std::list<ros::Subscriber> subs;
  auto imageSub = nh.subscribe<Image>("movie", 1, imageCb); subs.push_back(imageSub);
  auto compressedImageSub = nh.subscribe<CompressedImage>("movie/compressed", 1, compressedImageCb);
  subs.push_back(compressedImageSub);

  const auto log = std::make_shared<cras::MemoryLogHelper>();
  // const auto log = std::make_shared<cras::NodeLogHelper>();

  auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, nodelet);

  const auto subTest = [](const ros::Subscriber& p) {return p.getNumPublishers() == 0;};
  for (size_t i = 0; i < 1000 && std::any_of(subs.begin(), subs.end(), subTest); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for subscriber connections.");
  }

  ASSERT_FALSE(std::any_of(subs.begin(), subs.end(), subTest));

  for (size_t i = 0; i < 50 && (lastImages.empty() || lastCompressedImages.empty()) && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }

  ASSERT_FALSE(lastImages.empty());
  ASSERT_FALSE(lastCompressedImages.empty());

  EXPECT_EQ("test_optical_frame", lastImages[0].header.frame_id);
  EXPECT_NEAR(ros::Time(1723725351, 71722222).toSec(), lastImages[0].header.stamp.toSec(), 0.2);
  EXPECT_EQ(1080, lastImages[0].width);
  EXPECT_EQ(1920, lastImages[0].height);
  EXPECT_EQ(sensor_msgs::image_encodings::BGR8, lastImages[0].encoding);
  EXPECT_THAT(lastImages[0].step, alignedStep(1080, 3));
  EXPECT_EQ(0, lastImages[0].is_bigendian);

  EXPECT_EQ("test_optical_frame", lastCompressedImages[0].header.frame_id);
  EXPECT_NEAR(ros::Time(1723725351, 71722222).toSec(), lastCompressedImages[0].header.stamp.toSec(), 0.2);
  EXPECT_EQ("bgr8; jpeg compressed bgr8", lastCompressedImages[0].format);

  for (size_t i = 0; i < 50 && \
    (lastImages.size() < 2 || lastCompressedImages.size() < 2) && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }

  ASSERT_LT(1, lastImages.size());
  ASSERT_LT(1, lastCompressedImages.size());

  EXPECT_EQ("test_optical_frame", lastImages[1].header.frame_id);
  EXPECT_NEAR(ros::Time(1723725351, 171688889).toSec(), lastImages[1].header.stamp.toSec(), 0.2);
  EXPECT_EQ(1080, lastImages[1].width);
  EXPECT_EQ(1920, lastImages[1].height);
  EXPECT_EQ(sensor_msgs::image_encodings::BGR8, lastImages[1].encoding);
  EXPECT_THAT(lastImages[0].step, alignedStep(1080, 3));
  EXPECT_EQ(0, lastImages[1].is_bigendian);

  EXPECT_EQ("test_optical_frame", lastCompressedImages[1].header.frame_id);
  EXPECT_NEAR(ros::Time(1723725351, 171688889).toSec(), lastCompressedImages[1].header.stamp.toSec(), 0.2);
  EXPECT_EQ("bgr8; jpeg compressed bgr8", lastCompressedImages[1].format);

  nodelet->requestStop();
}

TEST(MoviePublisherNodelet, IphoneMovie)  // NOLINT
{
  // The values in this test are extracted from a real-world bag file recording.

  // The iphone movie has GPS information stored so that libav can read them, so we can also test fix here.

  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("movie_file", std::string(TEST_DATA_DIR) + "/iphone/IMG_2585.MOV");
  pnh.setParam("frame_id", "test");
  pnh.setParam("wait_after_publisher_created", 0.5);
  // pnh.setParam("verbose", true);

  std::vector<Image> lastImages;
  auto imageCb = [&lastImages](const Image::ConstPtr& msg)
  {
    lastImages.push_back(*msg);
  };

  std::vector<CompressedImage> lastCompressedImages;
  auto compressedImageCb = [&lastCompressedImages](const CompressedImage::ConstPtr& msg)
  {
    lastCompressedImages.push_back(*msg);
  };

  std::vector<Fix> lastFixes;
  auto fixCb = [&lastFixes](const Fix::ConstPtr& msg)
  {
    lastFixes.push_back(*msg);
  };

  std::vector<FixDetail> lastFixDetails;
  auto fixDetailsCb = [&lastFixDetails](const FixDetail::ConstPtr& msg)
  {
    lastFixDetails.push_back(*msg);
  };

  std::list<ros::Subscriber> subs;
  auto imageSub = nh.subscribe<Image>("movie", 1, imageCb); subs.push_back(imageSub); subs.push_back(imageSub);
  auto compressedImageSub = nh.subscribe<CompressedImage>("movie/compressed", 1, compressedImageCb);
  subs.push_back(compressedImageSub);
  auto fixSub = nh.subscribe<Fix>("movie/fix", 1, fixCb); subs.push_back(fixSub);
  auto fixDetailsSub = nh.subscribe<FixDetail>("movie/fix_detail", 1, fixDetailsCb); subs.push_back(fixDetailsSub);

  const auto log = std::make_shared<cras::MemoryLogHelper>();
  // const auto log = std::make_shared<cras::NodeLogHelper>();

  auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, nodelet);

  const auto subTest = [](const ros::Subscriber& p) {return p.getNumPublishers() == 0;};
  for (size_t i = 0; i < 1000 && std::any_of(subs.begin(), subs.end(), subTest); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for subscriber connections.");
  }

  ASSERT_FALSE(std::any_of(subs.begin(), subs.end(), subTest));

  for (size_t i = 0; i < 50 && (lastImages.empty() || lastCompressedImages.empty()) && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }

  ASSERT_FALSE(lastImages.empty());
  ASSERT_FALSE(lastCompressedImages.empty());
  ASSERT_FALSE(lastFixes.empty());
  ASSERT_FALSE(lastFixDetails.empty());

  EXPECT_EQ("test_optical_frame", lastImages[0].header.frame_id);
  EXPECT_NEAR(ros::Time(1727351280, 0).toSec(), lastImages[0].header.stamp.toSec(), 0.2);
  EXPECT_EQ(1920, lastImages[0].width);
  EXPECT_EQ(1080, lastImages[0].height);
  EXPECT_EQ(sensor_msgs::image_encodings::BGR8, lastImages[0].encoding);
  EXPECT_THAT(lastImages[0].step, alignedStep(1920, 3));
  EXPECT_EQ(0, lastImages[0].is_bigendian);

  EXPECT_EQ("test_optical_frame", lastCompressedImages[0].header.frame_id);
  EXPECT_NEAR(ros::Time(1727351280, 0).toSec(), lastCompressedImages[0].header.stamp.toSec(), 0.2);
  EXPECT_EQ("bgr8; jpeg compressed bgr8", lastCompressedImages[0].format);

  auto nav = lastFixes[0];
  EXPECT_EQ("test", nav.header.frame_id);
  EXPECT_NEAR(ros::Time(1727351280, 0).toSec(), nav.header.stamp.toSec(), 0.2);
  EXPECT_NEAR(50.0799, nav.latitude, 1e-6);
  EXPECT_NEAR(14.3976, nav.longitude, 1e-6);
  EXPECT_NEAR(319.745, nav.altitude, 1e-6);
  EXPECT_EQ(0, nav.status.status);

  auto gps = lastFixDetails[0];
  EXPECT_EQ("test", gps.header.frame_id);
  EXPECT_NEAR(ros::Time(1727351280, 0).toSec(), gps.header.stamp.toSec(), 0.2);
  EXPECT_NEAR(50.0799, gps.latitude, 1e-6);
  EXPECT_NEAR(14.3976, gps.longitude, 1e-6);
  EXPECT_NEAR(319.745, gps.altitude, 1e-6);
  EXPECT_EQ(0, gps.status.status);
  EXPECT_NEAR(40.404804, gps.err_horz, 1e-6);
  EXPECT_NEAR(1632.548186, gps.position_covariance[0 * 3 + 0], 1e-6);
  EXPECT_NEAR(1632.548186, gps.position_covariance[1 * 3 + 1], 1e-6);
  EXPECT_NEAR(10000 * 10000, gps.position_covariance[2 * 3 + 2], 1e-6);
  EXPECT_EQ(gps_common::GPSFix::COVARIANCE_TYPE_APPROXIMATED, gps.position_covariance_type);

  for (size_t i = 0; i < 50 && \
    (lastImages.size() < 2 || lastCompressedImages.size() < 2) && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }

  ASSERT_LT(1, lastImages.size());
  ASSERT_LT(1, lastCompressedImages.size());

  EXPECT_EQ("test_optical_frame", lastImages[1].header.frame_id);
  EXPECT_NEAR(ros::Time(1727351280, 66666667).toSec(), lastImages[1].header.stamp.toSec(), 0.2);
  EXPECT_EQ(1920, lastImages[1].width);
  EXPECT_EQ(1080, lastImages[1].height);
  EXPECT_EQ(sensor_msgs::image_encodings::BGR8, lastImages[1].encoding);
  EXPECT_THAT(lastImages[0].step, alignedStep(1920, 3));
  EXPECT_EQ(0, lastImages[1].is_bigendian);

  EXPECT_EQ("test_optical_frame", lastCompressedImages[1].header.frame_id);
  EXPECT_NEAR(ros::Time(1727351280, 66666667).toSec(), lastCompressedImages[1].header.stamp.toSec(), 0.2);
  EXPECT_EQ("bgr8; jpeg compressed bgr8", lastCompressedImages[1].format);

  nodelet->requestStop();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  // Remove the program name from argv because the nodelet handling code does not expect it
  argc -= 1;
  argv += 1;
  ros::removeROSArgs(argc, argv, my_argv);
  ros::init(argc, argv, "test_movie_publisher_nodelet");

  ros::NodeHandle nh;  // Just prevent ROS being uninited when the test-private nodehandles go out of scope

  return RUN_ALL_TESTS();
}
