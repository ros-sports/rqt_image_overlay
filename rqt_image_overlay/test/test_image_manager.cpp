// Copyright 2022 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "../src/image_manager.hpp"

class TestImageManager : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestImageManager, imageAvailableFalse)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  rqt_image_overlay::ImageManager imageManager{node};
  EXPECT_FALSE(imageManager.imageAvailable());
}

TEST_F(TestImageManager, imageAvailableTrue)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  rqt_image_overlay::ImageManager imageManager{node};

  // Create subscription on "test_topic"
  imageManager.addImageTopicExplicitly(rqt_image_overlay::ImageTopic{"test_topic", "raw"});
  imageManager.onTopicChanged(1);

  // Publish on "test_topic"
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>("test_topic", 1);
  publisher->publish(sensor_msgs::msg::Image{});

  // Give a chance for the topic to be picked up
  rclcpp::sleep_for(std::chrono::milliseconds(10));
  rclcpp::spin_some(node);

  EXPECT_TRUE(imageManager.imageAvailable());
}

TEST_F(TestImageManager, getClosestImageAndHeaderTime)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  rqt_image_overlay::ImageManager imageManager{node};
  EXPECT_THROW(
    imageManager.getClosestImageAndHeaderTime(rclcpp::Time{}),
    rqt_image_overlay::StorageEmptyException);
}

TEST_F(TestImageManager, getHeaderTime)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  rqt_image_overlay::ImageManager imageManager{node};

  // Create subscription on "test_topic"
  imageManager.addImageTopicExplicitly(rqt_image_overlay::ImageTopic{"test_topic", "raw"});
  imageManager.onTopicChanged(1);

  // Publish on "test_topic"
  rclcpp::Time time{1, 0, RCL_ROS_TIME};
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>("test_topic", 1);
  auto msg = sensor_msgs::msg::Image{};
  msg.header.stamp = time;
  msg.encoding = sensor_msgs::image_encodings::RGB8;
  publisher->publish(msg);

  // Give a chance for the topic to be picked up
  rclcpp::sleep_for(std::chrono::milliseconds(10));
  rclcpp::spin_some(node);

  ASSERT_TRUE(imageManager.imageAvailable());
  auto [image, headerTime] =
    imageManager.getClosestImageAndHeaderTime(rclcpp::Clock().now());
  EXPECT_EQ(headerTime, time);
  EXPECT_NE(image, nullptr);
}

TEST_F(TestImageManager, TestGetImageTopic)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  rqt_image_overlay::ImageManager imageManager{node};

  auto imageTopic1 = rqt_image_overlay::ImageTopic{"topic_1", "raw"};
  imageManager.addImageTopicExplicitly(imageTopic1);
  auto imageTopic2 = rqt_image_overlay::ImageTopic{"topic_2", "raw"};
  imageManager.addImageTopicExplicitly(imageTopic2);

  EXPECT_EQ(imageManager.getImageTopic(0), std::nullopt);
  EXPECT_EQ(imageManager.getImageTopic(1).value(), imageTopic1);
  EXPECT_EQ(imageManager.getImageTopic(2).value(), imageTopic2);
}
