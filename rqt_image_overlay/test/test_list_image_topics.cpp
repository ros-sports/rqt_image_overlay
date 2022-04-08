// Copyright 2021 Kenji Brameld
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
#include <vector>
#include "../src/list_image_topics.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/string.hpp"

// In this test, there's two types of transports (ie. "raw" and "compressed") available.
// The "compressed" transport is declared as a test dependency so we can write tests for
// image_transport plugins.
class TestListImageTopics : public ::testing::Test
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

  static bool checkContains(
    const std::vector<rqt_image_overlay::ImageTopic> & imageTopics,
    const rqt_image_overlay::ImageTopic & imageTopic)
  {
    return std::count(imageTopics.begin(), imageTopics.end(), imageTopic) > 0;
  }
};

TEST_F(TestListImageTopics, TestNone)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");

  // Give a chance for the topic to be picked up
  rclcpp::sleep_for(std::chrono::milliseconds(10));
  rclcpp::spin_some(node);

  auto imageTopics = rqt_image_overlay::ListImageTopics(*node);
  EXPECT_EQ(imageTopics.size(), 0u);
}

TEST_F(TestListImageTopics, TestOne)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");

  auto publisher = image_transport::create_publisher(node.get(), "/test_topic");

  // Give a chance for the topic to be picked up
  rclcpp::sleep_for(std::chrono::milliseconds(10));
  rclcpp::spin_some(node);

  auto imageTopics = rqt_image_overlay::ListImageTopics(*node);
  ASSERT_EQ(imageTopics.size(), 2u);
  EXPECT_TRUE(checkContains(imageTopics, rqt_image_overlay::ImageTopic{"/test_topic", "raw"}));
  EXPECT_TRUE(
    checkContains(imageTopics, rqt_image_overlay::ImageTopic{"/test_topic", "compressed"}));
}

TEST_F(TestListImageTopics, TestThree)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");

  auto publisher1 = image_transport::create_publisher(node.get(), "/test_ns1/test_topic1");
  auto publisher2 = image_transport::create_publisher(node.get(), "/test_ns2/test_topic2");
  auto publisher3 = image_transport::create_publisher(node.get(), "/test_ns3/test_topic3");

  // Give a chance for the topic to be picked up
  rclcpp::sleep_for(std::chrono::milliseconds(10));
  rclcpp::spin_some(node);

  auto imageTopics = rqt_image_overlay::ListImageTopics(*node);
  ASSERT_EQ(imageTopics.size(), 6u);
  EXPECT_TRUE(
    checkContains(
      imageTopics,
      rqt_image_overlay::ImageTopic{"/test_ns1/test_topic1", "raw"}));
  EXPECT_TRUE(
    checkContains(
      imageTopics,
      rqt_image_overlay::ImageTopic{"/test_ns1/test_topic1", "compressed"}));
  EXPECT_TRUE(
    checkContains(
      imageTopics,
      rqt_image_overlay::ImageTopic{"/test_ns2/test_topic2", "raw"}));
  EXPECT_TRUE(
    checkContains(
      imageTopics,
      rqt_image_overlay::ImageTopic{"/test_ns2/test_topic2", "compressed"}));
  EXPECT_TRUE(
    checkContains(
      imageTopics,
      rqt_image_overlay::ImageTopic{"/test_ns3/test_topic3", "raw"}));
  EXPECT_TRUE(
    checkContains(
      imageTopics,
      rqt_image_overlay::ImageTopic{"/test_ns3/test_topic3", "compressed"}));
}

TEST_F(TestListImageTopics, TestOnlyCompressedTransport)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");

  auto publisher =
    node->create_publisher<sensor_msgs::msg::CompressedImage>("/test_topic/compressed", 1);

  // Give a chance for the topic to be picked up
  rclcpp::sleep_for(std::chrono::milliseconds(10));
  rclcpp::spin_some(node);

  auto imageTopics = rqt_image_overlay::ListImageTopics(*node);
  ASSERT_EQ(imageTopics.size(), 1u);
  EXPECT_TRUE(
    checkContains(
      imageTopics,
      rqt_image_overlay::ImageTopic{"/test_topic", "compressed"}));
}

TEST_F(TestListImageTopics, TestFakeCompressedTransport)
{
  // In this example, we test a case where the topic name ends with a transport type, but the msg
  // type is not sensor_msgs::msg::CompressedImage.
  auto node = std::make_shared<rclcpp::Node>("test_node");

  auto publisher =
    node->create_publisher<std_msgs::msg::String>("/test_topic/compressed", 1);

  // Give a chance for the topic to be picked up
  rclcpp::sleep_for(std::chrono::milliseconds(10));
  rclcpp::spin_some(node);

  auto topics = rqt_image_overlay::ListImageTopics(*node);
  ASSERT_TRUE(topics.empty());
}
