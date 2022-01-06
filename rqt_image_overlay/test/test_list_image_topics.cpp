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
#include "../src/list_image_topics.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"

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
};

TEST_F(TestListImageTopics, TestNone)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");

  // Give a chance for the topic to be picked up
  rclcpp::sleep_for(std::chrono::milliseconds(10));
  rclcpp::spin_some(node);

  auto topics = rqt_image_overlay::ListImageTopics(*node);
  EXPECT_EQ(topics.size(), 0u);
}

TEST_F(TestListImageTopics, TestOne)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");

  auto publisher = image_transport::create_publisher(node.get(), "test_topic");

  // Give a chance for the topic to be picked up
  rclcpp::sleep_for(std::chrono::milliseconds(10));
  rclcpp::spin_some(node);

  auto topics = rqt_image_overlay::ListImageTopics(*node);
  ASSERT_EQ(topics.size(), 1u);
  EXPECT_EQ(topics.at(0), "/test_topic");
}

TEST_F(TestListImageTopics, TestThree)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");

  auto publisher1 = image_transport::create_publisher(node.get(), "test_ns1/test_topic1");
  auto publisher2 = image_transport::create_publisher(node.get(), "test_ns2/test_topic2");
  auto publisher3 = image_transport::create_publisher(node.get(), "test_ns3/test_topic3");

  // Give a chance for the topic to be picked up
  rclcpp::sleep_for(std::chrono::milliseconds(10));
  rclcpp::spin_some(node);

  auto topics = rqt_image_overlay::ListImageTopics(*node);
  ASSERT_EQ(topics.size(), 3u);
  EXPECT_EQ(std::count(topics.begin(), topics.end(), "/test_ns1/test_topic1"), 1);
  EXPECT_EQ(std::count(topics.begin(), topics.end(), "/test_ns2/test_topic2"), 1);
  EXPECT_EQ(std::count(topics.begin(), topics.end(), "/test_ns3/test_topic3"), 1);
}
