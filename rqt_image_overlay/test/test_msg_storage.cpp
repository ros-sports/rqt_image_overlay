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
#include <string>
#include "../src/msg_storage.hpp"

TEST(TestMsgStorage, Empty)
{
  rqt_image_overlay::MsgStorage<std::string> storage;
  EXPECT_TRUE(storage.empty());
}

TEST(TestMsgStorage, NotEmpty)
{
  rqt_image_overlay::MsgStorage<std::string> storage;
  storage.store(rclcpp::Time{}, "");

  EXPECT_FALSE(storage.empty());
}

TEST(TestMsgStorage, GetMsgNoValue)
{
  rqt_image_overlay::MsgStorage<std::string> storage;

  EXPECT_THROW(storage.getMsg(rclcpp::Time{}), std::out_of_range);
}

TEST(TestMsgStorage, GetMsgNoValueWrongTime)
{
  rqt_image_overlay::MsgStorage<std::string> storage;
  std::string msg = "hello world";
  storage.store(rclcpp::Time{1, 0}, msg);

  EXPECT_THROW(storage.getMsg(rclcpp::Time{2, 0}), std::out_of_range);
}

TEST(TestMsgStorage, GetMsg)
{
  rqt_image_overlay::MsgStorage<std::string> storage;
  auto time = rclcpp::Time{1, 0};
  std::string msg = "hello world";
  storage.store(time, msg);

  ASSERT_NO_THROW(
  {
    auto ret = storage.getMsg(time);
    EXPECT_EQ(ret, msg);
  });
}

TEST(TestMsgStorage, GetClosestTimeException)
{
  rqt_image_overlay::MsgStorage<std::string> storage;

  EXPECT_THROW(storage.getClosestTime(rclcpp::Time{}), rqt_image_overlay::StorageEmptyException);
}

TEST(TestMsgStorage, GetClosestTimeExceptionTimeSource)
{
  rqt_image_overlay::MsgStorage<std::string> storage;
  storage.store(rclcpp::Time{1, 0, RCL_SYSTEM_TIME}, "");

  ASSERT_THROW(storage.getClosestTime(rclcpp::Time{0, 0, RCL_ROS_TIME}), std::runtime_error);
}

TEST(TestMsgStorage, GetClosestTime)
{
  rqt_image_overlay::MsgStorage<std::string> storage;
  storage.store(rclcpp::Time{1, 0}, "");
  storage.store(rclcpp::Time{5, 0}, "");
  storage.store(rclcpp::Time{10, 0}, "");

  ASSERT_NO_THROW(
  {
    EXPECT_EQ(storage.getClosestTime(rclcpp::Time{0, 0}), rclcpp::Time(1, 0));
    EXPECT_EQ(storage.getClosestTime(rclcpp::Time{4, 0}), rclcpp::Time(5, 0));
    EXPECT_EQ(storage.getClosestTime(rclcpp::Time{7, 0}), rclcpp::Time(5, 0));
    EXPECT_EQ(storage.getClosestTime(rclcpp::Time{100, 0}), rclcpp::Time(10, 0));
  });
}

TEST(TestMsgStorage, TestClear)
{
  rqt_image_overlay::MsgStorage<std::string> storage;
  storage.store(rclcpp::Time{}, "");
  EXPECT_FALSE(storage.empty());

  storage.clear();
  EXPECT_TRUE(storage.empty());
}

TEST(TestMsgStorage, TestCapacity)
{
  rqt_image_overlay::MsgStorage<std::string> storage(3);
  storage.store(rclcpp::Time{1, 0}, "");
  storage.store(rclcpp::Time{2, 0}, "");
  storage.store(rclcpp::Time{3, 0}, "");

  EXPECT_NO_THROW(storage.getMsg(rclcpp::Time{1, 0}));

  storage.store(rclcpp::Time{4, 0}, "");
  EXPECT_THROW(storage.getMsg(rclcpp::Time{1, 0}), std::out_of_range);
}
