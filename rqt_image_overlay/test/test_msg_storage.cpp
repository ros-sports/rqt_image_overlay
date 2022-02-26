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

  auto ret = storage.getMsg(rclcpp::Time{});
  EXPECT_FALSE(ret.has_value());
}

TEST(TestMsgStorage, GetMsgNoValueWrongTime)
{
  rqt_image_overlay::MsgStorage<std::string> storage;
  std::string msg = "hello world";
  storage.store(rclcpp::Time{1, 0}, msg);

  auto ret = storage.getMsg(rclcpp::Time{2, 0});
  EXPECT_FALSE(ret.has_value());
}

TEST(TestMsgStorage, GetMsg)
{
  rqt_image_overlay::MsgStorage<std::string> storage;
  auto time = rclcpp::Time{1, 0};
  std::string msg = "hello world";
  storage.store(time, msg);

  auto ret = storage.getMsg(time);
  ASSERT_TRUE(ret.has_value());
  EXPECT_EQ(ret.value(), msg);
}
