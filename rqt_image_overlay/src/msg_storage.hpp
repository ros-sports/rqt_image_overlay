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

#ifndef MSG_STORAGE_HPP_
#define MSG_STORAGE_HPP_

#include <map>
#include <deque>
#include <utility>
#include "rclcpp/time.hpp"

namespace rqt_image_overlay
{

template<typename T>
class MsgStorage
{
public:
  // std::optional<rclcpp::Time> getClosestTime() const
  // {
  //   return std::nullopt;
  // }

  std::optional<T> getMsg(const rclcpp::Time & time) const
  {
    try {
      return std::make_optional<T>(msgMap.at(time));
    } catch (std::out_of_range &) {
      return std::nullopt;
    }
  }

  bool empty() const
  {
    return msgTimeDeque.empty();
  }

  void store(const rclcpp::Time & time, const T & msg)
  {
    msgMap.insert(make_pair(time, msg));
    msgTimeDeque.push_back(time);
  }

private:
  // msgMap and msgTimeDeque two together, create a FIFO Map, as described in
  // https://stackoverflow.com/a/21315813
  std::map<const rclcpp::Time, const T> msgMap;
  std::deque<rclcpp::Time> msgTimeDeque;
};

}  // namespace rqt_image_overlay

#endif  // MSG_STORAGE_HPP_
