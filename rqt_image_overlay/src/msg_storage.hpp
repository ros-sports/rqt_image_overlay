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
#include <mutex>
#include <utility>
#include "rclcpp/time.hpp"
#include "exceptions.hpp"

namespace rqt_image_overlay
{

template<typename T>
class MsgStorage
{
public:
  explicit MsgStorage(unsigned capacity = 50)
  : capacity(capacity) {}

  // throws StorageEmptyException if storage is empty
  // throws std::runtime_error("can't compare times with different time sources") if ros time source
  //        doesn't match
  rclcpp::Time getClosestTime(const rclcpp::Time & targetTime) const
  {
    if (empty()) {
      throw StorageEmptyException();
    }

    std::lock_guard<std::mutex> guard(mutex);

    rclcpp::Time closestTimeReceived;
    rclcpp::Duration minDiff = rclcpp::Duration::max();
    for (const auto &[timeReceived, msg] : msgMap) {
      rclcpp::Duration diff =
        (timeReceived < targetTime) ? (targetTime - timeReceived) : (timeReceived - targetTime);
      if (diff < minDiff) {
        minDiff = diff;
        closestTimeReceived = timeReceived;
      } else {
        // A cpp map is sorted, so we won't find anything closer. Break out!
        break;
      }
    }
    return closestTimeReceived;
  }

  // throws std::out_of_range if doesn't exist
  T getMsg(const rclcpp::Time & time) const
  {
    std::lock_guard<std::mutex> guard(mutex);
    return msgMap.at(time);
  }

  bool empty() const
  {
    std::lock_guard<std::mutex> guard(mutex);
    return msgTimeDeque.empty();
  }

  void store(const rclcpp::Time & time, const T & msg)
  {
    std::lock_guard<std::mutex> guard(mutex);

    // Ensure size msgMap is less than capacity
    if (msgMap.size() == capacity) {
      msgMap.erase(msgTimeDeque.front());
      msgTimeDeque.pop_front();
    }

    msgMap.insert(make_pair(time, msg));
    msgTimeDeque.push_back(time);
  }

  void clear()
  {
    std::lock_guard<std::mutex> guard(mutex);
    msgMap.clear();
    msgTimeDeque.clear();
  }

private:
  mutable std::mutex mutex;

  // msgMap and msgTimeDeque two together, create a FIFO Map, as described in
  // https://stackoverflow.com/a/21315813
  std::map<const rclcpp::Time, const T> msgMap;
  std::deque<rclcpp::Time> msgTimeDeque;

  const unsigned capacity;
};

}  // namespace rqt_image_overlay

#endif  // MSG_STORAGE_HPP_
