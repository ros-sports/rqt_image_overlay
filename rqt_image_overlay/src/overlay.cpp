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

#include <string>
#include <memory>
#include <map>
#include <vector>
#include "overlay.hpp"
#include "rclcpp/create_generic_subscription.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/time.hpp"
#include "rqt_image_overlay_layer/plugin_interface.hpp"
#include "image_manager.hpp"

namespace rqt_image_overlay
{

Overlay::Overlay(
  std::string pluginClass,
  pluginlib::ClassLoader<rqt_image_overlay_layer::PluginInterface> & pluginLoader,
  const std::shared_ptr<rclcpp::Node> & node, unsigned msgHistoryLength)
: pluginClass(pluginClass), instance(pluginLoader.createSharedInstance(pluginClass)),
  msgType(instance->getTopicType()), node(node), msgHistoryLength(msgHistoryLength)
{
}

void Overlay::setTopic(std::string topic)
{
  if (topic != "") {
    try {
      subscription = node->create_generic_subscription(
        topic, msgType, rclcpp::QoS(10),
        std::bind(&Overlay::msgCallback, this, std::placeholders::_1));
      this->topic = topic;

      // reset history variables
      {
        std::lock_guard<std::mutex> guard(msgHistoryMutex);
        msgMap.clear();
        msgTimeQueue = {};
        lastMsg.reset();
      }
    } catch (const std::exception & e) {
      qWarning("(Overlay) Failed to change subscription topic: %s", e.what());
      rcutils_reset_error();
    }
  }
}

void Overlay::overlay(QImage & image, const rclcpp::Time & targetTime)
{
  if (!firstMsgReceived) {
    return;
  }

  std::shared_ptr<rclcpp::SerializedMessage> msgToDraw;

  {
    std::lock_guard<std::mutex> guard(msgHistoryMutex);
    if (useHeaderTimestamp) {
      try {
        msgToDraw = msgMap.at(targetTime);
      } catch (std::out_of_range & e) {
        // Message not received for the targetTime, ignore.
      }
    } else {
      msgToDraw = lastMsg;
    }
  }

  if (msgToDraw) {
    instance->overlay(image, msgToDraw);
  }
}

std::string Overlay::getTopic() const
{
  return topic;
}

std::string Overlay::getPluginClass() const
{
  return pluginClass;
}

std::string Overlay::getMsgType() const
{
  return msgType;
}

std::string Overlay::getReceivedStatus() const
{
  std::shared_ptr<rclcpp::Time> timeLastMsgReceivedCopy(std::atomic_load(&timeLastMsgReceived));
  if (timeLastMsgReceivedCopy) {
    rclcpp::Duration diff = node->now() - *timeLastMsgReceivedCopy;
    char msg[50];
    snprintf(msg, sizeof(msg), "%.4fs ago", diff.nanoseconds() / 1000000000.0);
    return msg;
  } else {
    return "Not received yet";
  }
}

void Overlay::setEnabled(bool enabled)
{
  this->enabled = enabled;
}

bool Overlay::isEnabled() const
{
  return enabled;
}

void Overlay::msgCallback(std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  // On first msg, check if the time can be deduced from the msg
  if (!firstMsgReceived) {
    firstMsgReceived = true;
    useHeaderTimestamp = instance->getTime(msg).has_value();
  }

  {
    std::lock_guard<std::mutex> guard(msgHistoryMutex);

    // Delete old messages because we don't need them anymore
    if (msgMap.size() > msgHistoryLength) {
      msgMap.erase(msgTimeQueue.front());
      msgTimeQueue.pop();
    }

    if (useHeaderTimestamp) {
      auto time = instance->getTime(msg).value();

      // Must convert the time from RCL_SYSTEM_TIME to RCL_ROS_TIME. Remove the line below when
      // changes sugggested in https://github.com/ros2/message_filters/issues/32 get merged.
      time = rclcpp::Time{time.nanoseconds(), RCL_ROS_TIME};

      msgMap.insert(make_pair(time, msg));
      msgTimeQueue.push(time);
    } else {
      lastMsg = msg;
    }
  }

  std::atomic_store(&timeLastMsgReceived, std::make_shared<rclcpp::Time>(node->now()));
}

}  // namespace rqt_image_overlay
