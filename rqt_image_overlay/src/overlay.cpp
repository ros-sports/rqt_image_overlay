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
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rqt_image_overlay_layer/plugin_interface.hpp"
#include "overlay_time_info.hpp"

namespace rqt_image_overlay
{

Overlay::Overlay(
  std::string pluginClass,
  pluginlib::ClassLoader<rqt_image_overlay_layer::PluginInterface> & pluginLoader,
  const std::shared_ptr<rclcpp::Node> & node)
: pluginClass(pluginClass), instance(pluginLoader.createSharedInstance(pluginClass)),
  msgType(instance->getTopicType()), node(node), useHeaderTimestamp(instance->hasMsgHeader())
{
}

void Overlay::setTopic(std::string topic)
{
  if (topic != "") {
    try {
      subscription = node->create_generic_subscription(
        topic, msgType, rclcpp::SensorDataQoS(),
        std::bind(&Overlay::msgCallback, this, std::placeholders::_1));
      this->topic = topic;
      msgStorage.clear();
    } catch (const std::exception & e) {
      qWarning("(Overlay) Failed to change subscription topic: %s", e.what());
      rcutils_reset_error();
    }
  }
}

void Overlay::overlay(QPainter & painter, const OverlayTimeInfo & overlayTimeInfo) const
{
  if (msgStorage.empty()) {
    return;
  }

  std::shared_ptr<rclcpp::SerializedMessage> msg;
  if (useHeaderTimestamp) {
    try {
      msg = msgStorage.getMsg(overlayTimeInfo.timeFromHeader);
    } catch (const std::out_of_range & e) {
      // Matching time didn't exist in the msgStorage, continue
    }
  } else {
    auto closestTime = msgStorage.getClosestTime(overlayTimeInfo.timeReceived);
    msg = msgStorage.getMsg(closestTime);
  }

  if (msg) {
    instance->overlay(painter, msg);
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
  return msgStorage.empty() ? "Not received" : "Received";
}

QColor Overlay::getColor() const
{
  return color;
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
  rclcpp::Time time = useHeaderTimestamp ? instance->getHeaderTime(msg) : systemClock.now();
  msgStorage.store(time, msg);
}

void Overlay::setColor(QColor color)
{
  this->color = color;
}

}  // namespace rqt_image_overlay
