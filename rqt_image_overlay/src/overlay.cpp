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
#include "rqt_image_overlay/overlay.hpp"
#include "rclcpp/create_generic_subscription.hpp"
#include "rclcpp/node.hpp"
#include "rqt_image_overlay/image_overlay_plugin.hpp"

namespace rqt_image_overlay
{

Overlay::Overlay(
  std::string pluginClass, pluginlib::ClassLoader<ImageOverlayPlugin> & pluginLoader,
  const std::shared_ptr<rclcpp::Node> & node)
: pluginClass(pluginClass), instance(pluginLoader.createSharedInstance(pluginClass)),
  msgType(instance->getTopicType()), node(node)
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
    } catch (const std::exception & e) {
      std::cerr << "Failed to create subscription: " << e.what() << '\n';
    }
  }
}

void Overlay::overlay(QImage & image)
{
  // Create a new shared_ptr, since lastMsg may change if a new message arrives.
  const std::shared_ptr<rclcpp::SerializedMessage> lastMsgCopy(std::atomic_load(&lastMsg));

  if (lastMsgCopy) {
    instance->overlay(image, lastMsgCopy);
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

void Overlay::setEnabled(bool enabled)
{
  this->enabled = enabled;
}

bool Overlay::getEnabled() const
{
  return enabled;
}

void Overlay::msgCallback(std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  std::atomic_store(&lastMsg, msg);
}

}  // namespace rqt_image_overlay
