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
#include "image_overlay/plugin.hpp"

Plugin::Plugin(
  std::string pluginClass, std::shared_ptr<ImageOverlayPlugin> instance,
  const rclcpp::Node::SharedPtr & node)
: pluginClass(pluginClass), instance(instance), msgType(instance->getTopicType()), node_(node)
{
  setTopic(guessTopic());
}

void Plugin::setTopic(std::string topic)
{
  this->topic = topic;

  if (topic != "") {
    subscription = node_->create_generic_subscription(
      topic, msgType, rclcpp::QoS(10),
      std::bind(&Plugin::msgCallback, this, std::placeholders::_1));
  }
}

void Plugin::overlay(QImage & image)
{
  // Create a new shared_ptr, since lastMsg may change if a new message arrives.
  const std::shared_ptr<rclcpp::SerializedMessage> lastMsgCopy(std::atomic_load(&lastMsg));

  if (lastMsgCopy) {
    instance->overlay(image, lastMsgCopy);
  }
}

std::string Plugin::getTopic() const
{
  return topic;
}

std::string Plugin::getPluginClass() const
{
  return pluginClass;
}

std::string Plugin::getMsgType() const
{
  return msgType;
}

void Plugin::setEnabled(bool enabled)
{
  this->enabled = enabled;
}

bool Plugin::getEnabled() const
{
  return enabled;
}

void Plugin::msgCallback(std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  std::atomic_store(&lastMsg, msg);
}

std::string Plugin::guessTopic()
{
  // try and automatically detect topic name from plugin type
  std::map<std::string, std::vector<std::string>> topic_info =
    node_->get_topic_names_and_types();
  for (auto const & [topic_name, topic_types] : topic_info) {
    if (topic_types.at(0) == instance->getTopicType()) {
      return topic_name;
    }
  }

  return "";
}
