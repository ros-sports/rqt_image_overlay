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

#ifndef IMAGE_OVERLAY__PLUGIN_HPP_
#define IMAGE_OVERLAY__PLUGIN_HPP_

#include <memory>
#include <string>
#include "image_overlay/image_overlay_plugin.hpp"
#include "rclcpp/create_generic_subscription.hpp"
#include "rclcpp/node.hpp"

class Plugin
{
public:
  Plugin(
    std::string pluginClass, std::shared_ptr<ImageOverlayPlugin>,
    const rclcpp::Node::SharedPtr & node);
  void setTopic(std::string topic);
  void overlay(QImage & image);
  void setEnabled(bool enabled);

  std::string getTopic() const;
  std::string getPluginClass() const;
  std::string getMsgType() const;
  bool getEnabled() const;

private:
  const std::string pluginClass;
  const std::shared_ptr<ImageOverlayPlugin> instance;
  const std::string msgType;
  std::string topic;
  bool enabled = true;
  rclcpp::GenericSubscription::SharedPtr subscription;
  const rclcpp::Node::SharedPtr & node_;
  std::shared_ptr<rclcpp::SerializedMessage> lastMsg;

  void msgCallback(std::shared_ptr<rclcpp::SerializedMessage> msg);
  std::string guessTopic();
};

#endif  // IMAGE_OVERLAY__PLUGIN_HPP_
