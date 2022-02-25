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

#ifndef OVERLAY_HPP_
#define OVERLAY_HPP_

#include <memory>
#include <string>
#include <queue>
#include <mutex>
#include <map>
#include "pluginlib/class_loader.hpp"

// Forward Declaration
namespace rclcpp
{
class Node;
class GenericSubscription;
class SerializedMessage;
class Time;
}
class QImage;
namespace rqt_image_overlay_layer {class PluginInterface;}
namespace rqt_image_overlay {class OverlayRequest;}

namespace rqt_image_overlay
{

class Overlay
{
public:
  Overlay(
    std::string pluginClass,
    pluginlib::ClassLoader<rqt_image_overlay_layer::PluginInterface> & pluginLoader,
    const std::shared_ptr<rclcpp::Node> & node,
    unsigned msgHistoryLength = 100);
  void setTopic(std::string topic);
  void overlay(QImage & image, const OverlayRequest & overlayRequest);
  void setEnabled(bool enabled);

  std::string getTopic() const;
  std::string getPluginClass() const;
  std::string getMsgType() const;
  std::string getReceivedStatus() const;
  bool isEnabled() const;

private:
  const std::string pluginClass;
  const std::shared_ptr<rqt_image_overlay_layer::PluginInterface> instance;
  const std::string msgType;
  std::string topic;
  bool enabled = true;
  std::shared_ptr<rclcpp::GenericSubscription> subscription;
  const std::shared_ptr<rclcpp::Node> & node;
  std::shared_ptr<rclcpp::Time> timeLastMsgReceived;

  const unsigned msgHistoryLength;
  mutable std::mutex msgHistoryMutex;

  // msgMap and msgTimeQueue two together, create a FIFO Map, as described in
  // https://stackoverflow.com/a/21315813
  std::map<const rclcpp::Time, std::shared_ptr<rclcpp::SerializedMessage>> msgMap;
  std::queue<rclcpp::Time> msgTimeQueue;

  bool useHeaderTimestamp;
  std::shared_ptr<rclcpp::SerializedMessage> lastMsg; // used if useHeaderTimestamp == false

  bool firstMsgReceived = false;

  void msgCallback(std::shared_ptr<rclcpp::SerializedMessage> msg);

  int callbackCounter = 0;
};

}  // namespace rqt_image_overlay

#endif  // OVERLAY_HPP_
