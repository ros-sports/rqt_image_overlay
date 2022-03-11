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

#ifndef RQT_IMAGE_OVERLAY_LAYER__PLUGIN_INTERFACE_HPP_
#define RQT_IMAGE_OVERLAY_LAYER__PLUGIN_INTERFACE_HPP_

#include <QPainter>
#include <string>
#include <memory>
#include "rclcpp/time.hpp"

// forward declaration
namespace rclcpp {class SerializedMessage;}

namespace rqt_image_overlay_layer
{

class PluginInterface
{
public:
  virtual std::string getTopicType() const = 0;
  virtual void overlay(
    QPainter & painter,
    const std::shared_ptr<rclcpp::SerializedMessage> & msg) = 0;
  virtual bool hasMsgHeader() const = 0;

  // Throws std::runtime_error if called on msg type with no header
  virtual rclcpp::Time getHeaderTime(
    const std::shared_ptr<rclcpp::SerializedMessage> & msg) const = 0;

  virtual ~PluginInterface() {}

protected:
  PluginInterface() {}
};

}  // namespace rqt_image_overlay_layer

#endif  // RQT_IMAGE_OVERLAY_LAYER__PLUGIN_INTERFACE_HPP_
