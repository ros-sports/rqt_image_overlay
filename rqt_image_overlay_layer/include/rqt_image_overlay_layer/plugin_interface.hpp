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

#include <QImage>
#include <string>
#include <memory>

// forward declaration
namespace rclcpp {class SerializedMessage;}

namespace rqt_image_overlay_layer
{

class PluginInterface
{
public:
  virtual std::string getTopicType() const = 0;
  virtual void overlay(
    QImage & layer,
    const std::shared_ptr<rclcpp::SerializedMessage> & msg) = 0;
  virtual ~PluginInterface() {}

protected:
  PluginInterface() {}
};

}  // namespace rqt_image_overlay_layer

#endif  // RQT_IMAGE_OVERLAY_LAYER__PLUGIN_INTERFACE_HPP_
