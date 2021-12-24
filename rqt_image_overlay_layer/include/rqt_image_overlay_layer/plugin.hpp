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

#ifndef RQT_IMAGE_OVERLAY_LAYER__PLUGIN_HPP_
#define RQT_IMAGE_OVERLAY_LAYER__PLUGIN_HPP_

#include <string>
#include <memory>
#include "rqt_image_overlay_layer/plugin_interface.hpp"
#include "rclcpp/typesupport_helpers.hpp"
#include "rclcpp/serialization.hpp"
#include "rosidl_runtime_cpp/traits.hpp"
#include "rcpputils/asserts.hpp"

namespace rqt_image_overlay_layer
{

template<typename T>
class Plugin : public PluginInterface
{
public:
  std::string getTopicType() const override
  {
    return rosidl_generator_traits::name<T>();
  }

  void overlay(
    QImage & layer,
    const std::shared_ptr<rclcpp::SerializedMessage> & msg) override
  {
    try {
      overlay(layer, deserialize(msg));
    } catch (const rcpputils::IllegalStateException & ex) {
      // ignore exception
    }
  }

  virtual ~Plugin() {}

protected:
  Plugin()
  : library(rclcpp::get_typesupport_library(rosidl_generator_traits::name<T>(),
      "rosidl_typesupport_cpp")),
    stringTypesupport(rclcpp::get_typesupport_handle(rosidl_generator_traits::name<T>(),
      "rosidl_typesupport_cpp", *library)),
    base(stringTypesupport)
  {
  }

  virtual void overlay(
    QImage & layer,
    const T & msg) = 0;

private:
  T deserialize(const std::shared_ptr<rclcpp::SerializedMessage> & msg)
  {
    T des;
    base.deserialize_message(msg.get(), &des);
    return des;
  }

  const std::shared_ptr<rcpputils::SharedLibrary> library;
  const rosidl_message_type_support_t * stringTypesupport;
  const rclcpp::SerializationBase base;
};

}  // namespace rqt_image_overlay_layer

#endif  // RQT_IMAGE_OVERLAY_LAYER__PLUGIN_HPP_
