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

#ifndef IMAGE_OVERLAY__PLUGIN_HELPER_HPP_
#define IMAGE_OVERLAY__PLUGIN_HELPER_HPP_

#include <memory>
#include <string>
#include "rclcpp/typesupport_helpers.hpp"
#include "rclcpp/serialization.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

template<typename T>
class PluginHelper
{
public:
  T deserialize(std::shared_ptr<rclcpp::SerializedMessage> msg)
  {
    auto library = rclcpp::get_typesupport_library(
      rosidl_generator_traits::name<T>(), "rosidl_typesupport_cpp");
    auto string_typesupport = rclcpp::get_typesupport_handle(
      rosidl_generator_traits::name<T>(), "rosidl_typesupport_cpp", *library);
    rclcpp::SerializationBase base(string_typesupport);

    T des;
    base.deserialize_message(msg.get(), &des);
    return des;
  }

  std::string getTopicType()
  {
    return rosidl_generator_traits::name<T>();
  }
};

#endif  // IMAGE_OVERLAY__PLUGIN_HELPER_HPP_
