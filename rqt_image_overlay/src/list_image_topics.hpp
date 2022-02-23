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

#ifndef LIST_IMAGE_TOPICS_HPP_
#define LIST_IMAGE_TOPICS_HPP_

#include <vector>
#include "image_topic.hpp"

// forward declaration
namespace rclcpp {class Node;}

namespace rqt_image_overlay
{

std::vector<ImageTopic> ListImageTopics(rclcpp::Node & node);

}  // namespace rqt_image_overlay

#endif  // LIST_IMAGE_TOPICS_HPP_
