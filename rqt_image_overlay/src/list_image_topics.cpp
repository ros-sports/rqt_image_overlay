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
#include <vector>
#include <map>
#include "list_image_topics.hpp"
#include "rclcpp/node.hpp"

namespace rqt_image_overlay
{

std::vector<std::string> ListImageTopics(const rclcpp::Node & node)
{
  std::map<std::string, std::vector<std::string>> topic_info = node.get_topic_names_and_types();

  std::vector<std::string> topics;
  for (auto const & [topic_name, topic_types] : topic_info) {
    if (std::count(topic_types.begin(), topic_types.end(), "sensor_msgs/msg/Image") > 0) {
      topics.push_back(topic_name);
    }
    if (std::count(
      topic_types.begin(), topic_types.end(), "sensor_msgs/msg/CompressedImage") > 0) {
      const auto index = topic_name.find_last_of("/");
      if(index != std::string::npos) {
        auto compressed_topic_name = topic_name;
        compressed_topic_name.replace(index, 1, " ");
        topics.push_back(compressed_topic_name);
      }
    }
  }
  return topics;
}

}  // namespace rqt_image_overlay
