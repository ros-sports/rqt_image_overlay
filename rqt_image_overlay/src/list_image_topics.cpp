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

#include <vector>
#include <map>
#include "list_image_topics.hpp"
#include "rclcpp/node.hpp"

namespace rqt_image_overlay
{

std::vector<ImageTopic> ListImageTopics(const rclcpp::Node & node)
{
  std::map<std::string, std::vector<std::string>> topic_info = node.get_topic_names_and_types();

  std::vector<ImageTopic> topics;

  std::vector<std::string> types = {"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"};

  for (auto const & [topic_name, topic_types] : topic_info) {

    for (auto & type : types) {
      if (std::count(topic_types.begin(), topic_types.end(), type) > 0) {
        if (type == "sensor_msgs/msg/Image") {
          // Raw image transport
          topics.push_back({topic_name, "raw"});
        } else {
          // Some image transport other than raw is being used.
          // Assume transport name is after the last forward slash, due to image_trasport
          // conventions.
          // For example, a topic such as "/foo/bar/compressed" will be decomposed into
          // topic = "/foo/bar"
          // transport = "compressed"
          const auto lastSlashIndex = topic_name.find_last_of("/");
          std::string topic = topic_name.substr(0, lastSlashIndex);
          std::string transport = topic_name.substr(lastSlashIndex + 1);
          topics.push_back({topic, transport});
        }
      }
    }
  }
  return topics;
}

}  // namespace rqt_image_overlay
