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
#include <memory>
#include "./list_image_topics.hpp"
#include "image_transport/image_transport.hpp"

namespace rqt_image_overlay
{

std::vector<std::string> ListImageTopics(const rclcpp::Node & node)
{
  std::vector<std::string> message_types{
    "sensor_msgs/Image",
    "sensor_msgs/msg/Image"};
  std::vector<std::string> message_sub_types{
    "sensor_msgs/CompressedImage",
    "sensor_msgs/msg/CompressedImage"};

  // get declared transports
  std::vector<std::string> transports;
  std::vector<std::string> declared = image_transport::getDeclaredTransports();

  for (std::string transport : declared) {
    // strip prefix from transport name
    std::string prefix = "image_transport/";

    size_t index = transport.find(prefix);
    if (index == 0) {
      transport = transport.substr(prefix.size());
    }

    transports.push_back(transport);
  }

  // Get topics
  std::map<std::string, std::vector<std::string>> topic_info = node.get_topic_names_and_types();

  std::vector<std::string> all_topics;
  for (auto const & [key, _] : topic_info) {
    all_topics.push_back(key);
  }

  std::vector<std::string> topics;
  for (auto const & [topic_name, topic_types] : topic_info) {
    for (const std::string & topic_type : topic_types) {
      if (std::count(message_types.begin(), message_types.end(), topic_type) > 0) {
        // add raw topic
        topics.push_back(topic_name);

        // add transport specific sub-topics
        for (const std::string & transport : transports) {
          if (std::count(all_topics.begin(), all_topics.end(), topic_name + "/" + transport) > 0) {
            std::string sub = topic_name + " " + transport;
            topics.push_back(sub);
          }
        }
      }
      if (std::count(message_sub_types.begin(), message_sub_types.end(), topic_type) > 0) {
        size_t index = topic_name.find_last_of("/");
        if (index != std::string::npos) {
          std::string topic_name_copy = topic_name;
          topic_name_copy.replace(index, 1, " ");
          topics.push_back(topic_name_copy);
        }
      }
    }
  }
  return topics;
}

}  // namespace rqt_image_overlay
