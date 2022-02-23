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

std::vector<ImageTopic> ListImageTopics(const rclcpp::Node & node)
{
  std::map<std::string, std::vector<std::string>> topicInfo = node.get_topic_names_and_types();

  std::vector<ImageTopic> imageTopics;
  for (auto const & [topicName, topicTypes] : topicInfo) {
    if (std::count(topicTypes.begin(), topicTypes.end(), "sensor_msgs/msg/Image") > 0) {
      imageTopics.push_back({topicName, "raw"});
    }
  }
  return imageTopics;
}

}  // namespace rqt_image_overlay
