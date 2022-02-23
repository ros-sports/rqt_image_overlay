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
#include "image_transport/image_transport.hpp"

namespace rqt_image_overlay
{

std::vector<ImageTopic> ListImageTopics(rclcpp::Node & node)
{
  // Get declared transports. getDeclaredTransports returns a vector of transports prefixed with
  // 'image_transport/', but we strip it because when we create the subscription
  // (ie. image_transport::create_subscription), we don't want the 'image_transport/'.
  //  - image_transport/compressed
  //  - image_transport/raw
  std::vector<std::string> declaredTransports;
  for (auto & transport : image_transport::getDeclaredTransports()) {
    const std::string prefix = "image_transport/";
    if (transport.substr(0, prefix.size()) == prefix) {
      declaredTransports.push_back(transport.substr(prefix.size()));
    }
  }

  std::vector<ImageTopic> imageTopicCandidates;
  for (auto const & [topicName, topicTypes] : node.get_topic_names_and_types()) {
    for (auto & topicType : topicTypes) {
      if (std::count(topicTypes.begin(), topicTypes.end(), topicType) > 0) {
        if (topicType == "sensor_msgs/msg/Image") {
          // Raw image transport
          imageTopicCandidates.push_back({topicName, "raw"});
        } else {
          // Some image transport other than raw is being used.
          // Assume transport name is after the last forward slash, due to image_trasport
          // conventions.
          // For example, a topic such as "/foo/bar/compressed" will be decomposed into:
          //   topic - "/foo/bar"
          //   transport - "compressed"
          const auto lastSlashIndex = topicName.find_last_of("/");
          std::string topic = topicName.substr(0, lastSlashIndex);
          std::string transport = topicName.substr(lastSlashIndex + 1);

          // If the transport is in the list of declared transports
          if (std::count(declaredTransports.begin(), declaredTransports.end(), transport) > 0) {
            imageTopicCandidates.push_back({topic, transport});
          }
        }
      }
    }
  }

  std::vector<ImageTopic> imageTopics;
  // Check if a subscription can be made successfully, with more than one valid publisher
  for (auto & imageTopic : imageTopicCandidates) {
    try {
      auto subscriber = image_transport::create_subscription(
        &node, imageTopic.topic, image_transport::Subscriber::Callback{}, imageTopic.transport);

      // Check if the image subscriber managed to successfully connect to any publishers
      // This considers cases where the msg type of publisher and subscriber don't match.
      if (subscriber.getNumPublishers() > 0) {
        imageTopics.push_back(imageTopic);
      }
    } catch (image_transport::TransportLoadException & e) {
      // Wasn't a valid transport. Don't add to list
    } catch (rclcpp::exceptions::RCLError & e) {
      // Msg type of the subscription didn't match the one being published by an existing
      // publisher. Don't add to list.
    }
  }

  return imageTopics;
}

}  // namespace rqt_image_overlay
