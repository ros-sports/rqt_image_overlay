// Copyright 2022 Daisuke Nishimatsu
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
#include "try_discover_source.hpp"

namespace rqt_image_overlay
{
std::optional<std::pair<std::string, rclcpp::QoS>> tryDiscoverSource(
  const std::shared_ptr<rclcpp::Node> & node, const std::string & topic)
{
  // borrowed this from domain bridge
  // (https://github.com/ros2/domain_bridge/blob/main/src/domain_bridge/wait_for_graph_events.hpp)
  // Query QoS info for publishers
  std::vector<rclcpp::TopicEndpointInfo> endpoint_info_vec =
    node->get_publishers_info_by_topic(topic);
  std::size_t num_endpoints = endpoint_info_vec.size();

  // If there are no publishers, return an empty optional
  if (num_endpoints < 1u) {
    return {};
  }

  // Initialize QoS
  rclcpp::QoS qos{10};
  // Default reliability and durability to value of first endpoint
  qos.reliability(endpoint_info_vec[0].qos_profile().reliability());
  qos.durability(endpoint_info_vec[0].qos_profile().durability());
  // Always use automatic liveliness
  qos.liveliness(rclcpp::LivelinessPolicy::Automatic);

  // Reliability and durability policies can cause trouble with enpoint matching
  // Count number of "reliable" publishers and number of "transient local" publishers
  std::size_t reliable_count = 0u;
  std::size_t transient_local_count = 0u;
  // For duration-based policies, note the largest value to ensure matching all publishers
  rclcpp::Duration max_deadline(0, 0u);
  rclcpp::Duration max_lifespan(0, 0u);
  for (const auto & info : endpoint_info_vec) {
    const auto & profile = info.qos_profile();
    if (profile.reliability() == rclcpp::ReliabilityPolicy::Reliable) {
      reliable_count++;
    }
    if (profile.durability() == rclcpp::DurabilityPolicy::TransientLocal) {
      transient_local_count++;
    }
    if (profile.deadline() > max_deadline) {
      max_deadline = profile.deadline();
    }
    if (profile.lifespan() > max_lifespan) {
      max_lifespan = profile.lifespan();
    }
  }

  // If not all publishers have a "reliable" policy, then use a "best effort" policy
  // and print a warning
  if (reliable_count > 0u && reliable_count != num_endpoints) {
    qos.best_effort();
    RCLCPP_WARN(
      node->get_logger(), "Some, but not all, publishers on topic %s "
      "offer 'reliable' reliability. Falling back to 'best effort' reliability in order"
      "to connect to all publishers.", topic.c_str());
  }

  // If not all publishers have a "transient local" policy, then use a "volatile" policy
  // and print a warning
  if (transient_local_count > 0u && transient_local_count != num_endpoints) {
    qos.durability_volatile();
    RCLCPP_WARN(
      node->get_logger(), "Some, but not all, publishers on topic %s "
      "offer 'transient local' durability. Falling back to 'volatile' durability in order"
      "to connect to all publishers.", topic.c_str());
  }

  qos.deadline(max_deadline);
  qos.lifespan(max_lifespan);

  if (!endpoint_info_vec.empty()) {
    return std::make_pair(endpoint_info_vec[0].topic_type(), qos);
  } else {
    return {};
  }
}
}  // namespace rqt_image_overlay
