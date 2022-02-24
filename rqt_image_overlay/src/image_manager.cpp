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
#include <memory>
#include <limits>
#include "image_manager.hpp"
#include "list_image_topics.hpp"
#include "image_transport/image_transport.hpp"
#include "ros_image_to_qimage/ros_image_to_qimage.hpp"

namespace rqt_image_overlay
{

ImageManager::ImageManager(const std::shared_ptr<rclcpp::Node> & node, unsigned msgHistoryLength)
: node(node), msgHistoryLength(msgHistoryLength)
{
}

void ImageManager::callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> guard(msgHistoryMutex);

  // Delete old messages because we don't need them anymore
  if (msgMap.size() > msgHistoryLength) {
    msgMap.erase(msgTimeQueue.front());
    msgTimeQueue.pop();
  }

  rclcpp::Time time = rclcpp::Time{msg->header.stamp};
  msgMap.insert(make_pair(time, msg));
  msgTimeQueue.push(time);
}


void ImageManager::onTopicChanged(int index)
{
  subscriber.shutdown();

  // reset queue and map
  {
    std::lock_guard<std::mutex> guard(msgHistoryMutex);
    msgMap.clear();
    msgTimeQueue = {};
  }

  if (index > 0) {
    ImageTopic & imageTopic = imageTopics.at(index - 1);
    try {
      subscriber = image_transport::create_subscription(
        node.get(), imageTopic.topic,
        std::bind(&ImageManager::callbackImage, this, std::placeholders::_1),
        imageTopic.transport, rmw_qos_profile_sensor_data);
      qDebug(
        "ImageView::onTopicChanged() to topic '%s' with transport '%s'",
        imageTopic.topic.c_str(), imageTopic.transport.c_str());
    } catch (image_transport::TransportLoadException & e) {
      qWarning("(ImageManager) Loading image transport plugin failed");
    }
  }
}

void ImageManager::updateImageTopicList()
{
  beginResetModel();
  imageTopics.clear();

  // fill combo box
  imageTopics = ListImageTopics(*node);

  // if there are no publishers on the subscribed topic, delete the subscription
  if (subscriber.getNumPublishers() == 0) {
    subscriber.shutdown();
    subscriber = image_transport::Subscriber{};
  }

  endResetModel();
}


int ImageManager::rowCount(const QModelIndex &) const
{
  return imageTopics.size() + 1;
}

QVariant ImageManager::data(const QModelIndex & index, int role) const
{
  if (role == Qt::DisplayRole) {
    if (index.row() == 0) {
      return QVariant();
    } else {
      const ImageTopic & imageTopic = imageTopics.at(index.row() - 1);
      return QString::fromStdString(imageTopic.toString());
    }
  }

  return QVariant();
}

std::shared_ptr<QImage> ImageManager::getImage(const rclcpp::Time & exactTime) const
{
  std::lock_guard<std::mutex> guard(msgHistoryMutex);

  std::shared_ptr<QImage> image;

  try {
    image = std::make_shared<QImage>(ros_image_to_qimage::Convert(*msgMap.at(exactTime)));
  } catch (std::out_of_range &) {
    // Image at requested time doesn't exist. Shouldn't reach here.
  }

  return image;
}

std::optional<ImageTopic> ImageManager::getImageTopic(unsigned index)
{
  if (index > 0 && index < imageTopics.size()) {
    const ImageTopic & it = imageTopics.at(index);
    return std::make_optional<ImageTopic>(it);
  }
  return std::nullopt;
}

void ImageManager::addImageTopicExplicitly(ImageTopic imageTopic)
{
  beginResetModel();
  imageTopics.clear();
  imageTopics.push_back(imageTopic);
  endResetModel();
}

std::optional<rclcpp::Time> ImageManager::getLatestImageTime() const
{
  std::lock_guard<std::mutex> guard(msgHistoryMutex);
  if (!msgTimeQueue.empty()) {
    return std::optional<rclcpp::Time>{msgTimeQueue.back()};
  }
  return std::nullopt;
}

std::optional<rclcpp::Time> ImageManager::getClosestExactImageTime(
  const rclcpp::Time & targetTime) const
{
  {
    std::lock_guard<std::mutex> guard(msgHistoryMutex);

    if (!msgMap.empty()) {
      int64_t targetTimeNs = static_cast<int64_t>(targetTime.nanoseconds());

      rclcpp::Time closestImageTime;

      int64_t minDiff = std::numeric_limits<int64_t>::max();
      for (const auto &[msgTime, image]: msgMap) {
        int64_t msgTimeNs = static_cast<int64_t>(msgTime.nanoseconds());
        if (int64_t diff = std::abs(msgTimeNs - targetTimeNs); diff < minDiff) {
          minDiff = diff;
          closestImageTime = msgTime;
        } else {
          // A cpp map is sorted, so we won't find anything closer. Break out!
          break;
        }
      }
      return std::make_optional<rclcpp::Time>(closestImageTime);
    } else {
      return std::nullopt;
    }
  }
}

}  // namespace rqt_image_overlay
