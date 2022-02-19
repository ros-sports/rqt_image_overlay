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

ImageManager::ImageManager(const std::shared_ptr<rclcpp::Node> & node, unsigned maxDequeSize)
: node(node), maxDequeSize(maxDequeSize)
{
}

void ImageManager::callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> guard(dequeMutex);

  // Delete old messages because we don't need them anymore
  if (msgDeque.size() > maxDequeSize)
  {
    msgDeque.pop_front();
  }

  msgDeque.push_back(msg);
}


void ImageManager::onTopicChanged(int index)
{
  subscriber.shutdown();

  // reset deque
  {
    std::lock_guard<std::mutex> guard(dequeMutex);
    msgDeque.clear();
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

std::shared_ptr<QImage> ImageManager::getImage(const rclcpp::Time & time) const
{
  std::shared_ptr<QImage> image;

  sensor_msgs::msg::Image::ConstSharedPtr imageToShow;

  {
    std::lock_guard<std::mutex> guard(dequeMutex);

    if (!msgDeque.empty()) {
      int64_t timeNs = static_cast<int64_t>(time.nanoseconds());

      int64_t maxDiff = std::numeric_limits<int64_t>::max();
      imageToShow = msgDeque.front();
      for (auto const & image : msgDeque) {
        int64_t imageNs = static_cast<int64_t>(rclcpp::Time{image->header.stamp}.nanoseconds());
        if (int64_t diff = std::abs(imageNs - timeNs); diff < maxDiff) {
          maxDiff = diff;
          imageToShow = image;
        } else {
          break;
        }
      }
    }
  }

  if (imageToShow) {
    image = std::make_shared<QImage>(ros_image_to_qimage::Convert(*imageToShow));
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
  std::lock_guard<std::mutex> guard(dequeMutex);
  if (!msgDeque.empty()) {
    return std::optional<rclcpp::Time>{msgDeque.back()->header.stamp};
  }
  return std::nullopt;
}

}  // namespace rqt_image_overlay
