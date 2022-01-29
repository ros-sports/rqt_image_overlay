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
#include "image_manager.hpp"
#include "list_image_topics.hpp"
#include "image_transport/image_transport.hpp"
#include "ros_image_to_qimage/ros_image_to_qimage.hpp"

namespace rqt_image_overlay
{

ImageManager::ImageManager(const std::shared_ptr<rclcpp::Node> & node)
: node(node)
{
}

void ImageManager::callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  std::atomic_store(&lastMsg, msg);
}


void ImageManager::onTopicChanged(const QString & text)
{
  subscriber.shutdown();

  // reset image on topic change
  std::atomic_store(&lastMsg, sensor_msgs::msg::Image::ConstSharedPtr{});

  QStringList parts = text.split(" ");
  QString topic = parts.first();
  QString transport = parts.length() == 2 ? parts.last() : "raw";

  if (!topic.isEmpty()) {
    image_transport::ImageTransport it(node);
    const image_transport::TransportHints hints(node.get(), transport.toStdString());
    try {
      subscriber =
        it.subscribe(topic.toStdString(), 1, &ImageManager::callbackImage, this, &hints);
      qDebug(
        "ImageView::onTopicChanged() to topic '%s' with transport '%s'",
        topic.toStdString().c_str(), subscriber.getTransport().c_str());
    } catch (image_transport::TransportLoadException & e) {
      qWarning("(ImageManager) Loading image transport plugin failed");
    }
  }
}

void ImageManager::updateImageTopicList()
{
  beginResetModel();
  topics.clear();

  // fill combo box
  topics = ListImageTopics(*node);

  for (std::string & topic : topics) {
    std::replace(topic.begin(), topic.end(), ' ', '/');
  }
  endResetModel();
}


int ImageManager::rowCount(const QModelIndex &) const
{
  return topics.size() + 1;
}

QVariant ImageManager::data(const QModelIndex & index, int role) const
{
  if (role == Qt::DisplayRole) {
    if (index.row() == 0) {
      return QVariant();
    } else {
      std::string topic = topics.at(index.row() - 1);
      return QString::fromStdString(topic);
    }
  }

  return QVariant();
}

std::shared_ptr<QImage> ImageManager::getImage() const
{
  std::shared_ptr<QImage> image;

  // Create a new shared_ptr, since lastMsg may change if a new message arrives.
  const sensor_msgs::msg::Image::ConstSharedPtr lastMsgCopy(std::atomic_load(&lastMsg));
  if (lastMsgCopy) {
    image = std::make_shared<QImage>(ros_image_to_qimage::Convert(*lastMsgCopy));
  }
  return image;
}

void ImageManager::setTopicExplicitly(QString topic)
{
  beginResetModel();
  topics.clear();
  topics.push_back(topic.toStdString());
  endResetModel();
}

}  // namespace rqt_image_overlay
