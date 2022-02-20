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


// void ImageManager::onTopicChanged(const QString & text)
void ImageManager::onTopicChanged(int index)
{
  subscriber.shutdown();

  // reset image on topic change
  std::atomic_store(&lastMsg, sensor_msgs::msg::Image::ConstSharedPtr{});

  if (index > 0) {
    ImageTopic & imageTopic = topics.at(index - 1);
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
  topics.clear();

  // fill combo box
  topics = ListImageTopics(*node);

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
      const ImageTopic & imageTopic = topics.at(index.row() - 1);
      return QString::fromStdString(imageTopic.topic) + " (" + QString::fromStdString(
        imageTopic.transport) + ")";
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

void ImageManager::setImageTopicExplicitly(ImageTopic topic)
{
  beginResetModel();
  topics.clear();
  topics.push_back(topic);
  endResetModel();
}

void ImageManager::saveSettings(qt_gui_cpp::Settings & settings) const
{
  // int currentItem = ui->image_topics_combo_box->currentData();
  // if (currentItem != 0) {
  //   const ImageTopic & imageTopic = topics.at(currentItem - 1);
  //   settings.setValue("image_topic", imageTopic.topic);
  //   settings.setValue("image_transport", imageTopic.transport);
  // }
}

void ImageManager::restoreSettings(const qt_gui_cpp::Settings & settings)
{
  // if (settings.contains("image_topic")) {
  //   QString topic = settings.value("image_topic").toString();
  //   if (topic != "") {
  //     QString transport = settings.value("image_transport").toString();
  //     imageManager->setTopicExplicitly(ImageTopic{topic, transport});
  //     ui->image_topics_combo_box->setCurrentIndex(1);
  //   }
  // }
}

}  // namespace rqt_image_overlay
