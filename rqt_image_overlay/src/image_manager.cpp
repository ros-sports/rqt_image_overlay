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
#include <tuple>
#include "./ui_depth_image_display_options_dialog.h"
#include "cv_bridge/cv_bridge.h"
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
  msgStorage.store(systemClock.now(), msg);
}


void ImageManager::onTopicChanged(int index)
{
  subscriber.shutdown();

  // reset image on topic change
  msgStorage.clear();

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

void ImageManager::updateDepthImageDisplayOptions()
{
  QDialog * dialog = new QDialog();
  auto ui = std::make_unique<Ui::DepthImageDisplayOptionsDialog>();
  ui->setupUi(dialog);

  // Place current settings into Dialog
  const std::shared_ptr<cv_bridge::CvtColorForDisplayOptions> options(
    std::atomic_load(&depthImageDisplayOptions));
  if (options) {
    ui->dynamic_scaling_check_box->setChecked(options->do_dynamic_scaling);
    ui->minimum_value_spin_box->setValue(options->min_image_value);
    ui->maximum_value_spin_box->setValue(options->max_image_value);
    ui->colormap_spin_box->setValue(options->colormap);
    ui->background_label_spin_box->setValue(options->bg_label);
  }

  if (dialog->exec() == QDialog::Accepted) {
    std::shared_ptr<cv_bridge::CvtColorForDisplayOptions> newOptions;
    newOptions->do_dynamic_scaling = ui->dynamic_scaling_check_box->isChecked();
    newOptions->min_image_value = ui->minimum_value_spin_box->value();
    newOptions->max_image_value = ui->maximum_value_spin_box->value();
    newOptions->colormap = ui->colormap_spin_box->value();
    newOptions->bg_label = ui->background_label_spin_box->value();
    std::atomic_store(&depthImageDisplayOptions, newOptions);
  }
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

std::tuple<std::shared_ptr<QImage>, rclcpp::Time> ImageManager::getClosestImageAndHeaderTime(
  const rclcpp::Time & targetTimeReceived) const
{
  auto closestTime = msgStorage.getClosestTime(targetTimeReceived);
  auto msg = msgStorage.getMsg(closestTime);
  auto image = std::make_shared<QImage>(
    ros_image_to_qimage::Convert(*msg), *std::atomic_load(&depthImageDisplayOptions));
  return std::make_tuple(image, rclcpp::Time{msg->header.stamp});
}

bool ImageManager::imageAvailable() const
{
  return !msgStorage.empty();
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

}  // namespace rqt_image_overlay
