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

#ifndef IMAGE_MANAGER_HPP_
#define IMAGE_MANAGER_HPP_

#include <optional>
#include <QAbstractListModel>
#include <QImage>
#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <map>
#include "image_transport/subscriber.hpp"
#include "image_topic.hpp"

namespace rclcpp {class Node;}
namespace rqt_image_overlay {class OverlayRequest;}

namespace rqt_image_overlay
{

class ImageManager : public QAbstractListModel
{
  Q_OBJECT

public:
  explicit ImageManager(const std::shared_ptr<rclcpp::Node> & node, unsigned msgHistoryLength = 50);
  std::shared_ptr<QImage> getImage(const rclcpp::Time & exactTime) const;
  std::optional<ImageTopic> getImageTopic(unsigned index);
  void addImageTopicExplicitly(ImageTopic imageTopic);
  std::optional<OverlayRequest> createOverlayRequest(const rclcpp::Time & targetTime) const;

protected:
  int rowCount(const QModelIndex & parent = QModelIndex()) const override;
  QVariant data(const QModelIndex & index, int role = Qt::DisplayRole) const override;

public slots:
  void onTopicChanged(int index);
  void updateImageTopicList();

private:
  void callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  image_transport::Subscriber subscriber;
  const std::shared_ptr<rclcpp::Node> & node;

  std::vector<ImageTopic> imageTopics;

  const unsigned msgHistoryLength;
  mutable std::mutex msgHistoryMutex;

  // msgMap and msgTimeQueue two together, create a FIFO Map, as described in
  // https://stackoverflow.com/a/21315813
  std::map<const rclcpp::Time, const sensor_msgs::msg::Image::ConstSharedPtr> msgMap;
  std::queue<rclcpp::Time> msgTimeQueue;

  rclcpp::Clock systemClock{RCL_SYSTEM_TIME};
};

}  // namespace rqt_image_overlay

#endif  // IMAGE_MANAGER_HPP_
