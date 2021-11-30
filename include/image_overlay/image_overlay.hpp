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

#ifndef IMAGE_OVERLAY__IMAGE_OVERLAY_HPP_
#define IMAGE_OVERLAY__IMAGE_OVERLAY_HPP_

#include <QWidget>
#include <string>
#include <vector>
#include <memory>
#include "rqt_gui_cpp/plugin.h"
#include "./ui_image_overlay.h"
#include "image_transport/subscriber.hpp"
#include "opencv2/core/core.hpp"
#include "image_overlay/image_overlay_plugin.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/create_generic_subscription.hpp"


class ImageOverlay : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  ImageOverlay();
  virtual void initPlugin(qt_gui_cpp::PluginContext & context);

protected:
  virtual QSet<QString> getTopics(
    const QSet<QString> & message_types,
    const QSet<QString> & message_sub_types,
    const QList<QString> & transports);

  virtual void selectTopic(const QString & topic);
  virtual void callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  image_transport::Subscriber subscriber_;
  cv::Mat conversion_mat_;

protected slots:
  virtual void updateImageTopicList();
  virtual void addOverlay(QString plugin_class);
  virtual void onTopicChanged(int index);
  virtual void updatePluginInstances(QTableWidgetItem * table_widget_item);

private:
  void fillOverlayMenu();

  QWidget * widget_;
  Ui::ImageOverlay ui_;

  // Empty layer blueprint that gets initialized upon a new image topic selection
  std::shared_ptr<QImage> layer_blueprint_;

  pluginlib::ClassLoader<ImageOverlayPlugin> image_overlay_plugin_loader;
  std::vector<std::string> image_overlay_plugin_classes;

  std::vector<std::shared_ptr<ImageOverlayPlugin>> plugin_instances;

  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions;
};

#endif  // IMAGE_OVERLAY__IMAGE_OVERLAY_HPP_
