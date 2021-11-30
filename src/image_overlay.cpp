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

#include <QMessageBox>
#include <QMenu>
#include <QSignalMapper>
#include <map>
#include <algorithm>
#include <vector>
#include <string>
#include <memory>
#include <iostream>
#include <utility>
#include "image_overlay/image_overlay.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"

#define COLUMN_TOPIC 0
#define COLUMN_TYPE 1
#define COLUMN_PLUGIN 2
#define COLUMN_RATE 3

ImageOverlay::ImageOverlay()
: rqt_gui_cpp::Plugin(), image_overlay_plugin_loader("image_overlay", "ImageOverlayPlugin"),
  image_overlay_plugin_classes(image_overlay_plugin_loader.getDeclaredClasses())
{
}

void ImageOverlay::initPlugin(qt_gui_cpp::PluginContext & context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  context.addWidget(widget_);

  updateImageTopicList();
  ui_.image_topics_combo_box->setCurrentIndex(ui_.image_topics_combo_box->findText(""));
  connect(
    ui_.image_topics_combo_box, SIGNAL(currentIndexChanged(int)), this,
    SLOT(onTopicChanged(int)));

  connect(ui_.refresh_image_topics_button, SIGNAL(pressed()), this, SLOT(updateImageTopicList()));
  connect(
    ui_.plugin_topic_table, SIGNAL(itemChanged(QTableWidgetItem*)), this,
    SLOT(updatePluginInstances(QTableWidgetItem*)));

  fillOverlayMenu();
}

void ImageOverlay::updateImageTopicList()
{
  QSet<QString> message_types;
  message_types.insert("sensor_msgs/Image");
  message_types.insert("sensor_msgs/msg/Image");
  QSet<QString> message_sub_types;
  message_sub_types.insert("sensor_msgs/CompressedImage");
  message_sub_types.insert("sensor_msgs/msg/CompressedImage");

  // get declared transports
  QList<QString> transports;
  image_transport::ImageTransport it(node_);
  std::vector<std::string> declared = it.getDeclaredTransports();
  for (std::vector<std::string>::const_iterator it = declared.begin(); it != declared.end(); it++) {
    // qDebug("ImageView::updateImageTopicList() declared transport '%s'", it->c_str());
    QString transport = it->c_str();

    // strip prefix from transport name
    QString prefix = "image_transport/";
    if (transport.startsWith(prefix)) {
      transport = transport.mid(prefix.length());
    }
    transports.append(transport);
  }

  QString selected = ui_.image_topics_combo_box->currentText();

  // fill combo box
  QList<QString> topics = getTopics(message_types, message_sub_types, transports).values();
  topics.append("");
  std::sort(topics.begin(), topics.end());
  ui_.image_topics_combo_box->clear();
  for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it++) {
    QString label(*it);
    label.replace(" ", "/");
    ui_.image_topics_combo_box->addItem(label, QVariant(*it));
  }

  // restore previous selection
  selectTopic(selected);
}

void ImageOverlay::addOverlay(QString plugin_class)
{
  std::cout << "addOverlay called with plugin_class: " << plugin_class.toStdString() << std::endl;

  int row = ui_.plugin_topic_table->rowCount();
  ui_.plugin_topic_table->insertRow(row);
  QTableWidgetItem * plugin_class_name_item = new QTableWidgetItem(plugin_class);
  plugin_class_name_item->setFlags(plugin_class_name_item->flags() ^ Qt::ItemIsEditable);
  ui_.plugin_topic_table->setItem(row, COLUMN_PLUGIN, plugin_class_name_item);

  // Sample code for instantiating the plugin
  std::shared_ptr<ImageOverlayPlugin> plugin_instance =
    image_overlay_plugin_loader.createSharedInstance(
    plugin_class.toStdString());
  plugin_instances.push_back(plugin_instance);

  // Code for filling in type column
  QTableWidgetItem * type_widget =
    new QTableWidgetItem(QString::fromStdString(plugin_instance->getTopicType()));
  type_widget->setFlags(type_widget->flags() ^ Qt::ItemIsEditable);
  ui_.plugin_topic_table->setItem(row, COLUMN_TYPE, type_widget);

  // Sample code to automatically detect topic name, from plugin type
  std::map<std::string, std::vector<std::string>> topic_info =
    node_->get_topic_names_and_types();
  for (auto const & [topic_name, topic_types] : topic_info) {
    if (topic_types.at(0) == plugin_instance->getTopicType()) {
      QTableWidgetItem * topic_name_item = new QTableWidgetItem(QString::fromStdString(topic_name));
      ui_.plugin_topic_table->setItem(row, COLUMN_TOPIC, topic_name_item);
      break;
    }
  }
}

void ImageOverlay::onTopicChanged(int index)
{
  subscriber_.shutdown();

  // reset image on topic change
  ui_.image_frame->setImage(QImage());

  // Reset blueprint layer since image size may have changed
  layer_blueprint_ = nullptr;

  QStringList parts = ui_.image_topics_combo_box->itemData(index).toString().split(" ");
  QString topic = parts.first();
  QString transport = parts.length() == 2 ? parts.last() : "raw";

  if (!topic.isEmpty()) {
    image_transport::ImageTransport it(node_);
    const image_transport::TransportHints hints(node_.get(), transport.toStdString());
    try {
      subscriber_ =
        it.subscribe(topic.toStdString(), 1, &ImageOverlay::callbackImage, this, &hints);
      qDebug(
        "ImageView::onTopicChanged() to topic '%s' with transport '%s'",
        topic.toStdString().c_str(), subscriber_.getTransport().c_str());
    } catch (image_transport::TransportLoadException & e) {
      QMessageBox::warning(widget_, tr("Loading image transport plugin failed"), e.what());
    }
  }
}

QSet<QString> ImageOverlay::getTopics(
  const QSet<QString> & message_types,
  const QSet<QString> & message_sub_types,
  const QList<QString> & transports)
{
  std::map<std::string, std::vector<std::string>> topic_info = node_->get_topic_names_and_types();

  QSet<QString> all_topics;
  for (std::map<std::string, std::vector<std::string>>::iterator it = topic_info.begin();
    it != topic_info.end(); ++it)
  {
    all_topics.insert(it->first.c_str());
  }

  QSet<QString> topics;
  for (std::map<std::string, std::vector<std::string>>::iterator it = topic_info.begin();
    it != topic_info.end(); ++it)
  {
    for (std::vector<std::string>::const_iterator msg_type_it = it->second.begin();
      msg_type_it != it->second.end(); ++msg_type_it)
    {
      if (message_types.contains(msg_type_it->c_str())) {
        QString topic = it->first.c_str();

        // add raw topic
        topics.insert(topic);
        // qDebug("ImageOverlay::getTopics() raw topic '%s'", topic.toStdString().c_str());

        // add transport specific sub-topics
        for (QList<QString>::const_iterator jt = transports.begin(); jt != transports.end(); jt++) {
          if (all_topics.contains(topic + "/" + *jt)) {
            QString sub = topic + " " + *jt;
            topics.insert(sub);
            // qDebug(
            //   "ImageOverlay::getTopics() transport specific sub-topic '%s'",
            //   sub.toStdString().c_str());
          }
        }
      }
      if (message_sub_types.contains(msg_type_it->c_str())) {
        QString topic = it->first.c_str();
        int index = topic.lastIndexOf("/");
        if (index != -1) {
          topic.replace(index, 1, " ");
          topics.insert(topic);
          // qDebug(
          //   "ImageOverlay::getTopics() transport specific sub-topic '%s'",
          //   topic.toStdString().c_str());
        }
      }
    }
  }
  return topics;
}

void ImageOverlay::selectTopic(const QString & topic)
{
  int index = ui_.image_topics_combo_box->findText(topic);
  if (index == -1) {
    // add topic name to list if not yet in
    QString label(topic);
    label.replace(" ", "/");
    ui_.image_topics_combo_box->addItem(label, QVariant(topic));
    index = ui_.image_topics_combo_box->findText(topic);
  }
  ui_.image_topics_combo_box->setCurrentIndex(index);
}

void ImageOverlay::callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  try {
    // Convert image from ros to cv type
    cv_bridge::CvImageConstPtr cv_ptr =
      cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    conversion_mat_ = cv_ptr->image;
  } catch (cv_bridge::Exception & e) {
    qWarning(
      "ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an "
      "exception was thrown (%s)",
      msg->encoding.c_str(), e.what());
    ui_.image_frame->setImage(QImage());
    return;
  }

  // image must be copied since it uses the conversion_mat_ for storage which is asynchronously
  // overwritten in the next callback invocation
  QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows,
    conversion_mat_.step[0], QImage::Format_RGB888);
  ui_.image_frame->setImage(image);

  if (!layer_blueprint_) {
    layer_blueprint_ = std::make_shared<QImage>(
      image.width(), image.height(), QImage::Format_ARGB32_Premultiplied);
    layer_blueprint_->fill(qRgba(0, 0, 0, 0));
  }
}

void ImageOverlay::fillOverlayMenu()
{
  QMenu * menu = new QMenu();
  QSignalMapper * signalMapper = new QSignalMapper(this);

  for (std::string str_plugin_class : image_overlay_plugin_classes) {
    QString qstr_plugin_class = QString::fromStdString(str_plugin_class);
    QAction * action = new QAction(qstr_plugin_class, this);
    menu->addAction(action);
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    signalMapper->setMapping(action, qstr_plugin_class);
  }

  connect(signalMapper, SIGNAL(mapped(QString)), this, SLOT(addOverlay(QString)));

  ui_.add_overlay_button->setMenu(menu);
}

void ImageOverlay::updatePluginInstances(QTableWidgetItem * table_widget_item)
{
  int row = table_widget_item->row();

  std::string topic_name;
  std::string topic_type;

  if (auto topicItem = ui_.plugin_topic_table->item(row, COLUMN_TOPIC)) {
    topic_name = topicItem->text().toStdString();
  } else {
    return;
  }

  topic_type = ui_.plugin_topic_table->item(row, COLUMN_TYPE)->text().toStdString();

  // TODO(ijnek): Fix hack below, since only first plugin works.
  std::shared_ptr<ImageOverlayPlugin> plugin_instance = plugin_instances.front();
  subscriptions.push_back(
    node_->create_generic_subscription(
      topic_name, topic_type,
      rclcpp::QoS(10),
      [this, plugin_instance, topic_name]
        (const std::shared_ptr<rclcpp::SerializedMessage> msg) -> void {
        if (layer_blueprint_) {
          std::shared_ptr<QImage> layer = std::make_shared<QImage>(*layer_blueprint_);
          plugin_instance->overlay(layer, msg);
          ui_.image_frame->setLayer(topic_name, std::move(layer));
        }
      }
    )
  );
}


PLUGINLIB_EXPORT_CLASS(ImageOverlay, rqt_gui_cpp::Plugin)
