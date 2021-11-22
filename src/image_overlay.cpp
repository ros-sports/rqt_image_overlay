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

#include <map>
#include <algorithm>
#include <vector>
#include <string>
#include "image_overlay/image_overlay.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "image_transport/image_transport.hpp"

ImageOverlay::ImageOverlay()
: rqt_gui_cpp::Plugin()
{
}

void ImageOverlay::initPlugin(qt_gui_cpp::PluginContext & context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  context.addWidget(widget_);

  updateTopicList();
  ui_.image_topics_combo_box->setCurrentIndex(ui_.image_topics_combo_box->findText(""));
  connect(
    ui_.image_topics_combo_box, SIGNAL(currentIndexChanged(int)), this,
    SLOT(onTopicChanged(int)));

  connect(ui_.refresh_image_topics_push_button, SIGNAL(pressed()), this, SLOT(updateTopicList()));
}

void ImageOverlay::updateTopicList()
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
    // qDebug("ImageView::updateTopicList() declared transport '%s'", it->c_str());
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

void ImageOverlay::onTopicChanged(int index)
{
  (void) index;
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


PLUGINLIB_EXPORT_CLASS(ImageOverlay, rqt_gui_cpp::Plugin)
