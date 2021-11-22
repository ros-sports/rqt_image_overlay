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
#include "rqt_gui_cpp/plugin.h"
#include "./ui_image_overlay.h"
#include "image_transport/subscriber.hpp"

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
  image_transport::Subscriber subscriber_;

protected slots:
  virtual void updateTopicList();
  virtual void onTopicChanged(int index);

private:
  QWidget * widget_;
  Ui::ImageOverlay ui_;
};

#endif  // IMAGE_OVERLAY__IMAGE_OVERLAY_HPP_
