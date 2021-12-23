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

#include <QMenu>
#include <QSignalMapper>
#include <functional>
#include <string>
#include "rqt_image_overlay/image_overlay.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace rqt_image_overlay
{

ImageOverlay::ImageOverlay()
: rqt_gui_cpp::Plugin(), thread(this), imageManager(node_), overlayManager(node_),
  compositor(imageManager, overlayManager, 30.0)
{
}

void ImageOverlay::initPlugin(qt_gui_cpp::PluginContext & context)
{
  QWidget * widget = new QWidget();
  ui_.setupUi(widget);
  context.addWidget(widget);

  ui_.overlay_table->setModel(&overlayManager);
  ui_.image_topics_combo_box->setModel(&imageManager);

  ui_.overlay_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

  fillOverlayMenu();

  ui_.image_topics_combo_box->setCurrentIndex(ui_.image_topics_combo_box->findText(""));
  connect(
    ui_.image_topics_combo_box, SIGNAL(currentTextChanged(QString)), &imageManager,
    SLOT(onTopicChanged(QString)));

  connect(
    ui_.refresh_image_topics_button, SIGNAL(pressed()), &imageManager,
    SLOT(updateImageTopicList()));

  connect(ui_.remove_overlay_button, SIGNAL(pressed()), this, SLOT(removeOverlay()));

  compositor.moveToThread(&thread);
  thread.start();

  compositor.setCallableSetImage(
    std::bind(
      &RatioLayoutedFrame::setImage, ui_.image_frame,
      std::placeholders::_1));
}

void ImageOverlay::addOverlay(QString plugin_class)
{
  overlayManager.addOverlay(plugin_class.toStdString());
}

void ImageOverlay::removeOverlay()
{
  QItemSelectionModel * select = ui_.overlay_table->selectionModel();
  for (auto & index : select->selectedRows()) {
    overlayManager.removeOverlay(index.row());
  }
}


void ImageOverlay::saveSettings(
  qt_gui_cpp::Settings &,
  qt_gui_cpp::Settings & instance_settings) const
{
  instance_settings.setValue("image_topic", ui_.image_topics_combo_box->currentText());
  overlayManager.saveSettings(instance_settings);
}

void ImageOverlay::restoreSettings(
  const qt_gui_cpp::Settings &,
  const qt_gui_cpp::Settings & instance_settings)
{
  if (instance_settings.contains("image_topic")) {
    QString topic = instance_settings.value("image_topic").toString();
    if (topic != "") {
      imageManager.setTopicExplicitly(topic);
      ui_.image_topics_combo_box->setCurrentIndex(1);
    }
  }

  overlayManager.restoreSettings(instance_settings);
}

void ImageOverlay::fillOverlayMenu()
{
  menu = new QMenu();
  QSignalMapper * signalMapper = new QSignalMapper(this);
  signalMappers.push_back(signalMapper);

  for (std::string str_plugin_class : overlayManager.getDeclaredPluginClasses()) {
    QString qstr_plugin_class = QString::fromStdString(str_plugin_class);
    QAction * action = new QAction(qstr_plugin_class, this);
    menu->addAction(action);  // ownership transferred
    connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
    signalMapper->setMapping(action, qstr_plugin_class);
  }

  connect(signalMapper, SIGNAL(mapped(QString)), this, SLOT(addOverlay(QString)));

  ui_.add_overlay_button->setMenu(menu);
}


ImageOverlay::~ImageOverlay()
{
  if (menu) {
    delete menu;
  }

  for (auto mapper : signalMappers) {
    if (mapper) {
      delete mapper;
    }
  }
}

}  // namespace rqt_image_overlay


PLUGINLIB_EXPORT_CLASS(rqt_image_overlay::ImageOverlay, rqt_gui_cpp::Plugin)
