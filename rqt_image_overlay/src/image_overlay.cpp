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

#include <functional>
#include <string>
#include <memory>
#include "./ui_image_overlay.h"
#include "./ui_configuration_dialog.h"
#include "image_overlay.hpp"
#include "compositor.hpp"
#include "overlay_manager.hpp"
#include "image_manager.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace rqt_image_overlay
{

ImageOverlay::ImageOverlay()
: rqt_gui_cpp::Plugin()
{
}

ImageOverlay::~ImageOverlay() = default;

void ImageOverlay::initPlugin(qt_gui_cpp::PluginContext & context)
{
  ui = std::make_unique<Ui::ImageOverlay>();
  ui_configuration_dialog = std::make_unique<Ui::ConfigurationDialog>();
  menu = std::make_unique<QMenu>();
  imageManager = std::make_shared<ImageManager>(node_);
  overlayManager = std::make_shared<OverlayManager>(node_);
  compositor = std::make_unique<Compositor>(*imageManager, *overlayManager, 30.0);

  QWidget * widget = new QWidget();
  ui->setupUi(widget);
  context.addWidget(widget);  // transfer ownership

  ui->overlay_manager_view->setModel(overlayManager.get());
  ui->image_topics_combo_box->setModel(imageManager.get());

  fillOverlayMenu();

  ui->image_topics_combo_box->setCurrentIndex(ui->image_topics_combo_box->findText(""));
  connect(
    ui->image_topics_combo_box, SIGNAL(currentIndexChanged(int)), imageManager.get(),
    SLOT(onTopicChanged(int)));

  connect(
    ui->refresh_image_topics_button, SIGNAL(pressed()), imageManager.get(),
    SLOT(updateImageTopicList()));

  connect(ui->remove_overlay_button, SIGNAL(pressed()), this, SLOT(removeOverlay()));

  compositor->setCallableSetImage(
    std::bind(
      &CompositionFrame::setImage, ui->image_frame,
      std::placeholders::_1));
}

void ImageOverlay::shutdownPlugin()
{
  // reset pointers in reverse order they were created in initPlugin
  compositor.reset();
  overlayManager.reset();
  imageManager.reset();
  menu.reset();
  ui.reset();
}

void ImageOverlay::removeOverlay()
{
  QItemSelectionModel * select = ui->overlay_manager_view->selectionModel();
  if (select) {
    for (auto const & index : select->selectedRows()) {
      overlayManager->removeOverlay(index.row());
    }
  }
}


void ImageOverlay::saveSettings(
  qt_gui_cpp::Settings &,
  qt_gui_cpp::Settings & instance_settings) const
{
  auto optionalImageTopic = imageManager->getImageTopic(ui->image_topics_combo_box->currentIndex());
  if (optionalImageTopic.has_value()) {
    auto imageTopic = optionalImageTopic.value();
    instance_settings.setValue("image_topic", QString::fromStdString(imageTopic.topic));
    instance_settings.setValue("image_transport", QString::fromStdString(imageTopic.transport));
  }
  instance_settings.setValue("window", compositor->window().seconds());

  overlayManager->saveSettings(instance_settings);
}

void ImageOverlay::restoreSettings(
  const qt_gui_cpp::Settings &,
  const qt_gui_cpp::Settings & instance_settings)
{
  if (instance_settings.contains("image_topic") && instance_settings.contains("image_transport")) {
    std::string topic = instance_settings.value("image_topic").toString().toStdString();
    std::string transport = instance_settings.value("image_transport").toString().toStdString();
    imageManager->addImageTopicExplicitly(ImageTopic{topic, transport});
    ui->image_topics_combo_box->setCurrentIndex(1);
  }
  auto window_setting = instance_settings.value("window");
  if (!window_setting.isNull()) {
    auto window_string = window_setting.toString().toStdString();
    char * err;
    double window = std::strtod(window_string.c_str(), &err);
    if (err == window_string.c_str()) {
      // double conversion error
    } else {
      compositor->setWindow(rclcpp::Duration::from_seconds(window));
    }
  }
  overlayManager->restoreSettings(instance_settings);
}

void ImageOverlay::fillOverlayMenu()
{
  menu->clear();

  for (const std::string & str_plugin_class : overlayManager->getDeclaredPluginClasses()) {
    QString qstr_plugin_class = QString::fromStdString(str_plugin_class);
    QAction * action = new QAction(qstr_plugin_class, this);
    menu->addAction(action);  // ownership transferred
    connect(
      action, &QAction::triggered, [this, str_plugin_class] {
        overlayManager->addOverlay(str_plugin_class);
      });
  }

  ui->add_overlay_button->setMenu(menu.get());
}

bool ImageOverlay::hasConfiguration() const
{
  return true;
}

void ImageOverlay::triggerConfiguration()
{
  if (configuration_dialog) {
    return;
  }
  configuration_dialog = std::make_unique<QDialog>();
  configuration_dialog->setAttribute(Qt::WA_DeleteOnClose);
  ui_configuration_dialog->setupUi(configuration_dialog.get());
  ui_configuration_dialog->window->setValue(compositor->window().seconds());
  auto connection = connect(
    configuration_dialog.get(), &QDialog::finished, [this](int result) {
      if (result == QDialog::Accepted) {
        auto window_seconds = ui_configuration_dialog->window->value();
        compositor->setWindow(rclcpp::Duration::from_seconds(window_seconds));
      }
      configuration_dialog.reset();
    });
  configuration_dialog->open();
}


}  // namespace rqt_image_overlay


PLUGINLIB_EXPORT_CLASS(rqt_image_overlay::ImageOverlay, rqt_gui_cpp::Plugin)
