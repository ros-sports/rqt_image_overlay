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
  instance_settings.setValue("compositor_window", compositor->getWindow().seconds());
  imageManager->saveSettings(instance_settings);
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

  if (instance_settings.contains("compositor_window")) {
    auto window_double = instance_settings.value("compositor_window").toDouble();
    auto duration = rclcpp::Duration::from_seconds(window_double);
    compositor->setWindow(duration);
  }
  imageManager->restoreSettings(instance_settings);
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
  auto configuration_dialog = std::make_unique<QDialog>();
  auto ui_configuration_dialog = std::make_unique<Ui::ConfigurationDialog>();
  ui_configuration_dialog->setupUi(configuration_dialog.get());

  // Load current configurations
  ui_configuration_dialog->window_spin_box->setValue(compositor->getWindow().seconds());
  auto options = imageManager->getCvtColorForDisplayOptions();
  ui_configuration_dialog->dynamic_scaling_check_box->setChecked(options.do_dynamic_scaling);
  ui_configuration_dialog->minimum_value_spin_box->setValue(options.min_image_value);
  ui_configuration_dialog->maximum_value_spin_box->setValue(options.max_image_value);
  ui_configuration_dialog->colormap_spin_box->setValue(options.colormap);
  ui_configuration_dialog->bg_label_spin_box->setValue(options.bg_label);

  if (configuration_dialog->exec() == QDialog::Accepted) {
    // Set configurations
    auto window_seconds = ui_configuration_dialog->window_spin_box->value();
    compositor->setWindow(rclcpp::Duration::from_seconds(window_seconds));

    cv_bridge::CvtColorForDisplayOptions options;
    options.do_dynamic_scaling = ui_configuration_dialog->dynamic_scaling_check_box->isChecked();
    options.min_image_value = ui_configuration_dialog->minimum_value_spin_box->value();
    options.max_image_value = ui_configuration_dialog->maximum_value_spin_box->value();
    options.colormap = ui_configuration_dialog->colormap_spin_box->value();
    options.bg_label = ui_configuration_dialog->bg_label_spin_box->value();
    imageManager->setCvtColorForDisplayOptions(options);
  }
}


}  // namespace rqt_image_overlay


PLUGINLIB_EXPORT_CLASS(rqt_image_overlay::ImageOverlay, rqt_gui_cpp::Plugin)
