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

#ifndef RQT_IMAGE_OVERLAY__IMAGE_OVERLAY_HPP_
#define RQT_IMAGE_OVERLAY__IMAGE_OVERLAY_HPP_

#include <QThread>
#include <vector>
#include "rqt_gui_cpp/plugin.h"
#include "./ui_image_overlay.h"
#include "rqt_image_overlay/compositor.hpp"
#include "rqt_image_overlay/overlay_manager.hpp"
#include "rqt_image_overlay/image_manager.hpp"

class QSignalMapper;

namespace rqt_image_overlay
{

class ImageOverlay : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  ImageOverlay();
  ~ImageOverlay();
  void initPlugin(qt_gui_cpp::PluginContext & context) override;
  void saveSettings(
    qt_gui_cpp::Settings &,
    qt_gui_cpp::Settings & instanceSettings) const override;
  void restoreSettings(
    const qt_gui_cpp::Settings &,
    const qt_gui_cpp::Settings & instanceSettings) override;

public slots:
  void addOverlay(QString plugin_class);
  void removeOverlay();

private:
  void fillOverlayMenu();

  Ui::ImageOverlay ui;

  QMenu * menu;
  std::vector<QSignalMapper *> signalMappers;

  QThread thread;

  ImageManager imageManager;
  OverlayManager overlayManager;

  Compositor compositor;
};

}  // namespace rqt_image_overlay

#endif  // RQT_IMAGE_OVERLAY__IMAGE_OVERLAY_HPP_
