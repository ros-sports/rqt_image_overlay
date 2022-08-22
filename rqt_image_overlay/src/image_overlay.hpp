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

#ifndef IMAGE_OVERLAY_HPP_
#define IMAGE_OVERLAY_HPP_

#include <QMenu>
#include <vector>
#include <memory>
#include "rqt_gui_cpp/plugin.h"

namespace Ui {class ImageOverlay;}
namespace rqt_image_overlay
{
class ImageManager;
class OverlayManager;
class Compositor;

class ImageOverlay : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  ImageOverlay();
  ~ImageOverlay();  // need a destructor (StackOverflow: 13414652)
  void initPlugin(qt_gui_cpp::PluginContext & context) override;
  void shutdownPlugin() override;
  void saveSettings(
    qt_gui_cpp::Settings &,
    qt_gui_cpp::Settings & instanceSettings) const override;
  void restoreSettings(
    const qt_gui_cpp::Settings &,
    const qt_gui_cpp::Settings & instanceSettings) override;
  bool hasConfiguration() const override;
  void triggerConfiguration() override;

public slots:
  void removeOverlay();

private:
  void fillOverlayMenu();

  std::unique_ptr<Ui::ImageOverlay> ui;
  std::unique_ptr<QMenu> menu;
  std::shared_ptr<ImageManager> imageManager;
  std::shared_ptr<OverlayManager> overlayManager;
  std::unique_ptr<Compositor> compositor;
};

}  // namespace rqt_image_overlay

#endif  // IMAGE_OVERLAY_HPP_
