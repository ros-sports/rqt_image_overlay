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

#include <memory>
#include <utility>
#include "image_overlay/compositor.hpp"
#include "image_overlay/image_manager.hpp"
#include "image_overlay/plugin_manager.hpp"

Compositor::Compositor(
  const ImageManager & imageManager, const PluginManager & pluginManager,
  float frequency, QObject * parent)
: QObject(parent), imageManager(imageManager), pluginManager(pluginManager)
{
  startTimer(1000.0 / frequency);
}

void Compositor::setCallableSetImage(std::function<void(std::unique_ptr<QImage>)> setImage)
{
  this->setImage = setImage;
}

std::unique_ptr<QImage> Compositor::compose()
{
  std::unique_ptr<QImage> composition = imageManager.getImage();
  if (composition != nullptr) {
    pluginManager.overlay(*composition);
  }

  return composition;
}


void Compositor::timerEvent(QTimerEvent * event)
{
  (void) event;

  if (!setImage) {
    std::cerr << "(Compositor) setCallableSetImage method not called" << std::endl;
    return;
  }

  std::unique_ptr<QImage> image = compose();

  if (!image) {
    return;
  }

  setImage(std::move(image));
}
