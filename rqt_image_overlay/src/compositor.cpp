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
#include "compositor.hpp"
#include "image_manager.hpp"
#include "overlay_manager.hpp"

namespace rqt_image_overlay
{

Compositor::Compositor(
  const ImageManager & imageManager, const OverlayManager & overlayManager, float frequency)
: imageManager(imageManager), overlayManager(overlayManager),
  window_(std::make_shared<rclcpp::Duration>(0, 300000000))
{
  startTimer(1000.0 / frequency);
}

void Compositor::setWindow(const rclcpp::Duration & window)
{
  std::atomic_store(&window_, std::make_shared<rclcpp::Duration>(window));
}

rclcpp::Duration Compositor::getWindow() const
{
  auto window = std::atomic_load(&window_);
  return *window;
}

void Compositor::setCallableSetImage(std::function<void(std::shared_ptr<QImage>)> setImage)
{
  this->setImage = setImage;
}

std::shared_ptr<QImage> Compositor::compose()
{
  if (!imageManager.imageAvailable()) {
    return nullptr;
  }

  // Get window_ value through the getWindow() method so that it is thread-safe
  auto window = getWindow();

  rclcpp::Time targetTime = systemClock.now() - window;
  auto [composition, imageHeaderTime] = imageManager.getClosestImageAndHeaderTime(targetTime);
  OverlayTimeInfo overlayTimeInfo{targetTime, imageHeaderTime};
  overlayManager.overlay(*composition, overlayTimeInfo);
  return composition;
}


void Compositor::timerEvent(QTimerEvent * event)
{
  (void) event;

  if (!setImage) {
    qWarning("(Compositor) setCallableSetImage method not called");
    return;
  }

  if (auto image = compose(); image) {
    setImage(std::move(image));
  }
}

}  // namespace rqt_image_overlay
