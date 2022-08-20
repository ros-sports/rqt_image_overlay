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

#ifndef COMPOSITOR_HPP_
#define COMPOSITOR_HPP_

#include <QImage>
#include <QObject>
#include <memory>
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"

namespace rqt_image_overlay
{
class ImageManager;
class OverlayManager;

class Compositor : public QObject
{
  Q_OBJECT

public:
  Compositor(
    const ImageManager & imageManager, const OverlayManager & overlayManager, float frequency);

  // Thread safe setter/getter for window
  void setWindow(const rclcpp::Duration & window);
  rclcpp::Duration getWindow() const;

  void setCallableSetImage(std::function<void(std::shared_ptr<QImage>)> setImage);

private:
  std::shared_ptr<QImage> compose();
  void timerEvent(QTimerEvent *) override;

  const ImageManager & imageManager;
  const OverlayManager & overlayManager;

  std::function<void(std::shared_ptr<QImage>)> setImage;

  // Wait window for collecting messages before composing image.
  // To access this value, use the getWindow() method to ensure thread-safety, even from within
  // this class.
  std::shared_ptr<rclcpp::Duration> window_;

  rclcpp::Clock systemClock{RCL_SYSTEM_TIME};
};

}  // namespace rqt_image_overlay

#endif  // COMPOSITOR_HPP_
