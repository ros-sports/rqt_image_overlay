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

#include <QPainter>
#include <memory>
#include <utility>
#include <algorithm>
#include "composition_frame.hpp"

namespace rqt_image_overlay
{

CompositionFrame::CompositionFrame(QWidget * parent, Qt::WindowFlags flags)
: QFrame(parent, flags)
{
  connect(this, SIGNAL(delayedUpdate()), this, SLOT(update()), Qt::QueuedConnection);
}

void CompositionFrame::setImage(std::shared_ptr<QImage> image)
{
  std::atomic_store(&qimage, image);
  emit delayedUpdate();
}

void CompositionFrame::paintEvent(QPaintEvent * event)
{
  (void)event;
  QPainter painter(this);

  const std::shared_ptr<QImage> qimageCopy(std::atomic_load(&qimage));
  // Draw camera image
  if (qimageCopy && !qimageCopy->isNull()) {
    float widthRatio = static_cast<float>(width()) / qimageCopy->width();
    float heightRatio = static_cast<float>(height()) / qimageCopy->height();

    float scale = std::min(widthRatio, heightRatio);
    painter.scale(scale, scale);

    QPoint leftTop = QPoint(
      (width() / scale - qimageCopy->width()) / 2,
      (height() / scale - qimageCopy->height()) / 2);
    painter.drawImage(leftTop, *qimageCopy);
  } else {
    // default image with gradient
    QLinearGradient gradient(0, 0, frameRect().width(), frameRect().height());
    gradient.setColorAt(0, Qt::white);
    gradient.setColorAt(1, Qt::black);
    painter.setBrush(gradient);
    painter.drawRect(0, 0, frameRect().width() + 1, frameRect().height() + 1);
  }
}

}  // namespace rqt_image_overlay
