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

#include <QImage>
#include <QPainter>
#include "image_overlay/ratio_layouted_frame.hpp"

RatioLayoutedFrame::RatioLayoutedFrame(QWidget * parent, Qt::WindowFlags flags)
: QFrame(parent, flags)
{
  connect(this, SIGNAL(delayed_update()), this, SLOT(update()), Qt::QueuedConnection);
}

void RatioLayoutedFrame::setImage(const QImage & image)
{
  qimage_ = image.copy();
  emit delayed_update();
}

void RatioLayoutedFrame::paintEvent(QPaintEvent * event)
{
  (void)event;
  QPainter painter(this);
  if (!qimage_.isNull()) {
    QImage scaledImage = qimage_.scaled(width(), height(), Qt::KeepAspectRatio);
    QPoint leftTop = QPoint(
      (width() - scaledImage.width()) / 2,
      (height() - scaledImage.height()) / 2);
    painter.drawImage(leftTop, scaledImage);
  } else {
    // default image with gradient
    QLinearGradient gradient(0, 0, frameRect().width(), frameRect().height());
    gradient.setColorAt(0, Qt::white);
    gradient.setColorAt(1, Qt::black);
    painter.setBrush(gradient);
    painter.drawRect(0, 0, frameRect().width() + 1, frameRect().height() + 1);
  }
}
