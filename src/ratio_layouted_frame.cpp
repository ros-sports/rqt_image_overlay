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
#include "image_overlay/ratio_layouted_frame.hpp"

RatioLayoutedFrame::RatioLayoutedFrame(QWidget * parent, Qt::WindowFlags flags)
: QLabel(parent, flags)
{
}

void RatioLayoutedFrame::resizeEvent(QResizeEvent * event)
{
  (void) event;

  // Test, display image (https://stackoverflow.com/questions/8211982/qt-resizing-a-qlabel-containing-a-qpixmap-while-keeping-its-aspect-ratio)
  QImage myImage;
  myImage.load("/home/ijnek/Pictures/test-face.jpg");
  setPixmap(QPixmap::fromImage(myImage).scaled(width(), height(), Qt::KeepAspectRatio));
}
