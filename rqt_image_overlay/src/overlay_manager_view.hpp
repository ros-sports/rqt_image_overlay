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

#ifndef OVERLAY_MANAGER_VIEW_HPP_
#define OVERLAY_MANAGER_VIEW_HPP_

#include <QTableView>
#include <memory>

namespace rqt_image_overlay
{
class ColorDialogDelegate;

class OverlayManagerView
  : public QTableView
{
  Q_OBJECT

public:
  explicit OverlayManagerView(QWidget * parent = nullptr);
  ~OverlayManagerView();  // need a destructor (StackOverflow: 13414652)
  void setModel(QAbstractItemModel * model) override;

private:
  std::unique_ptr<ColorDialogDelegate> colorDialogDelegate;
};


}  // namespace rqt_image_overlay

#endif  // OVERLAY_MANAGER_VIEW_HPP_
