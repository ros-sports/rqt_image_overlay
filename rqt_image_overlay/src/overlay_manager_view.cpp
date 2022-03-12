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

#include <QHeaderView>
#include <memory>
#include "overlay_manager_view.hpp"
#include "color_dialog_delegate.hpp"
#include "user_roles.hpp"

namespace rqt_image_overlay
{

OverlayManagerView::OverlayManagerView(QWidget * parent)
: QTableView(parent), colorDialogDelegate(std::make_unique<ColorDialogDelegate>())
{
}

OverlayManagerView::~OverlayManagerView() = default;

void OverlayManagerView::setModel(QAbstractItemModel * model)
{
  QTableView::setModel(model);
  horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

  for (int i = 0; i < model->columnCount(); ++i) {
    // Check whether a color dialog delegate should be used for the column
    // using user_roles::UseColorDialogRole
    QVariant useColorDialog = model->headerData(i, Qt::Horizontal, user_roles::UseColorDialogRole);
    if (useColorDialog.isValid() && useColorDialog.toBool()) {
      setItemDelegateForColumn(
        i, reinterpret_cast<QAbstractItemDelegate *>(colorDialogDelegate.get()));
    }

    // Check whether the column should have a specific fixed size, using Qt::SizeHintRole
    QVariant sizeHint = model->headerData(i, Qt::Horizontal, Qt::SizeHintRole);
    if (sizeHint.isValid()) {
      horizontalHeader()->setSectionResizeMode(i, QHeaderView::Fixed);
      setColumnWidth(i, sizeHint.toInt());
    }
  }
}


}  // namespace rqt_image_overlay
