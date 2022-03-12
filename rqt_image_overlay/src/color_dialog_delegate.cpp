// Copyright 2022 Kenji Brameld
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

#include <QColorDialog>
#include "color_dialog_delegate.hpp"

namespace rqt_image_overlay
{

ColorDialogDelegate::ColorDialogDelegate(QObject * parent)
: QStyledItemDelegate(parent)
{
}

QWidget * ColorDialogDelegate::createEditor(
  QWidget * parent,
  const QStyleOptionViewItem &,
  const QModelIndex &) const
{
  auto dialog = new QColorDialog{parent};
  dialog->setOptions(QColorDialog::ShowAlphaChannel);
  return dialog;
}

void ColorDialogDelegate::setEditorData(
  QWidget * editor,
  const QModelIndex & index) const
{
  auto color = index.data(Qt::DecorationRole).value<QColor>();
  QColorDialog * dialog = reinterpret_cast<QColorDialog *>(editor);
  dialog->setCurrentColor(color);
}

void ColorDialogDelegate::setModelData(
  QWidget * editor, QAbstractItemModel * model,
  const QModelIndex & index) const
{
  QColorDialog * dialog = reinterpret_cast<QColorDialog *>(editor);
  auto color = dialog->currentColor();
  model->setData(index, color, Qt::DecorationRole);
}

}  // namespace rqt_image_overlay
