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

#ifndef COLOR_DIALOG_DELEGATE_HPP_
#define COLOR_DIALOG_DELEGATE_HPP_

#include <QStyledItemDelegate>

namespace rqt_image_overlay
{

class ColorDialogDelegate : public QStyledItemDelegate
{
  Q_OBJECT

public:
  explicit ColorDialogDelegate(QObject * parent = nullptr);

  QWidget * createEditor(
    QWidget * parent, const QStyleOptionViewItem & option,
    const QModelIndex & index) const override;

  void setEditorData(QWidget * editor, const QModelIndex & index) const override;
  void setModelData(
    QWidget * editor, QAbstractItemModel * model,
    const QModelIndex & index) const override;
};

}  // namespace rqt_image_overlay

#endif  // COLOR_DIALOG_DELEGATE_HPP_
