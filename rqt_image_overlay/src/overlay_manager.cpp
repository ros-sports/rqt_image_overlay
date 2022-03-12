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

#include <QColor>
#include <string>
#include <memory>
#include <vector>
#include "overlay_manager.hpp"
#include "overlay.hpp"
#include "qt_gui_cpp/settings.h"
#include "user_roles.hpp"

#define STATUS_UPDATE_MS 200
#define STATUS_INDEX_UNFOUND -1

namespace rqt_image_overlay
{

OverlayManager::OverlayManager(const std::shared_ptr<rclcpp::Node> & node)
: pluginLoader("rqt_image_overlay_layer", "rqt_image_overlay_layer::PluginInterface"),
  declaredPluginClasses(pluginLoader.getDeclaredClasses()),
  node(node),
  columns{"Topic", "Type", "Plugin", "Status", "Color"},
  statusIndex(findStatusIndex())
{
  startTimer(STATUS_UPDATE_MS);
}

bool OverlayManager::addOverlay(std::string pluginClass)
{
  try {
    overlays.push_back(std::make_unique<Overlay>(pluginClass, pluginLoader, node));
  } catch (const std::exception & e) {
    qWarning("(OverlayManager) Failed to add overlay: %s", e.what());
    return false;
  }

  insertRows(overlays.size(), 1);
  return true;
}

void OverlayManager::removeOverlay(unsigned index)
{
  if (index < overlays.size()) {
    overlays.erase(overlays.begin() + index);
    removeRows(index, 1);
  } else {
    qWarning("(OverlayManager) Failed to remove overlay on row %d, which doesn't exist", index);
  }
}

const std::vector<std::string> & OverlayManager::getDeclaredPluginClasses() const
{
  return declaredPluginClasses;
}

int OverlayManager::rowCount(const QModelIndex &) const
{
  return overlays.size();
}

int OverlayManager::columnCount(const QModelIndex &) const
{
  return columns.size();
}

QVariant OverlayManager::data(const QModelIndex & index, int role) const
{
  std::string column = columns.at(index.column());

  if (role == Qt::DisplayRole || role == Qt::EditRole) {
    if (column == "Topic") {
      return QString::fromStdString(overlays.at(index.row())->getTopic());
    } else if (column == "Type") {
      return QString::fromStdString(overlays.at(index.row())->getMsgType());
    } else if (column == "Plugin") {
      return QString::fromStdString(overlays.at(index.row())->getPluginClass());
    } else if (column == "Status") {
      return QString::fromStdString(overlays.at(index.row())->getReceivedStatus());
    }
  }

  if (role == Qt::CheckStateRole) {
    if (column == "Topic") {
      if (overlays.at(index.row())->isEnabled()) {
        return Qt::Checked;
      } else {
        return Qt::Unchecked;
      }
    }
  }

  if (role == Qt::DecorationRole) {
    if (column == "Color") {
      return overlays.at(index.row())->getColor();
    }
  }

  return QVariant();
}

bool OverlayManager::setData(
  const QModelIndex & index, const QVariant & value,
  int role)
{
  std::string column = columns.at(index.column());

  if (role == Qt::EditRole) {
    if (column == "Topic") {
      overlays.at(index.row())->setTopic(value.toString().toStdString());
    }
    emit dataChanged(index, index);
    return true;
  }

  if (role == Qt::CheckStateRole) {
    if (column == "Topic") {
      overlays.at(index.row())->setEnabled(value.toBool());
    }
    emit dataChanged(index, index);
    return true;
  }

  if (role == Qt::DecorationRole) {
    if (column == "Color") {
      overlays.at(index.row())->setColor(value.value<QColor>());
    }
    emit dataChanged(index, index);
    return true;
  }

  return false;
}

Qt::ItemFlags OverlayManager::flags(const QModelIndex & index) const
{
  std::string column = columns.at(index.column());
  if (column == "Topic") {
    return QAbstractItemModel::flags(index) | Qt::ItemIsEditable | Qt::ItemIsUserCheckable;
  } else if (column == "Color") {
    return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
  }
  return QAbstractItemModel::flags(index) | Qt::ItemIsEnabled;
}

QVariant OverlayManager::headerData(
  int section, Qt::Orientation orientation,
  int role) const
{
  std::string column = columns.at(section);

  if (role == Qt::DisplayRole) {
    if (orientation == Qt::Horizontal) {
      if (column == "Color") {
        // Don't display column name for color, because it's self-explanatory
        return QVariant();
      } else {
        return QString::fromStdString(columns.at(section));
      }
    } else if (orientation == Qt::Vertical) {
      return QVariant();
    }
  }

  if (role == Qt::SizeHintRole) {
    if (column == "Color") {
      return QVariant(24);
    }
  }

  if (role == user_roles::UseColorDialogRole) {
    if (column == "Color") {
      return QVariant(true);
    }
  }

  return QAbstractTableModel::headerData(section, orientation, role);
}

bool OverlayManager::insertRows(int row, int, const QModelIndex & parent)
{
  beginInsertRows(parent, row, row);
  endInsertRows();
  return true;
}

bool OverlayManager::removeRows(int row, int, const QModelIndex & parent)
{
  beginRemoveRows(parent, row, row);
  endRemoveRows();
  return true;
}

void OverlayManager::overlay(QImage & image, const OverlayTimeInfo & overlayTimeInfo) const
{
  QPainter painter(&image);
  for (auto & overlay : overlays) {
    if (overlay->isEnabled()) {
      painter.save();
      QPen pen(overlay->getColor());
      painter.setPen(pen);
      overlay->overlay(painter, overlayTimeInfo);
      painter.restore();
    }
  }
}

void OverlayManager::saveSettings(qt_gui_cpp::Settings & settings) const
{
  QList<QVariant> list;

  for (auto const & overlay : overlays) {
    QMap<QString, QVariant> map;
    map.insert("Topic", QString::fromStdString(overlay->getTopic()));
    map.insert("Plugin", QString::fromStdString(overlay->getPluginClass()));
    map.insert("Enabled", overlay->isEnabled());
    map.insert("Color", overlay->getColor());
    list.append(QVariant(map));
  }

  settings.setValue("overlay table", QVariant(list));
}

void OverlayManager::restoreSettings(const qt_gui_cpp::Settings & settings)
{
  if (settings.contains("overlay table")) {
    QList<QVariant> list = settings.value("overlay table").toList();
    for (QList<QVariant>::iterator i = list.begin(); i != list.end(); ++i) {
      QMap<QString, QVariant> map = i->toMap();
      if (map.contains("Plugin")) {
        std::string plugin = map.value("Plugin").toString().toStdString();
        if (!addOverlay(plugin)) {
          continue;  // Couldn't add overlay of the plugin type successfully, skip this one
        }
      }

      if (map.contains("Topic")) {
        std::string topic = map.value("Topic").toString().toStdString();
        overlays.back()->setTopic(topic);
      }

      if (map.contains("Enabled")) {
        bool enabled = map.value("Enabled").toBool();
        overlays.back()->setEnabled(enabled);
      }

      if (map.contains("Color")) {
        QColor color = map.value("Color").value<QColor>();
        overlays.back()->setColor(color);
      }
    }
  }
}

void OverlayManager::timerEvent(QTimerEvent * event)
{
  (void) event;
  if (!overlays.empty() && statusIndex != STATUS_INDEX_UNFOUND) {
    QModelIndex topLeft = createIndex(0, statusIndex);
    QModelIndex bottomRight = createIndex(overlays.size() - 1, statusIndex);
    emit dataChanged(topLeft, bottomRight, {Qt::DisplayRole});
  }
}

int OverlayManager::findStatusIndex() const
{
  auto it = find(columns.begin(), columns.end(), "Status");

  if (it != columns.end()) {
    return it - columns.begin();
  } else {
    qWarning("(OverlayManager) Couldn't find index of 'Status' column, can't display status");
    return STATUS_INDEX_UNFOUND;
  }
}

}  // namespace rqt_image_overlay
