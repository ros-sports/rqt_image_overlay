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

#include <string>
#include <memory>
#include <vector>
#include <iostream>
#include "rqt_image_overlay/overlay_manager.hpp"
#include "rqt_image_overlay/overlay.hpp"
#include "qt_gui_cpp/settings.h"

namespace rqt_image_overlay
{

OverlayManager::OverlayManager(const std::shared_ptr<rclcpp::Node> & node, QObject * parent)
: QAbstractTableModel(parent),
  pluginLoader("rqt_image_overlay", "rqt_image_overlay::ImageOverlayPlugin"),
  declaredPluginClasses(pluginLoader.getDeclaredClasses()),
  node(node),
  columns{"Topic", "Type", "Plugin"}
{
}

bool OverlayManager::addOverlay(std::string pluginClass)
{
  try {
    overlays.push_back(std::make_unique<Overlay>(pluginClass, pluginLoader, node));
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
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
    std::cerr << "Failed to remove overlay on row " << index << ", which doesn't exist" <<
      std::endl;
  }
}

const std::vector<std::string> & OverlayManager::getDeclaredPluginClasses()
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
    }
  }

  if (role == Qt::CheckStateRole) {
    if (column == "Topic") {
      if (overlays.at(index.row())->getEnabled()) {
        return Qt::Checked;
      } else {
        return Qt::Unchecked;
      }
    }
  }

  return QVariant();
}

bool OverlayManager::setData(
  const QModelIndex & index, const QVariant & value,
  int role)
{
  if (role == Qt::EditRole) {
    if (columns.at(index.column()) == "Topic") {
      overlays.at(index.row())->setTopic(value.toString().toStdString());
    }
    emit dataChanged(index, index);
    return true;
  }

  if (role == Qt::CheckStateRole) {
    if (columns.at(index.column()) == "Topic") {
      overlays.at(index.row())->setEnabled(value.toBool());
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
  }
  return QAbstractItemModel::flags(index) | Qt::ItemIsEnabled;
}

QVariant OverlayManager::headerData(
  int section, Qt::Orientation orientation,
  int role) const
{
  if (role == Qt::DisplayRole) {
    if (orientation == Qt::Horizontal) {
      return QString::fromStdString(columns.at(section));
    } else if (orientation == Qt::Vertical) {
      return QVariant();
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

void OverlayManager::overlay(QImage & image) const
{
  for (auto & overlay : overlays) {
    overlay->overlay(image);
  }
}

void OverlayManager::saveSettings(qt_gui_cpp::Settings & settings) const
{
  QList<QVariant> list;

  for (auto & overlay : overlays) {
    QMap<QString, QVariant> map;
    map.insert("Topic", QString::fromStdString(overlay->getTopic()));
    map.insert("Plugin", QString::fromStdString(overlay->getPluginClass()));
    map.insert("Enabled", overlay->getEnabled());
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
    }
  }
}

}  // namespace rqt_image_overlay
