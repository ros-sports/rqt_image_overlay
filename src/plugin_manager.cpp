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
#include "image_overlay/plugin_manager.hpp"
#include "image_overlay/plugin.hpp"

PluginManager::PluginManager(const rclcpp::Node::SharedPtr & node, QObject * parent)
: QAbstractTableModel(parent),
  plugin_loader("image_overlay", "ImageOverlayPlugin"),
  declared_classes(plugin_loader.getDeclaredClasses()),
  node_(node),
  columns{"Show", "Topic", "Type", "Plugin", "Detail"}
{
}

void PluginManager::addPlugin(std::string plugin_class)
{
  insertRows(plugins.size(), 1);

  plugins.push_back(
    std::make_unique<Plugin>(
      plugin_class,
      plugin_loader.createSharedInstance(plugin_class), node_));
}

const std::vector<std::string> & PluginManager::getDeclaredClasses()
{
  return declared_classes;
}

int PluginManager::rowCount(const QModelIndex &) const
{
  return plugins.size();
}

int PluginManager::columnCount(const QModelIndex &) const
{
  return columns.size();
}

QVariant PluginManager::data(const QModelIndex & index, int role) const
{
  std::string column = columns.at(index.column());

  if (role == Qt::DisplayRole || role == Qt::EditRole) {
    if (column == "Topic") {
      return QString::fromStdString(plugins.at(index.row())->getTopic());
    } else if (column == "Type") {
      return QString::fromStdString(plugins.at(index.row())->getMsgType());
    } else if (column == "Plugin") {
      return QString::fromStdString(plugins.at(index.row())->getPluginClass());
    } else if (column == "Detail") {
      return QVariant();
    }
  }

  if (role == Qt::CheckStateRole) {
    if (column == "Show") {
      if (plugins.at(index.row())->getEnabled()) {
        return Qt::Checked;
      } else {
        return Qt::Unchecked;
      }
    }
  }

  return QVariant();
}

bool PluginManager::setData(
  const QModelIndex & index, const QVariant & value,
  int role)
{
  if (role == Qt::EditRole) {
    if (columns.at(index.column()) == "Topic") {
      plugins.at(index.row())->setTopic(value.toString().toStdString());
    }
    emit dataChanged(index, index);
    return true;
  }

  if (role == Qt::CheckStateRole) {
    if (columns.at(index.column()) == "Show") {
      plugins.at(index.row())->setEnabled(value.toBool());
    }
    emit dataChanged(index, index);
    return true;
  }
  return false;
}

Qt::ItemFlags PluginManager::flags(const QModelIndex & index) const
{
  std::string column = columns.at(index.column());
  if (column == "Topic") {
    return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
  } else if (column == "Show") {
    return QAbstractItemModel::flags(index) | Qt::ItemIsUserCheckable;
  }
  return QAbstractItemModel::flags(index) | Qt::ItemIsEnabled;
}

QVariant PluginManager::headerData(
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

bool PluginManager::insertRows(int row, int, const QModelIndex & parent)
{
  beginInsertRows(parent, row, row);
  endInsertRows();
  return true;
}

void PluginManager::overlay(QImage & image) const
{
  for (auto & plugin : plugins) {
    plugin->overlay(image);
  }
}
