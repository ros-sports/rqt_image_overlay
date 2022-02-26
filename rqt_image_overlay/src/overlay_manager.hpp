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

#ifndef OVERLAY_MANAGER_HPP_
#define OVERLAY_MANAGER_HPP_

#include <QAbstractTableModel>
#include <vector>
#include <string>
#include <memory>
#include "overlay.hpp"
#include "pluginlib/class_loader.hpp"
#include "rqt_image_overlay_layer/plugin_interface.hpp"

// forward declaration
namespace qt_gui_cpp {class Settings;}
namespace rclcpp {class Node;}
class Overlay;
namespace rqt_image_overlay {class OverlayTimeInfo;}

namespace rqt_image_overlay
{

// Refer to https://doc.qt.io/archives/4.6/model-view-model-subclassing.html
// for implementing methods of QAbstractTableModel

class OverlayManager : public QAbstractTableModel
{
public:
  explicit OverlayManager(const std::shared_ptr<rclcpp::Node> & node);
  const std::vector<std::string> & getDeclaredPluginClasses() const;
  bool addOverlay(std::string pluginClass);
  void removeOverlay(unsigned index);
  void overlay(QImage & image, const OverlayTimeInfo & overlayTimeInfo) const;
  void saveSettings(qt_gui_cpp::Settings & settings) const;
  void restoreSettings(const qt_gui_cpp::Settings & settings);

protected:
  // Overriding QAbstractTableModel methods
  int rowCount(const QModelIndex & parent = QModelIndex()) const override;
  int columnCount(const QModelIndex & parent = QModelIndex()) const override;
  QVariant data(const QModelIndex & index, int role = Qt::DisplayRole) const override;
  bool setData(const QModelIndex & index, const QVariant & value, int role = Qt::EditRole) override;
  Qt::ItemFlags flags(const QModelIndex & index) const override;
  QVariant headerData(
    int section, Qt::Orientation orientation,
    int role = Qt::DisplayRole) const override;
  bool insertRows(int row, int count, const QModelIndex & parent = QModelIndex()) override;
  bool removeRows(int row, int count, const QModelIndex & parent = QModelIndex()) override;

private:
  void timerEvent(QTimerEvent *) override;

  pluginlib::ClassLoader<rqt_image_overlay_layer::PluginInterface> pluginLoader;
  const std::vector<std::string> declaredPluginClasses;
  const std::shared_ptr<rclcpp::Node> & node;

  std::vector<std::unique_ptr<Overlay>> overlays;
  const std::vector<std::string> columns;

  const int statusIndex;  // keep track of status index to avoid constant look up
  int findStatusIndex() const;
};

}  // namespace rqt_image_overlay

#endif  // OVERLAY_MANAGER_HPP_
