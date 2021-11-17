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

#include <memory>
#include "pluginlib/class_loader.hpp"
#include "image_overlay/image_overlay_plugin.hpp"

int main(int argc, char * argv[])
{
  // rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<ImageOverlayNode>());
  // rclcpp::shutdown();

  // To avoid unused parameter warnings
  (void) argc;
  (void) argv;

  pluginlib::ClassLoader<ImageOverlayPlugin> plugin_loader("image_overlay", "ImageOverlayPlugin");
  for (auto str : plugin_loader.getDeclaredClasses()) {
    printf("%s", str.c_str());
  }

  try {
    std::shared_ptr<ImageOverlayPlugin> triangle = plugin_loader.createSharedInstance(
      "SamplePlugin1");
    triangle->initialize(10.0);

    printf("Triangle area: %.2f\n", triangle->area());
  } catch (pluginlib::PluginlibException & ex) {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  return 0;
}
