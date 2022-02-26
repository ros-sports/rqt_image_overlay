cmake_minimum_required(VERSION 3.8)
project(rqt_image_overlay_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(message_filters REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rclcpp)
ament_export_dependencies(rosidl_runtime_cpp)
ament_export_dependencies(rcpputils)
ament_export_dependencies(pluginlib)
ament_export_dependencies(Qt5Widgets)
ament_export_dependencies(message_filters)

ament_package()
