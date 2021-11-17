cmake_minimum_required(VERSION 3.8)
project(image_overlay)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rqt_gui_cpp REQUIRED)

# Include hpp files
include_directories(
  include
)

# Build image_overlay_lib
add_library(image_overlay_lib SHARED
  src/image_overlay.cpp)
ament_target_dependencies(image_overlay_lib
  rclcpp
  rqt_gui_cpp)

pluginlib_export_plugin_description_file(rqt_gui plugin.xml)  # This must be rqt_gui and not rqt_gui_cpp

# This flag must be set for the foxy branch to compile
target_compile_definitions(image_overlay_lib PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Build image_overlay executable
add_executable(image_overlay
  src/main.cpp)
ament_target_dependencies(image_overlay
  rclcpp
  pluginlib)

# Install - headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install - executables
install(TARGETS
  image_overlay
  DESTINATION lib/${PROJECT_NAME})

# Install - libraries
install(
  TARGETS image_overlay_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export includes
ament_export_include_directories(
  include
)

ament_package()
