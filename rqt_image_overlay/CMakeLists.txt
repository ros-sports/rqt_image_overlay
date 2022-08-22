cmake_minimum_required(VERSION 3.8)
project(rqt_image_overlay)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(ros_image_to_qimage REQUIRED)
find_package(rqt_image_overlay_layer REQUIRED)

# Build rqt_image_overlay
set(SOURCES
  src/image_overlay.cpp
  src/composition_frame.cpp
  src/compositor.cpp
  src/overlay_manager.cpp
  src/overlay.cpp
  src/image_manager.cpp
  src/list_image_topics.cpp
  src/color_dialog_delegate.cpp
  src/overlay_manager_view.cpp)
qt5_wrap_cpp(SOURCES    # Must do this for qt's Meta-Object Compiler.
  src/image_overlay.hpp
  src/composition_frame.hpp
  src/compositor.hpp
  src/image_manager.hpp
  src/color_dialog_delegate.hpp
  src/overlay_manager_view.hpp)
qt5_wrap_ui(UIS resource/image_overlay.ui resource/configuration_dialog.ui)
add_library(rqt_image_overlay SHARED
  ${SOURCES}
  ${UIS})
target_include_directories(rqt_image_overlay PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(rqt_image_overlay
  rclcpp
  rqt_gui_cpp
  image_transport
  Qt5Widgets
  ros_image_to_qimage
  rqt_image_overlay_layer)

pluginlib_export_plugin_description_file(rqt_gui plugin.xml)  # This must be rqt_gui and not rqt_gui_cpp

# Install - libraries
install(
  TARGETS rqt_image_overlay
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install - script for running rqt_image_overlay standalone
install(PROGRAMS scripts/rqt_image_overlay
  DESTINATION lib/${PROJECT_NAME}
)

# Test
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(
  rqt_image_overlay
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
