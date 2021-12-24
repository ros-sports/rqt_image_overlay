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
find_package(image_transport_helpers REQUIRED)
find_package(ros_image_to_qimage REQUIRED)

# Build rqt_image_overlay
set(SOURCES
  src/image_overlay.cpp
  src/ratio_layouted_frame.cpp
  src/compositor.cpp
  src/overlay_manager.cpp
  src/overlay.cpp
  src/image_manager.cpp)
qt5_wrap_cpp(SOURCES    # Must do this for qt's Meta-Object Compiler.
  include/rqt_image_overlay/image_overlay.hpp
  include/rqt_image_overlay/ratio_layouted_frame.hpp
  include/rqt_image_overlay/compositor.hpp
  include/rqt_image_overlay/image_manager.hpp)
qt5_wrap_ui(UIS src/image_overlay.ui)
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
  image_transport_helpers
  ros_image_to_qimage)

pluginlib_export_plugin_description_file(rqt_gui plugin.xml)  # This must be rqt_gui and not rqt_gui_cpp

# This flag must be set for the foxy branch to compile
target_compile_definitions(rqt_image_overlay PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Install - headers
install(
  DIRECTORY include/
  DESTINATION include)

# Install - libraries
install(
  TARGETS rqt_image_overlay
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install - scripts
install(PROGRAMS scripts/rqt_image_overlay
  DESTINATION lib/${PROJECT_NAME}
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export includes
ament_export_include_directories(include)

ament_export_libraries(
  rqt_image_overlay
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(rclcpp)
ament_export_dependencies(pluginlib)
ament_export_dependencies(rqt_gui_cpp)
ament_export_dependencies(image_transport)
ament_export_dependencies(Qt5Widgets)
ament_export_dependencies(image_transport_helpers)
ament_export_dependencies(ros_image_to_qimage)

ament_package()
