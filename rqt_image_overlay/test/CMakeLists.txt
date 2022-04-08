# Build test_list_image_topics
find_package(rclcpp REQUIRED)
find_package(compressed_image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_add_gtest(test_list_image_topics
  test_list_image_topics.cpp)

target_link_libraries(test_list_image_topics
  rqt_image_overlay
)

ament_target_dependencies(test_list_image_topics
  rclcpp
  compressed_image_transport
  std_msgs
  sensor_msgs)

# Build test_msg_storage
ament_add_gtest(test_msg_storage
  test_msg_storage.cpp)

target_link_libraries(test_msg_storage
  rqt_image_overlay
)

# Build test_image_manager
ament_add_gtest(test_image_manager
  test_image_manager.cpp)

target_link_libraries(test_image_manager
  rqt_image_overlay
)
