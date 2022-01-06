# Build test_list_image_topics
find_package(rclcpp REQUIRED)

ament_add_gtest(test_list_image_topics
test_list_image_topics.cpp)

target_link_libraries(test_list_image_topics
  rqt_image_overlay
)

ament_target_dependencies(test_list_image_topics
  rclcpp)