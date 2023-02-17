cmake_minimum_required(VERSION 3.8)
project(tb3_autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Install directories
install(DIRECTORY
  bt_xml launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python package
ament_python_install_package(
    tb3_behaviors
    PACKAGE_DIR python/tb3_behaviors)

# Install Python scripts
install(PROGRAMS
    scripts/autonomy_node.py
    scripts/test_move_base.py
    scripts/test_vision.py
    DESTINATION lib/${PROJECT_NAME}
)

# Install C++ nodes
set(AUTONOMY_SOURCES
    src/navigation_behaviors.cpp 
    src/vision_behaviors.cpp
)
set(TARGET_DEPENDS
    ament_index_cpp rclcpp rclcpp_action
    nav2_msgs sensor_msgs cv_bridge image_transport
    behaviortree_cpp tf2 tf2_ros tf2_geometry_msgs yaml-cpp
)
include_directories(include)
add_executable(autonomy_node_cpp src/autonomy_node.cpp ${AUTONOMY_SOURCES})
install(TARGETS
    autonomy_node_cpp
    DESTINATION lib/${PROJECT_NAME}
)
ament_target_dependencies(autonomy_node_cpp ${TARGET_DEPENDS})
target_link_libraries(autonomy_node_cpp ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES})

ament_package()
