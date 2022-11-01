cmake_minimum_required(VERSION 3.8)
project(tb3_worlds)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlebot3_gazebo REQUIRED)

# Install programs
install(PROGRAMS
    scripts/block_spawner.py
    scripts/set_init_amcl_pose.py
    DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch maps models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
