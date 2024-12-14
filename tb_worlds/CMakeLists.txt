cmake_minimum_required(VERSION 3.22)
project(tb_worlds)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_minimal_tb3_sim REQUIRED)
find_package(nav2_minimal_tb4_sim REQUIRED)

# Install directories
install(DIRECTORY
  launch maps models worlds urdf configs
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
