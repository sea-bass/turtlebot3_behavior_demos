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
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(yaml-cpp REQUIRED)

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
    #src/vision_behaviors.cpp
)
include_directories(include)
add_executable(autonomy_node src/autonomy_node.cpp ${AUTONOMY_SOURCES})
install(TARGETS
    autonomy_node
    DESTINATION lib/${PROJECT_NAME}
)
ament_target_dependencies(autonomy_node
    ament_index_cpp rclcpp rclcpp_action behaviortree_cpp_v3 nav2_msgs yaml-cpp)
target_link_libraries(autonomy_node ${YAML_CPP_LIBRARIES})

# Install directories
install(DIRECTORY
  bt_xml launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# find_package(ament_cmake REQUIRED COMPONENTS 
#     roscpp 
#     cv_bridge
#     actionlib 
#     sensor_msgs
#     move_base_msgs 
#     image_transport
#     behaviortree_cpp_v3)
# find_package(OpenCV REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

# ###################################
# ## catkin specific configuration ##
# ###################################
# ## The catkin_package macro generates cmake config files for your package
# ## Declare things to be passed to dependent projects
# ## INCLUDE_DIRS: uncomment this if your package contains header files
# ## LIBRARIES: libraries you create in this project that dependent projects also need
# ## CATKIN_DEPENDS: catkin_packages dependent projects also need
# ## DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES ${OpenCV_LIBS}
#    CATKIN_DEPENDS 
#    DEPENDS
# )

# ###########
# ## Build ##
# ###########

# ## Specify additional locations of header files
# ## Your package locations should be listed before other locations
# include_directories(
#     include
#     ${catkin_INCLUDE_DIRS}
#     ${OpenCV_INCLUDE_DIRS}
# )

# ## Declare a C++ executable
# ## With catkin_make all packages are built within a single CMake context
# ## The recommended prefix ensures that target names across packages don't collide
# set(AUTONOMY_SOURCES
#     src/navigation_behaviors.cpp 
#     src/vision_behaviors.cpp
# )
# add_executable(
#     autonomy_node src/autonomy_node.cpp ${AUTONOMY_SOURCES}
# )

# ## Specify libraries to link a library or executable target against
# target_link_libraries(autonomy_node
#     ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp
# )
