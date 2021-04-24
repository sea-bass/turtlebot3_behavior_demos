cmake_minimum_required(VERSION 3.5)
project(tb3_autonomy)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    cv_bridge
    actionlib 
    sensor_msgs
    move_base_msgs 
    image_transport
    behaviortree_cpp_v3)
find_package(OpenCV REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${OpenCV_LIBS}
   CATKIN_DEPENDS 
   DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
set(AUTONOMY_SOURCES
    src/navigation_behaviors.cpp 
    src/vision_behaviors.cpp
)
add_executable(
    autonomy_node src/autonomy_node.cpp ${AUTONOMY_SOURCES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(autonomy_node
    ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp
)
