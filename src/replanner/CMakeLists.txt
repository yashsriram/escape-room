cmake_minimum_required(VERSION 2.8.3)
project(replanner)

add_compile_options(-std=c++11)

find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        rospy
        visualization_msgs
        gazebo_msgs
)

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
        #  INCLUDE_DIRS include
        #  LIBRARIES planner
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
)

include_directories(
        include
        ../../devel/include
        ${catkin_INCLUDE_DIRS}
        /opt/ros/melodic/include
)

add_executable(
        ${PROJECT_NAME}_node
        src/replanner_node.cpp
)

target_link_libraries(
        ${PROJECT_NAME}_node
        line
        line_extraction
        line_extraction_ros
        ${catkin_LIBRARIES}
)
