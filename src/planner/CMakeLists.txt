cmake_minimum_required(VERSION 2.8.3)
project(planner)

add_compile_options(-std=c++11)

find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        rospy
        visualization_msgs
        gazebo_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES listener
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        /opt/ros/melodic/include
)
add_executable(
        ${PROJECT_NAME}_node
        src/planner_node.cpp
)

target_link_libraries(
        ${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
)
