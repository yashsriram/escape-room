cmake_minimum_required(VERSION 2.8.3)
project(planner)

add_compile_options(-std=c++11)

find_package(
        catkin REQUIRED
        roscpp
        visualization_msgs
        gazebo_msgs
)

include_directories(
        include
        # ${catkin_INCLUDE_DIRS}
        /opt/ros/melodic/include
)
add_executable(
        ${PROJECT_NAME}_node
        src/planner_node.cpp
        include/robot/sensing/configuration_space.h
        include/robot/sensing/line_segment.h
        include/robot/sensing/room.h
        include/robot/planning/probabilistic_roadmap.h
        include/robot/planning/milestone.h
        include/robot/acting/differential_drive_agent.h
        include/robot/acting/human.h
)

target_link_libraries(
        ${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
)
