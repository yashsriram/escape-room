# escape-room
## description
- Simple and safe real-time navigation for no-noise differential-drive agent with LIDAR for escaping static environments.
- Dynamic known environments can be managed but safety is not guranteed.
## roadmap
- Problems solved until now are documented in `report.pdf`
## code
- The project is a typical ROS project.
- `src/` contains all source code.
- `src/planner` and `src/replanner` contain the core logic, the others are used as libraries.
## documentation
- For most of the code, the documentation is itself.
## usage
- Open a terminal at project root (the directory containing this file).
### Known environment with humans
- `roscore`
- `catkin_make`
- `source devel/setup.bash`
- `roslaunch turtlebot3_gazebo turtlebot3_zigzag_cylinder.launch`
### Unknown environment
- `roscore`
- `catkin_make`
- `source devel/setup.bash`
- `roslaunch turtlebot3_gazebo turtlebot3_zigzag.launch`
## demonstration
A quick showcase of features.

[![](http://img.youtube.com/vi/p_DRAERbJ2g/0.jpg)](https://www.youtube.com/watch?v=p_DRAERbJ2g)
