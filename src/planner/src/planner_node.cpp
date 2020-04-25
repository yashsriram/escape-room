#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "robot/sensing/room.h"
#include "robot/sensing/configuration_space.h"
#include "robot/planning/milestone.h"
#include "robot/planning/probabilistic_roadmap.h"
#include "robot/acting/differential_drive_agent.h"

using namespace std;
using namespace ros;
using namespace Eigen;

int main(int argc, char **argv) {
    init(argc, argv, "planner");
    NodeHandle node_handle;
    Publisher rviz = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 10000);
    Rate rate(10);

    Room room(rviz);
    room.add_wall(Vector2f(3, 3), Vector2f(3, -3));
    room.add_wall(Vector2f(3, 3), Vector2f(-3, 3));
    room.add_wall(Vector2f(-3, 3), Vector2f(-3, -3));
    room.add_wall(Vector2f(-3, -3), Vector2f(0, -3));

    DifferentialDriveAgent turtle(rviz, Vector2f(0.0, 0.0), 0.0, 0.4);

    ConfigurationSpace cs(rviz, room, turtle.radius + 0.1);

    ProbabilisticRoadmap prm(rviz, 1000, Vector2f(-5, -5), Vector2f(5, 5), 0.5, cs);
    while (ros::ok()) {
        /* Update */

        /* Draw */
        prm.draw_links();
        prm.draw_milestones();
        room.draw();
        cs.draw();
        turtle.draw();

        /* Sleep */
        rate.sleep();
    }
}