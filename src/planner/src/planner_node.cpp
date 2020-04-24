#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "robot/sensing/room.h"
#include "robot/sensing/configuration_space.h"
#include "robot/planning/milestone.h"
#include "robot/planning/probabilistic_roadmap.h"
#include "robot/sensing/line_segment_obstacle.h"

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
    room.add_wall(Vector2f(-3, -3), Vector2f(1, -3));

    ConfigurationSpace cs(room);

    ProbabilisticRoadmap prm(rviz, 1000, Vector2f(-5, -5), Vector2f(5, 5), 0.5, cs);
    // PRM does not change so drawing once is enough
    prm.draw_milestones();
    prm.draw_links();
    while (ros::ok()) {
        /* Update */

        /* Draw */
        room.draw();

        /* Sleep */
        rate.sleep();
    }
}