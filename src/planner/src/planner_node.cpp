#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <random>
#include "robot/planning/milestone.h"
#include "robot/planning/probabilistic_roadmap.h"
#include "robot/sensing/line_segment_obstacle.h"

using namespace std;
using namespace ros;
using namespace Eigen;

int main(int argc, char **argv) {
    init(argc, argv, "planner");
    NodeHandle node_handle;
    Publisher rviz = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    Rate rate(10);

    ProbabilisticRoadmap prm(rviz, 1000, Vector2f(-5, -5), Vector2f(5, 5));
    LineSegmentObstacle obstacle(rviz, Vector2f(0, 0), Vector2f(5, 5));

    while (ros::ok()) {
        /* Update */

        /* Draw */
        prm.draw();
        obstacle.draw();

        /* Sleep */
        rate.sleep();
    }
}