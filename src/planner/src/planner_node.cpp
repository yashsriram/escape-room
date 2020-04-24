#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <random>
#include "robot/planning/milestone.h"
#include "robot/planning/probabilistic_roadmap.h"

using namespace std;
using namespace ros;

int main(int argc, char **argv) {
    init(argc, argv, "points_and_lines");
    NodeHandle node_handle;
    Publisher rviz = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    Rate rate(10);

    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis(-5, 5);

    ProbabilisticRoadmap prm(rviz);
    // Create the vertices for the points and lines
    for (uint32_t i = 0; i < 1000; ++i) {
        prm.milestones.emplace_back(Milestone(dis(gen), dis(gen), true));
    }

    while (ros::ok()) {
        /* Update */

        /* Draw */
        prm.draw();

        /* Sleep */
        rate.sleep();
    }
}