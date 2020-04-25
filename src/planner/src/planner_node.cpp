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

void draw_path(const Publisher& rviz, const vector<Vector2f>& path) {
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "/map";
    path_marker.ns = "path";
    path_marker.header.stamp = ros::Time::now();
    path_marker.id = 0;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.pose.orientation.w = 1.0;
    path_marker.color.a = 1.0;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.scale.x = 0.01;
    path_marker.color.r = path_marker.color.g = path_marker.color.b = 1.0f;

    // Links
    for (const auto &milestone : path) {
        geometry_msgs::Point p;
        p.x = milestone[0];
        p.y = milestone[1];
        path_marker.points.push_back(p);
    }

    rviz.publish(path_marker);
}

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

    DifferentialDriveAgent turtle(rviz, Vector2f(0.0, 0.0), 0.0, 0.4);

    ConfigurationSpace cs(rviz, room, turtle.radius + 0.1);

    ProbabilisticRoadmap prm(rviz, 1000, Vector2f(-5, -5), Vector2f(5, 5), 0.6, cs);
    vector<Vector2f> path = prm.bfs(Vector2f(0.0, 0.0), Vector2f(-5.0, 5.0), 1, cs);

    while (ros::ok()) {
        /* Update */

        /* Draw */
        prm.draw_links();
        prm.draw_milestones();
        room.draw();
        cs.draw();
        turtle.draw();
        draw_path(rviz, path);

        /* Sleep */
        rate.sleep();
    }
}