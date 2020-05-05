#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <gazebo_msgs/ModelState.h>
#include "laser_line_extraction/LineSegmentList.h"

#include "robot/sensing/configuration_space.h"
#include "robot/planning/milestone.h"
#include "robot/planning/probabilistic_roadmap.h"
#include "robot/acting/differential_drive_agent.h"

using namespace std;
using namespace ros;
using namespace Eigen;

const float PI = 3.141592653589793;

DifferentialDriveAgent turtle(Vector2f(0, 0), PI / 4, 0.2, 5, 10);
ProbabilisticRoadmap prm(2000, Vector2f(-5, -5), Vector2f(5, 5), 0.5);
visualization_msgs::Marker observed_walls;

void on_laser_line_segments_detection(const laser_line_extraction::LineSegmentList& msg) {
    observed_walls.points.clear();
    Matrix2f C;
    C <<
        cos(turtle.orientation), -(sin(turtle.orientation)),
        sin(turtle.orientation), cos(turtle.orientation);
    for (int i = 0; i < msg.line_segments.size(); ++i) {
        Vector2f start(msg.line_segments[i].start[0], msg.line_segments[i].start[1]);
        Vector2f end(msg.line_segments[i].end[0], msg.line_segments[i].end[1]);
        start = turtle.center + C * start;
        end = turtle.center + C * end;

        geometry_msgs::Point p;
        p.x = start[0];
        p.y = start[1];
        observed_walls.points.emplace_back(p);
        p.x = end[0];
        p.y = end[1];
        observed_walls.points.emplace_back(p);
    }
}

int main(int argc, char **argv) {
    init(argc, argv, "planner");
    NodeHandle node_handle;
    Publisher rviz = node_handle.advertise<visualization_msgs::Marker>("/visualization_marker", 10000);
    Publisher gazebo = node_handle.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10000);
    Subscriber sub = node_handle.subscribe("/line_segments", 1000, on_laser_line_segments_detection);
    Rate rate(20);

    // // ConfigurationSpace cs(rviz, room, turtle.radius + 0.1);
    // vector<Vector2f> path = prm.bfs(Vector2f(turtle.center[0], turtle.center[1]), Vector2f(-5.0, 5.0), 1);
    // turtle.set_path(path);

    observed_walls.header.frame_id = "/base_scan";
    observed_walls.ns = "scratch";
    observed_walls.header.stamp = ros::Time::now();
    observed_walls.id = 0;
    observed_walls.type = visualization_msgs::Marker::LINE_LIST;
    observed_walls.action = visualization_msgs::Marker::ADD;
    observed_walls.pose.orientation.w = 1.0;
    observed_walls.scale.x = 0.02;
    observed_walls.color.r = 1.0f;
    observed_walls.color.a = 1.0;


    while (ros::ok()) {
        /* Sense */
        spinOnce();
        rviz.publish(observed_walls);
        /* Plan */
        /* Update */
        // for (int i = 0; i < 10; ++i) {
        //     turtle.update(0.001);
        // }
        
        /* Draw */
        prm.draw_links(rviz);
        prm.draw_milestones(rviz);
        // // cs.draw();
        // turtle.draw_rviz();
        // turtle.draw_path();
        turtle.draw_gazebo(gazebo);

        /* Sleep */
        rate.sleep();
    }
}