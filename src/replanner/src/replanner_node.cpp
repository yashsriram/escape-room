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

DifferentialDriveAgent turtle(Vector2f(0, 0), PI / 4, 0.1, 5, 10);
Room observed_room;
ProbabilisticRoadmap prm(500, Vector2f(-4, -4), Vector2f(4, 4), 0.8);
ConfigurationSpace cs;

void on_laser_line_segments_detection(const laser_line_extraction::LineSegmentList& msg) {
    // Rotation matrix
    Matrix2f C;
    C << cos(turtle.orientation), -(sin(turtle.orientation)),
         sin(turtle.orientation), cos(turtle.orientation);
    // For each line segment
    observed_room.clear_walls();
    for (int i = 0; i < msg.line_segments.size(); ++i) {
        Vector2f start(msg.line_segments[i].start[0], msg.line_segments[i].start[1]);
        Vector2f end(msg.line_segments[i].end[0], msg.line_segments[i].end[1]);
        // Transform to global frame; G_P_L = G_P_R + C * R_P_L
        start = turtle.center + C * start;
        end = turtle.center + C * end;
        // Add wall to observed_room
        observed_room.add_wall(start, end);
    }
    // Cull links in PRM
    prm.cull_links(observed_room, cs);
}

int main(int argc, char **argv) {
    init(argc, argv, "planner");
    NodeHandle node_handle;
    Publisher rviz = node_handle.advertise<visualization_msgs::Marker>("/visualization_marker", 10000);
    Publisher gazebo = node_handle.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10000);
    Subscriber sub = node_handle.subscribe("/line_segments", 1000, on_laser_line_segments_detection);
    Rate rate(20);

    // vector<Vector2f> path = prm.bfs(Vector2f(turtle.center[0], turtle.center[1]), Vector2f(-5.0, 5.0), 1);
    // turtle.set_path(path);

    while (ros::ok()) {
        /* Sense */
        spinOnce();
        /* Plan */

        /* Update */
        // for (int i = 0; i < 10; ++i) {
        //     turtle.update(0.001);
        // }
        
        /* Draw */
        observed_room.draw(rviz);
        prm.draw_links(rviz);
        prm.draw_milestones(rviz);
        cs.draw(rviz);
        // turtle.draw_rviz();
        // turtle.draw_path();
        turtle.draw_gazebo(gazebo);

        /* Sleep */
        rate.sleep();
    }
}