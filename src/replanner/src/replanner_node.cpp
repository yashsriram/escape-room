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

DifferentialDriveAgent turtle(Vector2f(0, 0), 0, 0.1, 5, 5);
Room observed_room;
ProbabilisticRoadmap prm(500, Vector2f(-4, -4), Vector2f(4, 4), 0.8);
ConfigurationSpace cs;
int num_culls = 0;
bool first_callback_in_queue = true;

void on_laser_line_segments_detection(const laser_line_extraction::LineSegmentList& msg) {
    // Don't consider more than one scan per iteration
    if (!first_callback_in_queue) {
        return;
    }
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
    cs.reset_room(observed_room, turtle.radius + 0.01);
    // num_culls = prm.cull_links(turtle.center, cs);
    first_callback_in_queue = false;
    turtle.orientation += 0.01;
}

int main(int argc, char **argv) {
    init(argc, argv, "planner");
    NodeHandle node_handle;
    Publisher rviz = node_handle.advertise<visualization_msgs::Marker>("/visualization_marker", 10000);
    Publisher gazebo = node_handle.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10000);
    Subscriber sub = node_handle.subscribe("/line_segments", 1000, on_laser_line_segments_detection);
    Rate rate(10);
    vector<Vector2f> path = prm.bfs(Vector2f(turtle.center), Vector2f(-4.0, 4.0), 1, cs);
    turtle.set_path(path);

    while (ros::ok()) {
        // Sense and update world info
        first_callback_in_queue = true;
        spinOnce();
        // Plan
        if (num_culls > 10) {
            vector<Vector2f> path = prm.bfs(Vector2f(turtle.center), Vector2f(-4.0, 4.0), 0.4, cs);
            turtle.set_path(path);
            cout << "replanned" << endl;
        }
        num_culls = 0;
        // Act
        // for (int i = 0; i < 10; ++i) {
        //     turtle.update(0.001);
        // }
        // Draw
        observed_room.draw(rviz);
        prm.draw_links(rviz);
        prm.draw_milestones(rviz);
        cs.draw(rviz);
        turtle.draw_rviz(rviz);
        turtle.draw_path(rviz);
        turtle.draw_gazebo(gazebo);

        /* Sleep */
        rate.sleep();
    }
}