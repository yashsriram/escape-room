#include <math.h>
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
const float MAX_EDGE_LEN = 0.45;
const Vector2f MAX_CORNER = Vector2f(-6, -6);
const Vector2f MIN_CORNER = Vector2f(2, 2);

DifferentialDriveAgent turtle(Vector2f(0, 0), 0, 0.1, 5, 10);
Room observed_room;
ConfigurationSpace cs;
ProbabilisticRoadmap prm(2500, MIN_CORNER, MAX_CORNER, MAX_EDGE_LEN);

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
        if (isnan(start[0]) || isnan(start[1]) || isnan(end[0]) || isnan(end[1])) {
            continue;
        }
        // Transform to global frame; G_P_L = G_P_R + C * R_P_L
        start = turtle.center + C * start;
        end = turtle.center + C * end;
        // Add wall to observed_room
        observed_room.add_wall(start, end);
    }
    first_callback_in_queue = false;
}

int main(int argc, char **argv) {
    init(argc, argv, "planner");
    NodeHandle node_handle;
    Publisher rviz = node_handle.advertise<visualization_msgs::Marker>("/visualization_marker", 10000);
    Publisher gazebo = node_handle.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10000);
    Subscriber sub = node_handle.subscribe("/line_segments", 1000, on_laser_line_segments_detection);
    Rate rate(10);

    vector<int> path_ids = prm.bfs(Vector2f(turtle.center), MAX_CORNER, MAX_EDGE_LEN);

    // Initial path
    vector<Vector2f> path;
    for (int i = 0; i < path_ids.size(); ++i) {
        path.push_back(prm.milestones[path_ids[i]].position);
    }
    turtle.set_path(path);

    int prev_milestone = -6;
    int prev_num_milestones_inside_obstacles = 0;
    while (ros::ok()) {
        if (turtle.current_milestone > prev_milestone + 5) {
            cout << "Found new milestone: Sensing" << endl;
            // Sense for some time
            for (int k = 0; k < 20; ++k) {
                if (!ros::ok()) {
                    break;
                }
                // Sense and update world info
                first_callback_in_queue = true;
                spinOnce();
                // Reset config space with current observations
                cs.reset(observed_room, turtle.radius + 0.01);
                // Cull edges in PRM
                cout << "Culling milestones" << endl;
                prm.cull_milestones(turtle.center, cs, 18);
                // Draw
                observed_room.draw(rviz);
                prm.draw_edges(rviz);
                prm.draw_milestones(rviz);
                cs.draw(rviz);
                turtle.draw_rviz(rviz);
                turtle.draw_path(rviz);
                turtle.draw_gazebo(gazebo);
                // Sleep
                rate.sleep();
            }
            if (prm.num_milestones_inside_obstacles > prev_num_milestones_inside_obstacles) {
                cout << (prm.num_milestones_inside_obstacles - prev_num_milestones_inside_obstacles) << " New in obstacle milestones detected; Culling edges" << endl;
                prm.cull_edges(turtle.center, cs);
                prev_num_milestones_inside_obstacles = prm.num_milestones_inside_obstacles;
            }
            prev_milestone = turtle.current_milestone;
        } else {
            // Replan if required
            bool replan = false;
            for (int i = 0; i < path_ids.size(); ++i) {
                if (prm.milestones[path_ids[i]].is_inside_obstacle) {
                    replan = true;
                    break;
                }
            }
            for (int i = turtle.current_milestone + 1; i < path_ids.size(); ++i) {
                bool does_intersect = cs.does_intersect(prm.milestones[path_ids[i]].position, prm.milestones[path_ids[i - 1]].position);
                if (does_intersect) {
                    replan = true;
                    Milestone& m1 = prm.milestones[path_ids[i]];
                    Milestone& m2 = prm.milestones[path_ids[i - 1]];
                    m1.remove_neighbour(m2.id);
                    m2.remove_neighbour(m1.id);
                    break;
                }
            }
            if (replan) {
                cout << "Replanning" << endl;
                path_ids = prm.bfs(Vector2f(turtle.center), MAX_CORNER, MAX_EDGE_LEN);
                vector<Vector2f> path;
                for (int i = 0; i < path_ids.size(); ++i) {
                    path.push_back(prm.milestones[path_ids[i]].position);
                }
                turtle.set_path(path);
                prev_milestone = 0;
            }
            cout << "Moving\r";
            // Act
            for (int i = 0; i < 100; ++i) {
                turtle.update(0.0001);
            }
        }
        // Draw
        observed_room.draw(rviz);
        prm.draw_edges(rviz);
        prm.draw_milestones(rviz);
        cs.draw(rviz);
        turtle.draw_rviz(rviz);
        turtle.draw_path(rviz);
        turtle.draw_gazebo(gazebo);
        // Sleep
        rate.sleep();
    }
}