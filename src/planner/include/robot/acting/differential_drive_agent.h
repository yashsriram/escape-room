#ifndef ESCAPE_ROOM_DIFFERENTIALDRIVE_H
#define ESCAPE_ROOM_DIFFERENTIALDRIVE_H

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include "robot/sensing/configuration_space.h"

using namespace std;
using namespace ros;
using namespace Eigen;

struct DifferentialDriveAgent {
    const float MILESTONE_SLACK = 0.01f;
    const float ORIENTATION_SLACK = 0.01f;
    const Publisher& rviz;
    const Publisher& gazebo;
    Vector2f center;
    float orientation;
    float radius;
    vector<Vector2f> path;
    int current_milestone = 0;
    float linear_speed;
    float angular_speed;

    DifferentialDriveAgent(const Publisher& rviz,
                           const Publisher& gazebo,
                           Vector2f center,
                           float orientation,
                           float radius,
                           float linear_speed,
                           float angular_speed): 
        rviz(rviz), gazebo(gazebo), center(center), orientation(orientation), radius(radius),
            linear_speed(linear_speed), angular_speed(angular_speed) {
        path.push_back(center);
    }

    void set_path(vector<Vector2f> new_path) {
        path = new_path;
    }

    void update(float dt, const ConfigurationSpace& cs) {
        if (current_milestone < path.size() - 1) {
            Vector2f to_goal = path[current_milestone + 1] - center;
            float goal_orientation = atan2(to_goal[1], to_goal[0]);
            float to_orientation = goal_orientation - orientation;
            // Orient towards goal
            if (abs(to_orientation) > ORIENTATION_SLACK) {
                orientation += (to_orientation > 0) ? angular_speed * dt : -angular_speed * dt;
                return;
            }
            // Reached next milestone
            if (to_goal.norm() < MILESTONE_SLACK) {
                current_milestone++;
                return;
            }
            // Next next milestone lookup
            if (current_milestone < path.size() - 2) {
                bool blocked = cs.does_intersect(center, path[current_milestone + 2]);
                if (!blocked) {
                    current_milestone++;
                }
            }

            // Move towards next milestone
            Vector2f velocity_dir = to_goal.normalized();
            Vector2f displacement = velocity_dir * linear_speed * dt;
            center += displacement;
        }
    }    

    void draw_rviz() {
        // Position
        visualization_msgs::Marker center_marker;
        center_marker.header.frame_id = "/base_scan";
        center_marker.ns = "agent";
        center_marker.header.stamp = ros::Time::now();
        center_marker.id = 0;
        center_marker.type = visualization_msgs::Marker::CYLINDER;
        center_marker.action = visualization_msgs::Marker::ADD;
        center_marker.pose.position.x = center[0];
        center_marker.pose.position.y = center[1];
        center_marker.pose.orientation.w = 1.0;
        center_marker.scale.x = 2 * radius;
        center_marker.scale.y = 2 * radius;
        center_marker.scale.z = 0.01;
        center_marker.color.r = 1.0f;
        center_marker.color.g = 1.0f;
        center_marker.color.b = 1.0f;
        center_marker.color.a = 1.0;
        rviz.publish(center_marker);

        // Orientation
        visualization_msgs::Marker orientation_marker;
        orientation_marker.header.frame_id = "/base_scan";
        orientation_marker.ns = "agent";
        orientation_marker.header.stamp = ros::Time::now();
        orientation_marker.id = 1;
        orientation_marker.action = visualization_msgs::Marker::ADD;
        orientation_marker.pose.orientation.w = 1.0;
        orientation_marker.color.a = 1.0;
        orientation_marker.type = visualization_msgs::Marker::LINE_LIST;
        orientation_marker.scale.x = 0.01;
        orientation_marker.color.b = 1.0f;

        geometry_msgs::Point p1;
        p1.x = center[0];
        p1.y = center[1];
        p1.z = 0.2;
        orientation_marker.points.push_back(p1);
        geometry_msgs::Point p2;
        p2.x = center[0] + cos(orientation) * radius;
        p2.y = center[1] + sin(orientation) * radius;
        p2.z = 0.2;
        orientation_marker.points.push_back(p2);
        rviz.publish(orientation_marker);

        // Next center
        if (current_milestone < path.size() - 1) {
            Vector2f next_center = path[current_milestone + 1];
            visualization_msgs::Marker next_center_marker;
            next_center_marker.header.frame_id = "/base_scan";
            next_center_marker.ns = "agent";
            next_center_marker.header.stamp = ros::Time::now();
            next_center_marker.id = 2;
            next_center_marker.type = visualization_msgs::Marker::CYLINDER;
            next_center_marker.action = visualization_msgs::Marker::ADD;
            next_center_marker.pose.position.x = next_center[0];
            next_center_marker.pose.position.y = next_center[1];
            next_center_marker.pose.orientation.w = 1.0;
            next_center_marker.scale.x = 2 * radius;
            next_center_marker.scale.y = 2 * radius;
            next_center_marker.scale.z = 0.01;
            next_center_marker.color.r = 1.0f;
            next_center_marker.color.a = 1.0;
            rviz.publish(next_center_marker);
        }
    }

    void draw_gazebo() {
        gazebo_msgs::ModelState tutlebot_state;
        tutlebot_state.model_name = "turtlebot3_burger";
        tutlebot_state.pose.position.x = center[0];
        tutlebot_state.pose.position.y = center[1];
        tutlebot_state.pose.orientation.z = sin(orientation / 2);
        tutlebot_state.pose.orientation.w = cos(orientation / 2);
        gazebo.publish(tutlebot_state);
    }


    void draw_path() {
        visualization_msgs::Marker path_marker;
        path_marker.header.frame_id = "/base_scan";
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

};


#endif
