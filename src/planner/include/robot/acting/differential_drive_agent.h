#ifndef ESCAPE_ROOM_DIFFERENTIALDRIVE_H
#define ESCAPE_ROOM_DIFFERENTIALDRIVE_H

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include "robot/sensing/configuration_space.h"

using namespace std;
using namespace ros;
using namespace Eigen;

struct DifferentialDriveAgent {
    const float MILESTONE_REACHED_RADIUS = 0.01f;
    const Publisher& rviz;
    const Publisher& gazebo;
    Vector2f position;
    float orientation;
    float radius;
    vector<Vector2f> path;
    int current_milestone = 0;
    float speed;

    DifferentialDriveAgent(const Publisher& rviz,
                           const Publisher& gazebo,
                           Vector2f position,
                           float orientation,
                           float radius,
                           float speed): 
        rviz(rviz), gazebo(gazebo), position(position), orientation(orientation), radius(radius), speed(speed) {
        path.push_back(position);
    }

    void set_path(vector<Vector2f> new_path) {
        path = new_path;
    }

    void update(float dt, const ConfigurationSpace& cs) {
        if (current_milestone < path.size() - 1) {
            // reached next milestone
            if ((path[current_milestone + 1] - position).norm() < MILESTONE_REACHED_RADIUS) {
                current_milestone++;
                return;
            }
            // move towards next milestone
            Vector2f velocity_dir = (path[current_milestone + 1] - position).normalized();
            Vector2f displacement = velocity_dir * speed * dt;
            position += displacement;
        }
    }    

    void draw_rviz() {
        visualization_msgs::Marker agent;
        agent.header.frame_id = "/map";
        agent.ns = "agent";
        agent.header.stamp = ros::Time::now();
        agent.id = 0;
        agent.type = visualization_msgs::Marker::CYLINDER;
        agent.action = visualization_msgs::Marker::ADD;
        agent.pose.position.x = position[0];
        agent.pose.position.y = position[1];
        agent.pose.orientation.w = 1.0;
        agent.scale.x = radius;
        agent.scale.y = radius;
        agent.scale.z = 0.01;
        agent.color.r = 1.0f;
        agent.color.g = 1.0f;
        agent.color.b = 1.0f;
        agent.color.a = 1.0;
        rviz.publish(agent);
    }

    void draw_gazebo() {
        gazebo_msgs::ModelState tutlebot_state;
        tutlebot_state.model_name = "turtlebot3_burger";
        tutlebot_state.pose.position.x = position[0];
        tutlebot_state.pose.position.y = position[1];
        tutlebot_state.pose.orientation.w = 1;
        gazebo.publish(tutlebot_state);
    }


    void draw_path() {
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

};


#endif
