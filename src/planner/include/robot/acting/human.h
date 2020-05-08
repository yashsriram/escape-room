#ifndef ESCAPE_ROOM_HUMAN_H
#define ESCAPE_ROOM_HUMAN_H

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include "robot/sensing/line_segment.h"

using namespace std;
using namespace ros;
using namespace Eigen;

struct Human {
 	const float END_SLACK = 0.01f;
    const Publisher& rviz;
    const Publisher& gazebo;
    Vector2f center;
    float radius;
    float linear_speed;

    Vector2f end1;
    Vector2f end2;
    int next_end = 2;

    Human(const Publisher& rviz,
			const Publisher& gazebo,
			Vector2f center,
			Vector2f end2,
			float radius,
			float linear_speed): 
        rviz(rviz), gazebo(gazebo), center(center), end1(center), end2(end2),
        radius(radius), linear_speed(linear_speed) {}

    void update(float dt) {
        Vector2f to_goal;
        if (next_end == 2) {
            to_goal = end2 - center;
            // Reached next milestone
            if (to_goal.norm() < END_SLACK) {
                next_end = 1;
                return;
            }
        } else {
            to_goal = end1 - center;
            // Reached next milestone
            if (to_goal.norm() < END_SLACK) {
                next_end = 2;
                return;
            }        	
        }
        // Move towards next milestone
        Vector2f velocity_dir = to_goal.normalized();
        Vector2f displacement = velocity_dir * linear_speed * dt;
        center += displacement;
    }    

    void draw_rviz() {
        visualization_msgs::Marker center_marker;
        center_marker.header.frame_id = "/base_scan";
        center_marker.ns = "human";
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
        center_marker.color.g = 0.0f;
        center_marker.color.b = 1.0f;
        center_marker.color.a = 1.0;
        rviz.publish(center_marker);
    }

};

#endif
