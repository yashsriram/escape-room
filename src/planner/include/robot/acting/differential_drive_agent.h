#ifndef ESCAPE_ROOM_DIFFERENTIALDRIVE_H
#define ESCAPE_ROOM_DIFFERENTIALDRIVE_H

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>

using namespace std;
using namespace ros;
using namespace Eigen;

struct DifferentialDriveAgent {
	const Publisher& rviz;
	Vector2f position;
	float orientation;
	float radius;

	DifferentialDriveAgent(const Publisher& rviz, Vector2f position, float orientation, float radius): 
		rviz(rviz), position(position), orientation(orientation), radius(radius) {}

	void draw() {
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
};


#endif
