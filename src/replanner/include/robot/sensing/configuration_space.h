#ifndef ESCAPE_ROOM_CONFIGURATIONSPACE_H
#define ESCAPE_ROOM_CONFIGURATIONSPACE_H

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include "robot/sensing/line_segment.h"
#include "robot/sensing/room.h"

using namespace std;
using namespace ros;
using namespace Eigen;

struct ConfigurationSpace {
	vector<LineSegment> obstacles;
	explicit ConfigurationSpace() {}

	void reset_room(const Room& room, float clearance) {
		obstacles.clear();
		Matrix2f rot90CCW;
		rot90CCW <<	0, -1, 1, 0;
		// Add configuration space obstacles
		for (const auto wall : room.walls) {
			const Vector2f& obs1 = wall.point1;
			const Vector2f& obs2 = wall.point2;
			Vector2f wall_parallel = (obs2 - obs1).normalized();
			Vector2f wall_perpendicular = rot90CCW * wall_parallel;
			Vector2f top_right = obs2 + clearance * wall_parallel + clearance * wall_perpendicular;
			Vector2f bottom_right = obs2 + clearance * wall_parallel - clearance * wall_perpendicular;
			Vector2f top_left = obs1 - clearance * wall_parallel + clearance * wall_perpendicular;
			Vector2f bottom_left = obs1 - clearance * wall_parallel - clearance * wall_perpendicular;
			obstacles.push_back(LineSegment(top_left, top_right));
			obstacles.push_back(LineSegment(bottom_left, bottom_right));
			obstacles.push_back(LineSegment(top_left, bottom_left));
			obstacles.push_back(LineSegment(top_right, bottom_right));
		}
	}

	bool is_inside_obstacle(const Vector2f& milestone) const {
		return false;
	}

	void draw(const Publisher& rviz) {
		visualization_msgs::Marker line;
        line.header.frame_id = "/base_scan";
        line.ns = "cs_obstacles";
        line.header.stamp = ros::Time::now();
        line.id = 0;
        line.type = visualization_msgs::Marker::LINE_LIST;
        line.action = visualization_msgs::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x = 0.02;
        line.color.r = 1.0f;
        line.color.a = 1.0;

        for (int i = 0; i < obstacles.size(); ++i) {
            geometry_msgs::Point p;
            p.x = obstacles[i].point1[0];
            p.y = obstacles[i].point1[1];
            line.points.emplace_back(p);
            p.x = obstacles[i].point2[0];
            p.y = obstacles[i].point2[1];
            line.points.emplace_back(p);
        }

        rviz.publish(line);
	}
};


#endif
