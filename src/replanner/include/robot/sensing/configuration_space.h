#ifndef ESCAPE_ROOM_CONFIGURATIONSPACE_H
#define ESCAPE_ROOM_CONFIGURATIONSPACE_H

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include "robot/sensing/line_segment.h"
#include "robot/sensing/room.h"

using namespace std;
using namespace ros;
using namespace Eigen;

struct Rectangle {
	Vector2f center;
	Vector2f major_unit;
	float major_norm;
	Vector2f minor_unit;
	float minor_norm;

	Rectangle(Vector2f center, Vector2f major_unit, float major_norm, Vector2f minor_unit, float minor_norm):
		center(center), major_unit(major_unit), major_norm(major_norm), minor_unit(minor_unit), minor_norm(minor_norm) {}
};

struct ConfigurationSpace {
	vector<Rectangle> rectangles;
	vector<LineSegment> obstacles;
	explicit ConfigurationSpace() {}

	void reset(const Room& room, float clearance) {
		rectangles.clear();
		obstacles.clear();
		Matrix2f rot90CCW;
		rot90CCW <<	0, -1, 1, 0;
		// Add configuration space obstacles
		for (const auto wall : room.walls) {
			const Vector2f& obs1 = wall.point1;
			const Vector2f& obs2 = wall.point2;
			Vector2f wall_parallel = (obs2 - obs1).normalized();
			Vector2f wall_perpendicular = (rot90CCW * wall_parallel).normalized();
			// Rectangle
			Rectangle r(
				(obs1 + obs2) / 2,
				wall_parallel, (obs2 - obs1).norm() + clearance,
				wall_perpendicular, clearance
			);
			rectangles.push_back(r);
			// Obstacles
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
		for (const Rectangle& rect: rectangles) {
			Vector2f to_milestone = milestone - rect.center;
			float major_dist = to_milestone.dot(rect.major_unit);
			float minor_dist = to_milestone.dot(rect.minor_unit);
			if (abs(major_dist) < rect.major_norm
				&& abs(minor_dist) < rect.minor_norm) {
				return true;
			}
		}
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
        line.color.r = line.color.b = 1.0f;
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
