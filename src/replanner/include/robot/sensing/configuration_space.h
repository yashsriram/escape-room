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
				wall_parallel, ((obs2 - obs1) / 2).norm() + clearance,
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

	bool does_intersect(const Vector2f& end1, const Vector2f& end2) const {
		for (const LineSegment& wall : obstacles) {
			const Vector2f& obs1 = wall.point1;
			const Vector2f& obs2 = wall.point2;
			// A
			Matrix2f A;
			A <<
				end2[1] - end1[1], -(end2[0] - end1[0]),
				obs2[1] - obs1[1], -(obs2[0] - obs1[0]);
			// b
			Vector2f b;
			b <<
				end1[0] * end2[1] - end1[1] * end2[0],
				obs1[0] * obs2[1] - obs1[1] * obs2[0];
			if (abs(A.determinant()) < 1e-6) {
				Vector2f e1 = (end2 - obs1).normalized();
				Vector2f e2 = (obs2 - end1).normalized();
				if (1 - abs(e1.dot(e2)) < 1e-6) {
					// Coincident, we will just assume to be intersecting for simplicity
					return true;
				} else {
					// Parallel
					continue;
				}
			}
			// Ax = b
			Vector2f x = A.inverse()* b;

			// Is intersection b/w edge end points?
			float edge_length = (end2 - end1).norm();
			Vector2f edge_unit = (end2 - end1).normalized();
			Vector2f t_times_edge_unit = x - end1;

			float t1 = 0;
			int den = 0;
			if (abs(edge_unit[0]) > 1e-4) {
				t1 += t_times_edge_unit[0] / edge_unit[0];
				den++;
			}
			if (abs(edge_unit[1]) > 1e-4) {
				t1 += t_times_edge_unit[1] / edge_unit[1];
				den++;
			}
			t1 = t1 / den;
			if ( t1 < 0 || t1 > edge_length ) {
				continue;
			}

			// Is intersection b/w obs end points?
			float obs_length = (obs2 - obs1).norm();
			Vector2f obs_unit = (obs2 - obs1).normalized();
			Vector2f t_times_obs_unit = x - obs1;
			float t2 = 0;
			den = 0;
			if (abs(obs_unit[0]) > 1e-4) {
				t2 += t_times_obs_unit[0] / obs_unit[0];
				den++;
			}
			if (abs(obs_unit[1]) > 1e-4) {
				t2 += t_times_obs_unit[1] / obs_unit[1];
				den++;
			}
			t2 = t2 / den;
			if ( t2 < 0 || t2 > obs_length ) {
				continue;
			}

			return true;
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
