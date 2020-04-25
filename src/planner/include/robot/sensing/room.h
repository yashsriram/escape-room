#ifndef ESCAPE_ROOM_ROOM_H
#define ESCAPE_ROOM_ROOM_H

#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include "robot/sensing/line_segment_obstacle.h"

using namespace ros;
using namespace Eigen;

struct Room {
    const Publisher &rviz;
    vector<LineSegmentObstacle> walls;

    explicit Room(const Publisher& rviz) : rviz(rviz) {}

    void add_wall(Vector2f point1, Vector2f point2) {
        walls.push_back(LineSegmentObstacle(point1, point2));
    }

    void draw() {
        visualization_msgs::Marker line;
        line.header.frame_id = "/map";
        line.ns = "obstacles";
        line.header.stamp = ros::Time::now();
        line.id = 0;
        line.type = visualization_msgs::Marker::LINE_LIST;
        line.action = visualization_msgs::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x = 0.02;
        line.color.r = 1.0f;
        line.color.a = 1.0;

        for (int i = 0; i < walls.size(); ++i) {
            geometry_msgs::Point p;
            p.x = walls[i].point1[0];
            p.y = walls[i].point1[1];
            line.points.emplace_back(p);
            p.x = walls[i].point2[0];
            p.y = walls[i].point2[1];
            line.points.emplace_back(p);
        }

        rviz.publish(line);
    }
};


#endif
