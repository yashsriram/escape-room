#ifndef ESCAPE_ROOM_LINESEGMENTOBSTACLE_H
#define ESCAPE_ROOM_LINESEGMENTOBSTACLE_H

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using namespace Eigen;
using namespace std;

struct LineSegmentObstacle {
    const Publisher &rviz;
    Vector2f point1;
    Vector2f point2;

    explicit LineSegmentObstacle(const Publisher &rviz, Vector2f point1, Vector2f point2)
            : rviz(rviz), point1(point1), point2(point2) {}

    void draw() {
        visualization_msgs::Marker line;
        line.header.frame_id = "/map";
        line.header.stamp = ros::Time::now();
        line.ns = "obstacles";
        line.id = 0;
        line.type = visualization_msgs::Marker::LINE_LIST;
        line.action = visualization_msgs::Marker::ADD;
        line.pose.orientation.w = 1.0;
        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line.scale.x = 0.02;
        // Points are green
        line.color.r = 1.0f;
        line.color.a = 1.0;

        geometry_msgs::Point p;
        p.x = point1[0];
        p.y = point1[1];
        line.points.emplace_back(p);
        p.x = point2[0];
        p.y = point2[1];
        line.points.emplace_back(p);

        rviz.publish(line);
    }
};

#endif
