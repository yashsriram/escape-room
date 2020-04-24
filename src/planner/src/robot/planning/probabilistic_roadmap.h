#ifndef ESCAPE_ROOM_PROBABILISTIC_ROADMAP_H
#define ESCAPE_ROOM_PROBABILISTIC_ROADMAP_H

#include <vector>
#include "milestone.h"

using namespace std;
using namespace ros;

struct ProbabilisticRoadmap {
    const Publisher& rviz;
    vector<Milestone> milestones;

    explicit ProbabilisticRoadmap(const Publisher& rviz): rviz(rviz) {}

    void draw() {
        visualization_msgs::Marker points;
        points.header.frame_id = "/map";
        points.header.stamp = ros::Time::now();
        points.ns = "milestones";
        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.02;
        points.scale.y = 0.02;
        // Points are green
        points.color.r = points.color.g = points.color.b = 1.0f;
        points.color.a = 1.0;

        for (int i = 0; i < milestones.size(); ++i) {
            geometry_msgs::Point p;
            p.x = milestones[i].position[0];
            p.y = milestones[i].position[1];
            points.points.emplace_back(p);
        }

        rviz.publish(points);
    }
};


#endif
