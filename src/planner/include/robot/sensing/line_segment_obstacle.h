#ifndef ESCAPE_ROOM_LINESEGMENTOBSTACLE_H
#define ESCAPE_ROOM_LINESEGMENTOBSTACLE_H

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>

using namespace Eigen;
using namespace std;
using namespace ros;

struct LineSegmentObstacle {
    const Vector2f point1;
    const Vector2f point2;

    explicit LineSegmentObstacle(Vector2f point1, Vector2f point2): point1(point1), point2(point2) {}
};

#endif
