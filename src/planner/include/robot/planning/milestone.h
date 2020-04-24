#ifndef ESCAPE_ROOM_MILESTONE_H
#define ESCAPE_ROOM_MILESTONE_H

#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

struct Milestone {
    const Vector2f position;
    const bool isOutsideObstacle;
    vector<Milestone> neighbours;

public:
    Milestone(float x, float y, bool isOutsideObstacle) : position(x, y), isOutsideObstacle(isOutsideObstacle) {}
};

#endif
