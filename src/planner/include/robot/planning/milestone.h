#ifndef ESCAPE_ROOM_MILESTONE_H
#define ESCAPE_ROOM_MILESTONE_H

#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

struct Milestone {
    const int id;
    const Vector2f position;
    vector<int> neighbourIds;

public:
    Milestone(int id, float x, float y) : id(id), position(x, y) {}
};

#endif
