#ifndef ESCAPE_ROOM_MILESTONE_H
#define ESCAPE_ROOM_MILESTONE_H

#include <eigen3/Eigen/Dense>
#include <vector>

using namespace std;
using namespace Eigen;

struct Milestone {
    const int id;
    bool is_inside_obstacle = false;
    int num_hits = 0;
    const Vector2f position;
    vector<int> neighbourIds;

    Milestone(int id, float x, float y) : id(id), position(x, y) {
        reset_search_state();
    }

    void add_neighbour(int id) {
        neighbourIds.push_back(id);
    }

    void remove_neighbour(int id) {
        for (int i = 0; i < neighbourIds.size(); ++i) {
            if (neighbourIds[i] == id) {
                neighbourIds.erase(neighbourIds.begin() + i);
                break;
            }
        }
    }

    bool is_explored = false;
	float distance_from_start = 0;
	float distance_to_finish = 0;
    vector<int> path_from_start;
    Vector3f color;

    void reset_search_state() {
	    is_explored = false;
		distance_from_start = 0;
		distance_to_finish = 0;
	    path_from_start.clear();
	    color = Vector3f(1.0, 1.0, 1.0);
    }
};

#endif
