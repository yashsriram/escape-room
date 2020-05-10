#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <gazebo_msgs/ModelState.h>

#include "robot/sensing/room.h"
#include "robot/sensing/configuration_space.h"
#include "robot/planning/milestone.h"
#include "robot/planning/probabilistic_roadmap.h"
#include "robot/acting/differential_drive_agent.h"
#include "robot/acting/human.h"

using namespace std;
using namespace ros;
using namespace Eigen;

const float PI = 3.141592653589793;

int main(int argc, char **argv) {
    init(argc, argv, "planner");
    NodeHandle node_handle;
    Publisher rviz = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 10000);
    Publisher gazebo = node_handle.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10000);
    Rate rate(20);

    Room room(rviz);
    // Outer walls
    room.add_wall(Vector2f(3, 3), Vector2f(3, -3));
    room.add_wall(Vector2f(3, 3), Vector2f(-3, 3));
    room.add_wall(Vector2f(-3, 3), Vector2f(-3, -3));
    room.add_wall(Vector2f(-3, -3), Vector2f(1, -3));
    // Inner walls
    room.add_wall(Vector2f(-1, 1.5), Vector2f(3, 1.5));
    room.add_wall(Vector2f(-3, 0.0), Vector2f(1, 0.0));
    room.add_wall(Vector2f(-1, -1.5), Vector2f(3, -1.5));

    vector<Human> humans;
    humans.push_back(Human(rviz, gazebo, Vector2f(1.5, 0), Vector2f(2.5, 0), 0.3, 0.75));
    humans.push_back(Human(rviz, gazebo, Vector2f(1, -3.5), Vector2f(3, -3.5), 0.3, 1));

    DifferentialDriveAgent turtle(rviz, gazebo, Vector2f(2.0, 2.0), PI / 4, 0.2, 10, 20);

    ConfigurationSpace cs(rviz, room, turtle.radius + 0.1);

    ProbabilisticRoadmap prm(rviz, 2000, Vector2f(-5, -5), Vector2f(5, 5), 0.5, cs);
    vector<Vector2f> path = prm.bfs(Vector2f(turtle.center[0], turtle.center[1]), Vector2f(5.0, -5.0), 1, cs);
    turtle.set_path(path);

    while (ros::ok()) {
        /* Update */
        for (int i = 0; i < 10; ++i) {
            turtle.update(0.001, cs, humans);
            for (int j = 0; j < humans.size(); ++j){
                humans[j].update(0.001);
            }
        }
        
        /* Draw */
        prm.draw_edges();
        prm.draw_milestones();
        room.draw();
        for (int j = 0; j < humans.size(); ++j){
            humans[j].draw_rviz(j);
            humans[j].draw_gazebo(j);
        }
        cs.draw();
        turtle.draw_rviz();
        turtle.draw_path();
        turtle.draw_gazebo();

        /* Sleep */
        rate.sleep();
    }
}