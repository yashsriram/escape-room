#ifndef ESCAPE_ROOM_PROBABILISTIC_ROADMAP_H
#define ESCAPE_ROOM_PROBABILISTIC_ROADMAP_H

#include <random>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include "milestone.h"

using namespace std;
using namespace ros;
using namespace Eigen;

struct ProbabilisticRoadmap {
    const Publisher &rviz;
    vector<Milestone> milestones;

    explicit ProbabilisticRoadmap(const Publisher &rviz, int num_milestones, Vector2f min_corner, Vector2f max_corner,
                                  float max_edge_len)
            : rviz(rviz) {
        random_device rd;
        mt19937 gen(rd());
        uniform_real_distribution<> x(min_corner[0], max_corner[0]);
        uniform_real_distribution<> y(min_corner[1], max_corner[1]);
        // Create milestones
        for (uint32_t i = 0; i < num_milestones; ++i) {
            milestones.emplace_back(Milestone((i + 1), x(gen), y(gen)));
        }
        // Create links
        int num_edges = 0;
        for (int i = 0; i < milestones.size() - 1; ++i) {
            for (int j = i + 1; j < milestones.size(); j++) {
                Milestone &v1 = milestones[i];
                Milestone &v2 = milestones[j];
                if ((v1.position - v2.position).norm() <= max_edge_len) {
                    v1.add_neighbour(v2.id);
                    v2.add_neighbour(v1.id);
                    num_edges++;
                }
            }
        }
        cout << "# edges = " << num_edges << endl;
    }

    void draw_milestones() {
        visualization_msgs::Marker milestone_markers;
        milestone_markers.header.frame_id = "/map";
        milestone_markers.header.stamp = ros::Time::now();
        milestone_markers.ns = "milestones";
        milestone_markers.id = 0;
        milestone_markers.action = visualization_msgs::Marker::ADD;
        milestone_markers.pose.orientation.w = 1.0;
        milestone_markers.color.a = 1.0;
        milestone_markers.type = visualization_msgs::Marker::POINTS;
        milestone_markers.scale.x = 0.01;
        milestone_markers.scale.y = 0.01;
        milestone_markers.color.r = milestone_markers.color.g = milestone_markers.color.b = 1.0f;

        // Milestones
        for (const auto &milestone : milestones) {
            geometry_msgs::Point p;
            p.x = milestone.position[0];
            p.y = milestone.position[1];
            milestone_markers.points.emplace_back(p);
        }

        rviz.publish(milestone_markers);
    }

    void draw_links() {
        visualization_msgs::Marker link_markers;
        link_markers.header.frame_id = "/map";
        link_markers.header.stamp = ros::Time::now();
        link_markers.ns = "links";
        link_markers.id = 0;
        link_markers.action = visualization_msgs::Marker::ADD;
        link_markers.pose.orientation.w = 1.0;
        link_markers.color.a = 1.0;
        link_markers.type = visualization_msgs::Marker::LINE_LIST;
        link_markers.scale.x = 0.01;
        link_markers.color.r = link_markers.color.g = link_markers.color.b = 1.0f;

        // Links
        for (const auto &milestone : milestones) {
            for (auto neighbourId: milestone.neighbourIds) {
                geometry_msgs::Point p;
                p.x = milestone.position[0];
                p.y = milestone.position[1];
                link_markers.points.emplace_back(p);
                p.x = milestones[neighbourId].position[0];
                p.y = milestones[neighbourId].position[1];
                link_markers.points.emplace_back(p);
            }
        }

        rviz.publish(link_markers);
    }
};


#endif
