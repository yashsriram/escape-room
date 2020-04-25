#ifndef ESCAPE_ROOM_PROBABILISTIC_ROADMAP_H
#define ESCAPE_ROOM_PROBABILISTIC_ROADMAP_H

#include <random>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include "milestone.h"
#include "robot/sensing/configuration_space.h"

using namespace std;
using namespace ros;
using namespace Eigen;

struct ProbabilisticRoadmap {
    const Publisher &rviz;
    vector<Milestone> milestones;

    explicit ProbabilisticRoadmap(const Publisher &rviz,
                                  int num_milestones,
                                  const Vector2f& min_corner,
                                  const Vector2f& max_corner,
                                  float max_edge_len,
                                  const ConfigurationSpace& cs) : rviz(rviz) {
        random_device rd;
        mt19937 gen(rd());
        uniform_real_distribution<> x(min_corner[0], max_corner[0]);
        uniform_real_distribution<> y(min_corner[1], max_corner[1]);
        // Create milestones
        for (uint32_t i = 0; i < num_milestones; ++i) {
            milestones.push_back(Milestone(i, x(gen), y(gen)));
        }
        // Create links
        int num_links = 0;
        for (int i = 0; i < milestones.size() - 1; ++i) {
            for (int j = i + 1; j < milestones.size(); j++) {
                Milestone &v1 = milestones[i];
                Milestone &v2 = milestones[j];
                // Too far off
                if ((v1.position - v2.position).norm() > max_edge_len) {
                    continue;
                }
                // Intersects with obstacles
                bool does_intersect = cs.does_intersect(v1.position, v2.position);
                if (does_intersect) {
                    continue;
                }
                // Add link if all okay
                v1.add_neighbour(v2.id);
                v2.add_neighbour(v1.id);
                num_links++;
            }
        }
        cout << "# links = " << num_links << endl;
    }

    void add_milestone(Vector2f position, float edge_len, const ConfigurationSpace& cs) {
        // Get the id of new milestone
        int id = milestones.size();
        // Add milestone
        milestones.push_back(Milestone(id, position[0], position[1]));
        Milestone& new_milestone = milestones[milestones.size() - 1];
        // Add links to neighbours properly
        for (int i = 0; i < milestones.size() - 1; ++i) {
            Milestone& neighbour = milestones[i];
            // Too far off
            if ((new_milestone.position - neighbour.position).norm() > edge_len) {
                continue;
            }
            // Intersects with obstacles
            bool does_intersect = cs.does_intersect(new_milestone.position, neighbour.position);
            if (does_intersect) {
                continue;
            }
            // Add link if all okay
            new_milestone.add_neighbour(neighbour.id);
            neighbour.add_neighbour(new_milestone.id);
        }
    }

    void search(Vector2f start, Vector2f finish, float edge_len, const ConfigurationSpace& cs) {
        // Add these to PRM
        add_milestone(start, edge_len, cs);
        add_milestone(finish, edge_len, cs);
        // Do A*
        // Return path
    }

    void draw_milestones() {
        visualization_msgs::Marker milestone_markers;
        milestone_markers.header.frame_id = "/map";
        milestone_markers.ns = "milestones";
        milestone_markers.header.stamp = ros::Time::now();
        milestone_markers.id = 0;
        milestone_markers.action = visualization_msgs::Marker::ADD;
        milestone_markers.pose.orientation.w = 1.0;
        milestone_markers.type = visualization_msgs::Marker::POINTS;
        milestone_markers.scale.x = 0.02;
        milestone_markers.scale.y = 0.02;

        // Milestones
        for (const auto &milestone : milestones) {
            geometry_msgs::Point p;
            p.x = milestone.position[0];
            p.y = milestone.position[1];
            milestone_markers.points.push_back(p);
            std_msgs::ColorRGBA c;
            c.a = 1.0;
            c.r = milestone.color[0];
            c.g = milestone.color[1];
            c.b = milestone.color[2];
            milestone_markers.colors.push_back(c);
        }

        rviz.publish(milestone_markers);
    }

    void draw_links() {
        visualization_msgs::Marker link_markers;
        link_markers.header.frame_id = "/map";
        link_markers.ns = "links";
        link_markers.header.stamp = ros::Time::now();
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
                geometry_msgs::Point p1;
                p1.x = milestone.position[0];
                p1.y = milestone.position[1];
                link_markers.points.push_back(p1);
                geometry_msgs::Point p2;
                p2.x = milestones[neighbourId].position[0];
                p2.y = milestones[neighbourId].position[1];
                link_markers.points.push_back(p2);
            }
        }

        rviz.publish(link_markers);
    }
};


#endif
