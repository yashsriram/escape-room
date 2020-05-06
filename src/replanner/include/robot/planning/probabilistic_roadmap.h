#ifndef ESCAPE_ROOM_PROBABILISTIC_ROADMAP_H
#define ESCAPE_ROOM_PROBABILISTIC_ROADMAP_H

#include <random>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include "milestone.h"
#include <queue>

using namespace std;
using namespace ros;
using namespace Eigen;

struct ProbabilisticRoadmap {
    vector<Milestone> milestones;
    int num_milestones_inside_obstacles = 0;

    explicit ProbabilisticRoadmap(int num_milestones,
                                  const Vector2f& min_corner,
                                  const Vector2f& max_corner,
                                  float max_edge_len) {
        random_device rd;
        mt19937 gen(rd());
        uniform_real_distribution<> x(min_corner[0], max_corner[0]);
        uniform_real_distribution<> y(min_corner[1], max_corner[1]);
        // Create milestones
        for (uint32_t i = 0; i < num_milestones; ++i) {
            milestones.push_back(Milestone(i, x(gen), y(gen)));
        }
        // Create edges
        int num_edges = 0;
        for (int i = 0; i < milestones.size() - 1; ++i) {
            for (int j = i + 1; j < milestones.size(); j++) {
                Milestone &v1 = milestones[i];
                Milestone &v2 = milestones[j];
                // Too far off
                if ((v1.position - v2.position).norm() > max_edge_len) {
                    continue;
                }
                // Add edge if all okay
                v1.add_neighbour(v2.id);
                v2.add_neighbour(v1.id);
                num_edges++;
            }
        }
        cout << "# edges = " << num_edges << endl;
    }

    int add_milestone(Vector2f position, float edge_len) {
        // Get the id of new milestone
        int id = milestones.size();
        // Add milestone
        milestones.push_back(Milestone(id, position[0], position[1]));
        Milestone& new_milestone = milestones[milestones.size() - 1];
        // Add edges to neighbours properly
        for (int i = 0; i < milestones.size() - 1; ++i) {
            Milestone& neighbour = milestones[i];
            // Too far off
            if ((new_milestone.position - neighbour.position).norm() > edge_len) {
                continue;
            }
            // If inside obstacle
            if (neighbour.is_inside_obstacle) {
                continue;
            }
            // Add edge if all okay
            new_milestone.add_neighbour(neighbour.id);
            neighbour.add_neighbour(new_milestone.id);
            break;
        }
        return id;
    }

    void reset_num_hits() {
        for (int i = 0; i < milestones.size(); ++i) {
            Milestone& milestone = milestones[i];
            if (!milestone.is_inside_obstacle) {
                milestone.num_hits = 0;
            }
        }
    }

    void cull_milestones(const Vector2f& center, const ConfigurationSpace& cs, int min_hit_for_inside_obstacle) {
        int num_culls = 0;
        for (int i = 0; i < milestones.size(); ++i) {
            Milestone& milestone = milestones[i];
            // If already inside obstacle no need to check
            if (milestone.is_inside_obstacle) {
                continue;
            }
            bool is_inside_obstacle = cs.is_inside_obstacle(milestone.position);
            if (is_inside_obstacle) {
                num_milestones_inside_obstacles++;
                milestone.num_hits++;
                if (milestone.num_hits >= min_hit_for_inside_obstacle) {
                    milestone.is_inside_obstacle = true;
                    for (const auto& neighbourId: milestone.neighbourIds) {
                        milestone.remove_neighbour(neighbourId);
                        milestones[neighbourId].remove_neighbour(milestone.id);
                    }
                    num_culls++;                    
                }
            }
        }
    }

    int cull_edges(const Vector2f& center, const ConfigurationSpace& cs) {
        int num_culls = 0;
        for (int i = 0; i < milestones.size() - 1; ++i) {
            Milestone &v1 = milestones[i];
            // Collect occlusions
            vector<int> occluded_neighbours;
            for (int j = 0; j < v1.neighbourIds.size(); ++j) {
                Milestone &v2 = milestones[v1.neighbourIds[j]];
                // If this edge intersects any observed wall cull it
                bool does_intersect = cs.does_intersect(v1.position, v2.position);
                if (does_intersect) {
                    occluded_neighbours.push_back(v2.id);
                    num_culls++;
                }
            }
            // Remove edges
            for (int occluded_neighbour_id: occluded_neighbours) {
                v1.remove_neighbour(occluded_neighbour_id);
                milestones[occluded_neighbour_id].remove_neighbour(v1.id);
            }
        }
        cout << num_culls << " culled\r" << endl;
        return num_culls;
    }

    void add_to_fringe(queue<int>& fringe, Milestone& current, Milestone& next) {
        next.distance_from_start = current.distance_from_start + (next.position - current.position).norm();
        next.is_explored = true;
        next.path_from_start = current.path_from_start;
        next.path_from_start.push_back(next.id);
        next.color = Vector3f(0, 1, 0);
        fringe.push(next.id);
    }

    vector<int> bfs(Vector2f start, Vector2f finish, float edge_len) {
        // Add these to PRM
        int start_id = add_milestone(start, edge_len);
        int finish_id = add_milestone(finish, edge_len);

        // Reset search state of all milestones
        for (auto& milestone: milestones) {
            milestone.reset_search_state();
        }

        // Do A* and return path
        int num_vertices_explored = 0;

        queue<int> fringe;
        // Add start to fringe
        add_to_fringe(fringe, milestones[start_id], milestones[start_id]);
        while (fringe.size() > 0) {
            // Pop one vertex
            int current_id = fringe.front();
            Milestone& current = milestones[current_id];
            fringe.pop();
            num_vertices_explored++;
            // Check if finish
            if (current_id == finish_id) {
                cout << "Reached finish, # vertices explored: " <<  num_vertices_explored << endl;
                cout << "path size: " << current.path_from_start.size() << endl;
                return current.path_from_start;
            }
            // Mark this vertex as explored
            current.color = Vector3f(1, 0, 0);
            // Update fringe
            for (int neighbourId : current.neighbourIds) {
                Milestone& neighbour = milestones[neighbourId];
                if (!neighbour.is_explored && !neighbour.is_inside_obstacle) {
                    add_to_fringe(fringe, current, neighbour);
                }
            }
        }

        cout << "Could not reach finish, # vertices explored: " << num_vertices_explored << endl;
        vector<int> path;
        path.push_back(start_id);
        return path;
    }

    void draw_milestones(const Publisher& rviz) {
        visualization_msgs::Marker milestone_markers;
        milestone_markers.header.frame_id = "/base_scan";
        milestone_markers.ns = "milestones";
        milestone_markers.header.stamp = ros::Time::now();
        milestone_markers.id = 0;
        milestone_markers.action = visualization_msgs::Marker::ADD;
        milestone_markers.pose.orientation.w = 1.0;
        milestone_markers.type = visualization_msgs::Marker::POINTS;
        milestone_markers.scale.x = 0.03;
        milestone_markers.scale.y = 0.03;

        // Milestones
        for (const auto &milestone : milestones) {
            std_msgs::ColorRGBA c;
            c.a = 1.0;
            if (milestone.is_inside_obstacle) {
                c.r = c.b = 1.0f;
                c.g = 0.0f;
            } else {
                c.r = milestone.color[0];
                c.g = milestone.color[1];
                c.b = milestone.color[2];
            }
            milestone_markers.colors.push_back(c);

            geometry_msgs::Point p;
            p.x = milestone.position[0];
            p.y = milestone.position[1];
            milestone_markers.points.push_back(p);
        }

        rviz.publish(milestone_markers);
    }

    void draw_edges(const Publisher& rviz) {
        visualization_msgs::Marker edge_markers;
        edge_markers.header.frame_id = "/base_scan";
        edge_markers.ns = "edges";
        edge_markers.header.stamp = ros::Time::now();
        edge_markers.id = 0;
        edge_markers.action = visualization_msgs::Marker::ADD;
        edge_markers.pose.orientation.w = 1.0;
        edge_markers.type = visualization_msgs::Marker::LINE_LIST;
        edge_markers.scale.x = 0.01;
        edge_markers.color.r = edge_markers.color.b = edge_markers.color.g = 1.0f;
        edge_markers.color.a = 0.2;

        // edges
        for (const auto &milestone : milestones) {
            for (auto neighbourId: milestone.neighbourIds) {
                geometry_msgs::Point p1;
                p1.x = milestone.position[0];
                p1.y = milestone.position[1];
                edge_markers.points.push_back(p1);
                geometry_msgs::Point p2;
                p2.x = milestones[neighbourId].position[0];
                p2.y = milestones[neighbourId].position[1];
                edge_markers.points.push_back(p2);
            }
        }

        rviz.publish(edge_markers);
    }
};


#endif
