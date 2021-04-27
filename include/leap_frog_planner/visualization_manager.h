//
// Created by vishnu on 4/13/21.
//

#ifndef LEAP_FROG_PLANNER_VISUALIZATION_MANAGER_H
#define LEAP_FROG_PLANNER_VISUALIZATION_MANAGER_H
#include <visualization_msgs/Marker.h>
#include "node.h"

namespace LeapFrog {

class VisualizationManager {

public:
    VisualizationManager(ros::NodeHandle& n_h) {
        viz_pub = n_h.advertise<visualization_msgs::Marker>(PLANNER_VISUALIZATION_TOPIC, 10);
        path_anim_counter = 0;
        path_anim_timer = ros::Time::now();
        viz_initialized = false;
    }

    void prepareVizualisation (std::vector<Node>& node_list_, int goal_node_idx, int start_node_idx, Node& target_goal_node) {
        node_list = node_list_;
        int n_idx = goal_node_idx;
        while(n_idx != -1) {
            best_path.insert(best_path.begin(), node_list[n_idx]);
            n_idx = node_list[n_idx].getParent();
        }
        path_anim_counter = 0;
        path_anim_timer = ros::Time::now();

        points.header.frame_id = "/map";
        points.header.stamp = ros::Time::now();
        points.ns = "robot_pos";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.5;
        points.scale.y = 0.5;
        points.lifetime = ros::Duration(2);

        start_goal_points.clear();
        geometry_msgs::Point p;
        p.z = 0;
        p.x = node_list[start_node_idx].X(0);
        p.y = node_list[start_node_idx].Y(0);
        start_goal_points.push_back(p);
        p.x = target_goal_node.X(0);
        p.y = target_goal_node.Y(0);
        start_goal_points.push_back(p);
        p.x = node_list[start_node_idx].X(1);
        p.y = node_list[start_node_idx].Y(1);
        start_goal_points.push_back(p);
        p.x = target_goal_node.X(1);
        p.y = target_goal_node.Y(1);
        start_goal_points.push_back(p);

        start_goal_colors.clear();
        std_msgs::ColorRGBA color;
        color.a = 0.1;
        color.r = 1.0;
        start_goal_colors.push_back(color);
        start_goal_colors.push_back(color);
        color.r = 0.0;
        color.b = 1.0;
        start_goal_colors.push_back(color);
        start_goal_colors.push_back(color);

        viz_initialized = true;
    }

    void resetVizualisation() {
        best_path.clear();
        path_anim_counter = 0;
        path_anim_timer = ros::Time::now();
        viz_initialized = false;
    }

    void publishRobotAnim () {
        if (!viz_initialized) {
            return;
        }
        ros::Duration timer_max(1);
        if (ros::Time::now() - path_anim_timer > timer_max) {
            points.points = start_goal_points;
            points.colors = start_goal_colors;

            if (!best_path.empty()) {
                geometry_msgs::Point p;
                p.x = best_path[path_anim_counter].X(0);
                p.y = best_path[path_anim_counter].Y(0);
                p.z = 0;
                points.points.push_back(p);

                std_msgs::ColorRGBA color;
                color.r = 1.0;
                color.a = 1.0;
                points.colors.push_back(color);

                p.x = best_path[path_anim_counter].X(1);
                p.y = best_path[path_anim_counter].Y(1);
                p.z = 0;
                points.points.push_back(p);

                color.r = 0.0;
                color.b = 1.0;
                points.colors.push_back(color);

                path_anim_counter = (path_anim_counter+1) % best_path.size();
            }

            path_anim_timer = ros::Time::now();
            viz_pub.publish(points);
        }
    }

    void updateVizualization(std::vector<Node>& node_list_) {
        node_list = node_list_;
    }

    void publishTrees(int start_node_idx = 0) {
        if (node_list.empty()) {
            return;
        }
        publishTree(start_node_idx, "tree_mean", -1);
        publishTree(start_node_idx, "tree_robot0", 0);
        publishTree(start_node_idx, "tree_robot1", 1);
    }

private:

    void publishTree(int start_node_idx, std::string tree_name, int robot_num) {
        visualization_msgs::Marker line_list;
        line_list.header.frame_id = "/map";
        line_list.header.stamp = ros::Time::now();
        line_list.ns = tree_name;
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.id = 0;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.scale.x = 0.05;
        if (robot_num == 0) {
            line_list.color.r = 1.0;
        } else if (robot_num == 1) {
            line_list.color.b = 1.0;
        } else {
            line_list.color.g = 1.0;
        }
        line_list.color.a = 1.0;

        addTree(line_list, start_node_idx, robot_num);
        viz_pub.publish(line_list);
    }

    void addEdge (visualization_msgs::Marker& line_list, Node& parent, Node& child, int robot_num) {
        geometry_msgs::Point p;
        p.z = 0;

        if (robot_num == 0) {
            p.x = parent.X(0);
            p.y = parent.Y(0);
        } else if (robot_num == 1) {
            p.x = parent.X(1);
            p.y = parent.Y(1);
        } else {
            p.x = (parent.X(0) + parent.X(1))/2;
            p.y = (parent.Y(0) + parent.Y(1))/2;
        }
        line_list.points.push_back(p);

        if (robot_num == 0) {
            p.x = child.X(0);
            p.y = child.Y(0);
        } else if (robot_num == 1) {
            p.x = child.X(1);
            p.y = child.Y(1);
        } else {
            p.x = (child.X(0) + child.X(1))/2;
            p.y = (child.Y(0) + child.Y(1))/2;
        }
        line_list.points.push_back(p);

    }

    void addTree (visualization_msgs::Marker& line_list, int parent_idx, int robot_num = -1) {
        for (int i=0; i < node_list[parent_idx].getChildrenIdx().size(); i++) {
            int child_idx = node_list[parent_idx].getChildrenIdx(i);
            addEdge(line_list, node_list[parent_idx], node_list[child_idx], robot_num);
            addTree(line_list, child_idx, robot_num);
        }
    }

    ros::Publisher viz_pub;

    std::vector<Node> best_path;
    int path_anim_counter;
    ros::Time path_anim_timer;
    visualization_msgs::Marker points;
    std::vector<geometry_msgs::Point> start_goal_points;
    std::vector<std_msgs::ColorRGBA> start_goal_colors;

    std::vector<Node> node_list;
    bool viz_initialized;
};

}

#endif //LEAP_FROG_PLANNER_VISUALIZATION_MANAGER_H
