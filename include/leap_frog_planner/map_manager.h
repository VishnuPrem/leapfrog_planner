//
// Created by vishnu on 3/15/21.
//

#ifndef LEAP_FROG_PLANNER_MAP_LOADER_H
#define LEAP_FROG_PLANNER_MAP_LOADER_H
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include "robot.h"

namespace LeapFrog {

class MapManager {

public:

    MapManager(ros::NodeHandle& n_h) {
        viz_pub = n_h.advertise<visualization_msgs::Marker>("planner_viz", 10);
        map_pub = n_h.advertise<nav_msgs::OccupancyGrid>("map_viz", 10);

        path_anim_counter = 0;
        path_anim_timer = ros::Time::now();

        world_x_min = -10;
        world_x_max = 10;
        world_y_min = -10;
        world_y_max = 10;

        resolution = 0.1;
        map_x_dim = (world_x_max - world_x_min) / (resolution * 1.0);
        map_y_dim = (world_y_max - world_y_min) / (resolution * 1.0);

        std::vector<std::vector<int> >  zeros_map (map_x_dim, std::vector<int>(map_y_dim));
        occupancy_map = zeros_map;

//        add_obstacle_to_map(-10,5,-3,-1);
//        add_obstacle_to_map(-5,8,4,6);

//        add_obstacle_to_map(-10,5,-4,-3);
        add_obstacle_to_map(-5,10,-4,-3);
        add_obstacle_to_map(-10,5,3,4);

//        add_obstacle_to_map(-1,1,-1,1);
//        add_obstacle_to_map(4,6,4,6);
//        add_obstacle_to_map(-6,-4,4,6);
//        add_obstacle_to_map(4,6,-6,-4);
//        add_obstacle_to_map(-6,-4,-6,-4);

        prepareOccupancyGridMsg();

        inflated_occupancy_map = occupancy_map;
        // TODO: inflate obstacles

    }

    bool isPointInCollision (float x, float y) {
        int x_map, y_map;
        worldToMap(x, y, x_map, y_map);
        if (x_map > map_x_dim-1 || y_map > map_y_dim -1) {
            return true;
        }
        if (inflated_occupancy_map[x_map][y_map] == 1) {
            return true;
        }
        return false;
    }

    void getMapRange(int& x_min_, int& x_max_, int& y_min_, int& y_max_) {
        x_min_ = world_x_min;
        x_max_ = world_x_max;
        y_min_ = world_y_min;
        y_max_ = world_y_max;
    }

    void publishTrees(std::vector<Node>& node_list, int start_node_idx = 0) {
        publishTree(node_list, start_node_idx, "tree_mean", -1);
        publishTree(node_list, start_node_idx, "tree_robot0", 0);
        publishTree(node_list, start_node_idx, "tree_robot1", 1);
    }

    void publishTree(std::vector<Node>& node_list, int start_node_idx, std::string tree_name, int robot_num) {
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

        addTree(line_list, node_list, start_node_idx, robot_num);
        viz_pub.publish(line_list);
    }

    void prepareVizualisation (std::vector<Node>& node_list, int goal_node_idx, int start_node_idx, Node& target_goal_node) {
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

        std_msgs::ColorRGBA color;
        color.a = 0.1;
        color.r = 1.0;
        start_goal_colors.push_back(color);
        start_goal_colors.push_back(color);
        color.r = 0.0;
        color.b = 1.0;
        start_goal_colors.push_back(color);
        start_goal_colors.push_back(color);

    }

    void publishRobotAnim () {
        if (best_path.empty()) {
            return;
        }

        ros::Duration timer_max(1);
        if (ros::Time::now() - path_anim_timer > timer_max) {
            points.points = start_goal_points;
            points.colors = start_goal_colors;
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
            path_anim_timer = ros::Time::now();
            viz_pub.publish(points);
        }
    }

    void publishMap () {
        map_pub.publish(occupancy_grid_msg);
    }

    bool isObstacleinTriangle (Position p1, Position p2, Position p3) {

        // generate x,y limits of points
        float x_min = std::min(std::min(p1.x, p2.x), p3.x);
        float y_min = std::min(std::min(p1.y, p2.y), p3.y);
        float x_max = std::max(std::max(p1.x, p2.x), p3.x);
        float y_max = std::max(std::max(p1.y, p2.y), p3.y);

        int x_min_map, y_min_map, x_max_map, y_max_map;
        worldToMap(x_min, y_min, x_min_map, y_min_map);
        worldToMap(x_max, y_max, x_max_map, y_max_map);

        int x_map, y_map;
        worldToMap(p1.x, p1.y,x_map, y_map);
        p1.x = x_map;
        p1.y = y_map;

        worldToMap(p2.x, p2.y, x_map, y_map);
        p2.x = x_map;
        p2.y = y_map;

        worldToMap(p3.x, p3.y, x_map, y_map);
        p3.x = x_map;
        p3.y = y_map;

        // generate list of pts inside triangle
        for (int x = x_min_map; x<=x_max_map; x++) {
            for (int y = y_min_map; y<=y_max_map; y++) {
                Position p0(x, y);
                if (isPointInTriangle(p0, p1, p2, p3)) {
                    if (inflated_occupancy_map[p0.x][p0.y] == 1) {
                        return true;
                    }
                }
            }
        }

        return false;
    }

private:

    bool isPointInTriangle (Position pt, Position v1, Position v2, Position v3)
    {
        float d1, d2, d3;
        bool has_neg, has_pos;

        d1 = sign(pt, v1, v2);
        d2 = sign(pt, v2, v3);
        d3 = sign(pt, v3, v1);

        has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        return !(has_neg && has_pos);
    }

    float sign (Position p1, Position p2, Position p3)
    {
        return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
    }

    void worldToMap (float x_world, float y_world, int &x_map, int &y_map) {
        x_map = (x_world - world_x_min) / (resolution * 1.0);
        y_map = (y_world - world_y_min) / (resolution * 1.0);
    }

    void mapToWorld (int x_map, int y_map, float& x_world, float& y_world) {
        x_world = x_map * resolution + world_x_min;
        y_world = y_map * resolution + world_y_min;
    }

    void add_obstacle_to_map (float w_x_min, float w_x_max, float w_y_min, float w_y_max) {
        int m_x_min, m_x_max, m_y_min, m_y_max;
        worldToMap(w_x_min, w_y_min, m_x_min, m_y_min);
        worldToMap(w_x_max, w_y_max, m_x_max, m_y_max);

        m_x_min = std::max(0, m_x_min);
        m_x_max = std::min(map_x_dim-1, m_x_max);
        m_y_min = std::max(0, m_y_min);
        m_y_max = std::min(map_y_dim-1, m_y_max);

        for (int x = m_x_min; x <= m_x_max; x++) {
            for (int y = m_y_min; y <= m_y_max; y++ ) {
                occupancy_map[x][y] = 1;
            }
        }
    }

    void prepareOccupancyGridMsg() {

        occupancy_grid_msg.header.frame_id = "map";
        occupancy_grid_msg.header.stamp = ros::Time::now();

        occupancy_grid_msg.info.resolution = resolution;
        occupancy_grid_msg.info.height = map_x_dim;
        occupancy_grid_msg.info.width = map_y_dim;
        geometry_msgs::Pose origin;
        origin.position.x = world_x_min;
        origin.position.y = world_y_min;
        occupancy_grid_msg.info.origin = origin;
        occupancy_grid_msg.info.map_load_time = ros::Time::now();

        std::vector<int>occupancy_grid_row_major;
//        for (int i=0; i<map_x_dim; i++) {
//            for (int j=0; j<map_y_dim; j++) {
//                occupancy_grid_msg.data.push_back(occupancy_map[i][j]*100);
//            }
//        }
        for (int j=0; j<map_y_dim; j++) {
            for (int i=0; i<map_x_dim; i++) {
                occupancy_grid_msg.data.push_back(occupancy_map[i][j]*100);
            }
        }
    }

    void print_map () {
        for (int i=0; i<map_x_dim; i++) {
            for (int j=0; j<map_y_dim; j++) {
                std::cout<<occupancy_map[i][j]<<' ';
            }
            std::cout<<"\n";
        }
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

    void addTree (visualization_msgs::Marker& line_list, std::vector<Node>& node_list, int parent_idx, int robot_num = -1) {
        for (int i=0; i < node_list[parent_idx].getChildrenIdx().size(); i++) {
            int child_idx = node_list[parent_idx].getChildrenIdx(i);
            addEdge(line_list, node_list[parent_idx], node_list[child_idx], robot_num);
            addTree(line_list, node_list, child_idx, robot_num);
        }
    }

    int world_x_min;
    int world_x_max;
    int world_y_min;
    int world_y_max;
    float resolution;
    int map_x_dim;
    int map_y_dim;

    std::vector<std::vector<int>> occupancy_map;
    std::vector<std::vector<int>> inflated_occupancy_map;

    ros::Publisher viz_pub;
    ros::Publisher map_pub;
    nav_msgs::OccupancyGrid occupancy_grid_msg;

    std::vector<Node> best_path;
    int path_anim_counter;
    ros::Time path_anim_timer;
    visualization_msgs::Marker points;
    std::vector<geometry_msgs::Point> start_goal_points;
    std::vector<std_msgs::ColorRGBA> start_goal_colors;

};

}
#endif //LEAP_FROG_PLANNER_MAP_LOADER_H
