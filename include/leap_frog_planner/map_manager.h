//
// Created by vishnu on 3/15/21.
//

#ifndef LEAP_FROG_PLANNER_MAP_LOADER_H
#define LEAP_FROG_PLANNER_MAP_LOADER_H
#include <nav_msgs/OccupancyGrid.h>
#include "robot.h"

namespace LeapFrog {

class MapManager {

public:

    MapManager(ros::NodeHandle& n_h) {

        map_pub = n_h.advertise<nav_msgs::OccupancyGrid>("map_viz", 10);

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
        add_obstacle_to_map(-5,6,-4,-3);
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

    int world_x_min;
    int world_x_max;
    int world_y_min;
    int world_y_max;
    float resolution;
    int map_x_dim;
    int map_y_dim;

    std::vector<std::vector<int>> occupancy_map;
    std::vector<std::vector<int>> inflated_occupancy_map;

    ros::Publisher map_pub;
    nav_msgs::OccupancyGrid occupancy_grid_msg;

};

}
#endif //LEAP_FROG_PLANNER_MAP_LOADER_H
