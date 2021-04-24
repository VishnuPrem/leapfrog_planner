//
// Created by vishnu on 3/15/21.
//

#ifndef LEAP_FROG_PLANNER_MAP_LOADER_H
#define LEAP_FROG_PLANNER_MAP_LOADER_H
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include "robot.h"

namespace LeapFrog {

class MapManager {

public:

    MapManager (ros::NodeHandle& n_h) {
        map_subscriber =  n_h.subscribe("/map", 1000, &MapManager::mapSubCallback, this);
        map_initialized = false;
    }

    void mapSubCallback (nav_msgs::OccupancyGrid msg) {
        map_initialized = true;
        occupancy_map = msg.data;
        resolution = msg.info.resolution;
        map_x_dim = msg.info.width;
        map_y_dim = msg.info.height;

        world_x_min = msg.info.origin.position.x;
        world_y_min = msg.info.origin.position.y;
        world_x_max = map_x_dim * resolution + world_x_min;
        world_y_max = map_y_dim * resolution + world_y_min;

        float inflate_diameter = 0.5;
        buildInflatedOccupancyGrid(inflate_diameter);

        ROS_DEBUG("Resolution: %f \nMap dims:(%i, %i) \nWorld dims [min,max] \tx: %f, %f\t y: %f %f",
                 resolution, map_x_dim, map_y_dim, world_x_min, world_x_max, world_y_min, world_y_max);
    }

    bool isMapInitialized() {
        return map_initialized;
    }

    bool isPointInCollision (float x, float y) {
        int x_map, y_map;
        worldToMap(x, y, x_map, y_map);
        if (x_map > map_x_dim-1 || y_map > map_y_dim -1) {
            return true;
        }
        return isCellInCollision(x_map, y_map);
    }

    void getMapRange(int& x_min_, int& x_max_, int& y_min_, int& y_max_) {
        x_min_ = world_x_min;
        x_max_ = world_x_max;
        y_min_ = world_y_min;
        y_max_ = world_y_max;
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
                    if (isCellInCollision(p0.x,p0.y)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

private:

    bool isCellInCollision (int x, int y) {
//        int occupancy = occupancy_map[y * map_x_dim + x];
//        if (occupancy > 50 || occupancy == -1) {
//            return true;
//        } else {
//            return false;
//        }
        return is_cell_occupied[y][x];
    }

    bool isPointInTriangle (Position pt, Position v1, Position v2, Position v3) {
        float d1, d2, d3;
        bool has_neg, has_pos;
        d1 = sign(pt, v1, v2);
        d2 = sign(pt, v2, v3);
        d3 = sign(pt, v3, v1);
        has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
        return !(has_neg && has_pos);
    }

    float sign (Position p1, Position p2, Position p3) {
        return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
    }

    void worldToMap (float x_world, float y_world, int &x_map, int &y_map) {
        x_map = (x_world - world_x_min) / (resolution * 1.0);
        y_map = (y_world - world_y_min) / (resolution * 1.0);
    }

//    void mapToWorld (int x_map, int y_map, float& x_world, float& y_world) {
//        x_world = x_map * resolution + world_x_min;
//        y_world = y_map * resolution + world_y_min;
//    }

    bool checkForInflate(int x, int y, int num_cell_inflate) {
        for (int i=-num_cell_inflate; i <= num_cell_inflate; i++) {
            for (int j=-num_cell_inflate; j <= num_cell_inflate; j++) {
                int near_x = x+i;
                int near_y = y+j;
                if (near_x >= 0 && near_x < map_x_dim && near_y >=0 && near_y <map_y_dim) {
                    signed char occupancy =  occupancy_map[near_y*map_x_dim + near_x];
                    if (occupancy > 50 || occupancy == -1) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    void buildInflatedOccupancyGrid (float inflate_diameter) {
        int num_cell_inflate = inflate_diameter/resolution;
        std::vector<std::vector<bool>> is_cell_occupied_(map_y_dim, std::vector<bool>(map_x_dim));
        for (int y=0; y<map_y_dim; y++) {
             for (int x=0; x<map_x_dim; x++) {
                 signed char occupancy =  occupancy_map[y*map_x_dim + x];
                 bool occupied = occupancy > 50 || occupancy == -1;
                 bool inflate = false;
                 if (!occupied) {
                     inflate = checkForInflate(x,y, num_cell_inflate);
                 }
                 is_cell_occupied_[y][x] = occupied || inflate;
            }
        }
        is_cell_occupied = is_cell_occupied_;
    }

    float world_x_min;
    float world_x_max;
    float world_y_min;
    float world_y_max;

    float resolution;
    int map_x_dim;
    int map_y_dim;

    std::vector<signed char> occupancy_map;
    std::vector<std::vector<bool>> is_cell_occupied;
    ros::Subscriber map_subscriber;
    bool map_initialized;

};

}
#endif //LEAP_FROG_PLANNER_MAP_LOADER_H
