//
// Created by vishnu on 4/25/21.
//

#ifndef LEAP_FROG_PLANNER_CONSTANTS_H
#define LEAP_FROG_PLANNER_CONSTANTS_H
#include <string.h>
#include <array>

namespace LeapFrog {
    
    // Topics
    const std::string SIM_START_TOPIC = "/initialpose";
    const std::string PLANNER_CURRENT_TOPIC = "/robot_marker";
    const std::string PLANNER_GOAL_TOPIC = "/move_base_simple/goal";
    const std::string MAP_TOPIC = "/map";
    const std::string PLANNER_VISUALIZATION_TOPIC = "/planner_viz";

    const std::string ROBOT0_GOAL_TOPIC = "/robot0/goal";
    const std::string ROBOT0_COMPLETE_TOPIC = "/robot0/complete";
    const std::string ROBOT1_GOAL_TOPIC = "/robot1/goal";
    const std::string ROBOT1_COMPLETE_TOPIC = "/robot1/complete";
    const std::string ROBOT_MARKER_TOPIC = "/robot_marker";

    // Planner params
    const int NUM_PLANNING_ITERATIONS = 2000;
    const float STEER_DIST = 6;
    const float NEIGHBOUR_RADIUS = 4;
    const float ROLE_CHANGE_COST = 5;
    const float GOAL_RADIUS = 4;
    const float MAX_INTERROBOT_DIST = 10;
    const float MIN_INTERROBOT_DIST = 1;
    const bool RRT_STAR = true;
    const bool SAMPLE_FULL_MAP = false;
    const float OBSTACLE_INFLATION_DIST = 0.5;
    const std::array<float,2> INTERROBOT_START_OFFSET = {2.0 ,2.0};
}

#endif //LEAP_FROG_PLANNER_CONSTANTS_H
