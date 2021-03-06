//
// Created by vishnu on 3/15/21.
//

#ifndef LEAP_FROG_PLANNER_PLANNER_H
#define LEAP_FROG_PLANNER_PLANNER_H
#include "node.h"
#include <utility>
#include <stdlib.h>
#include <cmath>
#include "map_manager.h"
#include <limits>
#include "visualization_manager.h"
#include <geometry_msgs/PoseStamped.h>
#include "constants.h"

namespace LeapFrog {

class Planner {

public:

    Planner(ros::NodeHandle& n_h) {
        current_pos_subscriber =  n_h.subscribe(PLANNER_CURRENT_TOPIC, 1000, &Planner::currentPosCallback, this);
        goal_pos_subscriber =  n_h.subscribe(PLANNER_GOAL_TOPIC, 1000, &Planner::goalPosCallback, this);
    }

    std::vector<Node> Plan(MapManager& Map, VisualizationManager& viz) {

        std::vector<Node> best_path;
        if(!start_initialized || !goal_initialized) {
            return best_path;
        } else if (!Map.isMapInitialized()) {
            ROS_ERROR("Map not received");
            return best_path;
        }
        ROS_DEBUG("Start planner");
        ros::Time planner_start_time = ros::Time::now();
        resetPlanner(viz);

        int k;
        for(k=0; k<NUM_PLANNING_ITERATIONS; k++) {

            ROS_INFO("Iteration: %i Curr dist to goal: %f", k, curr_dist_to_goal);
            int robot_num;
            Node n_new;
            n_new = generateSample(robot_num, Map);
            ROS_DEBUG("\t1 . Sample: %s", n_new.getNodeInfo().c_str());

            int n_nearest_idx = getNearestNode(robot_num, n_new);
            ROS_DEBUG("\t2a. Nearest: %s", node_list[n_nearest_idx].getNodeInfo().c_str());

            steerSample(robot_num, n_new, node_list[n_nearest_idx]);
            ROS_DEBUG("\t2b. Steered: %s", n_new.getNodeInfo().c_str());

            if (isEdgeValid(robot_num, node_list[n_nearest_idx], n_new, Map)) {
                ROS_DEBUG("\t3 . Valid edge");

                // find neighbours
                std::vector<int> neighbours_idx;
                getNeighbours(robot_num, n_new, neighbours_idx);
                int n_min_idx = n_nearest_idx;
                float min_n_new_cost = node_list[n_nearest_idx].cost(robot_num) + getEdgeCost(robot_num, node_list[n_nearest_idx], n_new);

                ROS_DEBUG("\t4 . Near nodes");
                // find lowest cost neighbour
                for (int near_idx: neighbours_idx) {
                    ROS_DEBUG("\t\t4a. Near: %s", node_list[near_idx].getNodeInfo().c_str());
                    if (isEdgeValid(robot_num, node_list[near_idx], n_new, Map)) {
                        ROS_DEBUG("\t\tValid edge");
                        float n_new_cost = node_list[near_idx].cost(robot_num) + getEdgeCost(robot_num, node_list[near_idx], n_new);
                        if (n_new_cost < min_n_new_cost) {
                            ROS_DEBUG("\t\tLowest cost");
                            min_n_new_cost = n_new_cost;
                            n_min_idx = near_idx;
                        }
                    }
                }

                // connect new to cheapest neighbour
                node_list.push_back(n_new);

                addNewNodeToTree(robot_num, node_list.size()-1, n_min_idx, min_n_new_cost);

                ROS_DEBUG("\t5 . Connected: %s", node_list.back().getNodeInfo().c_str());
                ROS_DEBUG("\t\t To: %s", node_list[n_min_idx].getNodeInfo().c_str());

                if (RRT_STAR) {
                    rewireTree(node_list.size()-1, Map);
                }

                assignGoal();
            }

            // check for termination
            if (!RRT_STAR && goal_found) {
                break;
            }
            if (!ros::ok()) {
                break;
            }

            if (k%100 == 0) {
                viz.updateVizualization(node_list);
                viz.publishTrees();
            }
        }
        ros::Duration planner_run_time = ros::Time::now() - planner_start_time;
        ROS_INFO("Iteration: %i", k);
        printPath();
        viz.prepareVizualisation(node_list, goal_node_idx, start_node_idx, target_goal_node);
        ROS_INFO("Run time: %f", planner_run_time.toSec());

        getBestPath(best_path);
        return best_path;
    }

private:

    void currentPosCallback(visualization_msgs::Marker msg) {
        int robot_num;
        if (msg.color.r == 1.0) {
            robot_num = 0;
        } else {
            robot_num = 1;
        }
        Position pos(msg.points[0].x, msg.points[0].y);
        current_pos_node.setPosition(robot_num, pos);
        start_initialized = true;
    }

    void goalPosCallback(geometry_msgs::PoseStamped msg) {
        ROS_INFO("Received goal: %f, %f",msg.pose.position.x, msg.pose.position.y);
        Position robot0_goal_pos(msg.pose.position.x, msg.pose.position.y);
        Position robot1_goal_pos(msg.pose.position.x + INTERROBOT_START_OFFSET[0], msg.pose.position.y + INTERROBOT_START_OFFSET[1]);
        target_goal_node.setPosition(0, robot0_goal_pos);
        target_goal_node.setPosition(1, robot1_goal_pos);
        start_node = current_pos_node;
        goal_initialized = true;
    }

    void resetPlanner(VisualizationManager& viz) {
        node_list.clear();
        start_node_idx = 0;
        node_list.push_back(start_node);

        goal_node_idx = -1;
        curr_dist_to_goal = std::numeric_limits<float>::max();
        goal_found = false;

        // start_initialized = false;
        goal_initialized = false;
        viz.resetVizualisation();
    }

    void getSampleRange (int& x_min, int& x_max, int& y_min, int& y_max, MapManager& Map) {
        if (SAMPLE_FULL_MAP) {
            Map.getMapRange(x_min, x_max, y_min, y_max);
        } else {
            x_min = std::min(std::min(start_node.X(0), start_node.X(1)), std::min(target_goal_node.X(0), target_goal_node.X(1))) - 10;
            x_max = std::max(std::max(start_node.X(0), start_node.X(1)), std::max(target_goal_node.X(0), target_goal_node.X(1))) + 10;
            y_min = std::min(std::min(start_node.Y(0), start_node.Y(1)), std::min(target_goal_node.Y(0), target_goal_node.Y(1))) - 10;
            y_max = std::max(std::max(start_node.Y(0), start_node.Y(1)), std::max(target_goal_node.Y(0), target_goal_node.Y(1))) + 10;
        }
    }

    Node generateSample(int &robot_num, MapManager& Map) {

        robot_num = rand() % 2;

        // generate float between min-max
        int x_min, x_max, y_min, y_max;
        getSampleRange(x_min, x_max, y_min, y_max, Map);
        float x = x_min + (((float) rand()) / (float) RAND_MAX) * (x_max - x_min);
        float y = y_min + (((float) rand()) / (float) RAND_MAX) * (y_max - y_min);
        ROS_DEBUG("\t\tRandom sample: %f %f", x,y);
        if (Map.isPointInCollision(x, y)) {
            Node new_n_sample;
            new_n_sample = generateSample(robot_num, Map);
            return new_n_sample;
        }
        Position moving_robot_pos(x,y);
        Robot moving_robot(moving_robot_pos, true);
        Position empty_pos;
        Robot stationary_robot(empty_pos, false);

        if (robot_num == 0) {
            Node n_sample(moving_robot, stationary_robot);
            return n_sample;
        } else {
            Node n_sample(stationary_robot, moving_robot);
            return n_sample;
        }
    }

    int getNearestNode(int robot_num, Node& n) {

        std::vector<int> nearest_idx_list;
        float min_dist = std::numeric_limits<float>::max();

        int fixed_robot_idx = (robot_num == 0)? 1 : 0;
        Position new_robot_pos = n.getPosition(robot_num);

        for (int i=0; i<node_list.size(); i++) {

            float dist = n.getDistance(robot_num, node_list[i]);
            Position fixed_robot_pos = node_list[i].getPosition(fixed_robot_idx);
            float fixed_dist = new_robot_pos.getDistance(fixed_robot_pos);
            if (fixed_dist > MAX_INTERROBOT_DIST || fixed_dist < MIN_INTERROBOT_DIST) {
                dist = std::numeric_limits<float>::max();
            }

            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx_list.clear();
                nearest_idx_list.push_back(i);
            } else if (dist == min_dist) {
                nearest_idx_list.push_back(i);
            }
        }

        int rand_choice = rand() % nearest_idx_list.size();

        return nearest_idx_list[rand_choice];
    }

    void getNeighbours(int robot_num, Node& n, std::vector<int>& neighbours) {

        Position new_robot_pos = n.getPosition(robot_num);
        int fixed_robot_idx = (robot_num == 0)? 1 : 0;

        for(int i=0; i<node_list.size(); i++) {
            Position fixed_robot_pos = node_list[i].getPosition(fixed_robot_idx);
            float fixed_dist = new_robot_pos.getDistance(fixed_robot_pos);
            if (fixed_dist > MAX_INTERROBOT_DIST || fixed_dist < MIN_INTERROBOT_DIST) {
                continue;
            }
            float dist = n.getDistance(robot_num, node_list[i]);
            if (dist <= NEIGHBOUR_RADIUS) {
                neighbours.push_back(i);
            }
        }
    }

    // steers n_new to n_old
    void steerSample(int robot_num, Node& n_new, Node& n_old) {
        float dist = n_new.getDistance(robot_num, n_old);
        if (dist > STEER_DIST) {
            float x_diff = STEER_DIST * (n_new.X(robot_num) - n_old.X(robot_num)) / dist;
            float y_diff = STEER_DIST * (n_new.Y(robot_num) - n_old.Y(robot_num)) / dist;
            float new_x = n_old.X(robot_num) + x_diff;
            float new_y = n_old.Y(robot_num) + y_diff;
            n_new.setX(robot_num, new_x);
            n_new.setY(robot_num, new_y);
        }
    }

    bool isEdgeValid(int robot_num, Node& n_old, Node& n_new, MapManager& Map) {
        /**
         * The robots should always be in line of sight and the path should be obstacle free.
         * If robot 1 at A is fixed and robot2 moves from B to B' check if:
         * 1. AB' is within range
         * 2. A and lineB are not too close
         * 3. Triangle ABB' is obstacle free
         */
         int stationary_robot_num = (robot_num == 0)? 1: 0;

         Position moving_robot_old_pos(n_old.X(robot_num), n_old.Y(robot_num));
         Position moving_robot_new_pos(n_new.X(robot_num), n_new.Y(robot_num));
         Position fixed_robot_pos(n_old.X(stationary_robot_num), n_old.Y(stationary_robot_num));

         // 1. AB' is within range
         float new_dist_bw_robots = fixed_robot_pos.getDistance(moving_robot_new_pos);
         if(new_dist_bw_robots > MAX_INTERROBOT_DIST || new_dist_bw_robots < MIN_INTERROBOT_DIST) {
             ROS_DEBUG("Inter robot distance %f outside range", new_dist_bw_robots);
             return false;
         }

         // 2. A and lineB are not too close
         float x1,y1,x2,y2,x0,y0;
         x0 = fixed_robot_pos.x;
         y0 = fixed_robot_pos.y;
         x1 = moving_robot_old_pos.x;
         y1 = moving_robot_old_pos.y;
         x2 = moving_robot_new_pos.x;
         y2 = moving_robot_new_pos.y;
         float min_dist_bw_fixed_and_moving = abs((x2-x1)*(y1-y0) - (y2-y1)*(x1-x0))/ moving_robot_old_pos.getDistance(moving_robot_new_pos);
         if (min_dist_bw_fixed_and_moving < MIN_INTERROBOT_DIST) {
             ROS_DEBUG("Inter robot distance %f too close during movement", min_dist_bw_fixed_and_moving);
             return false;
         }

         // 3. Triangle is obstacle free
         if (Map.isObstacleinTriangle(moving_robot_old_pos, moving_robot_new_pos, fixed_robot_pos)) {
             ROS_DEBUG("Obstacle in triangle");
             return false;
         }

         return true;
    }

    float getEdgeCost(int robot_num, Node& n_old, Node& n_new) {
        float dist = n_old.getDistance(robot_num, n_new);
        if (n_old.isInMotion(robot_num))
           return dist;
        else
            return dist + ROLE_CHANGE_COST;
    }

    void addNewNodeToTree(int robot_num, int child_idx, int parent_idx, float child_cost) {

        Node& child = node_list[child_idx];
        Node& parent = node_list[parent_idx];

        int stationary_robot_num = (robot_num == 0) ? 1 : 0;

        child.setX(stationary_robot_num, parent.X(stationary_robot_num));
        child.setY(stationary_robot_num, parent.Y(stationary_robot_num));

        child.setCost(stationary_robot_num, parent.cost(stationary_robot_num));
        child.setCost(robot_num, child_cost);

        connectNodes(parent_idx, child_idx);
    }

    void assignGoal () {
        ROS_DEBUG("ASSINGING GOAL");

        float min_dist = std::numeric_limits<float>::max();
        for (int i=0; i<node_list.size(); i++) {
            float dist_to_goal = (node_list[i].getDistance(0, target_goal_node) + node_list[i].getDistance(1, target_goal_node)) / 2;
            if (dist_to_goal < min_dist) {
                min_dist = dist_to_goal;
                if (dist_to_goal < GOAL_RADIUS) {
                    goal_found = true;
                    goal_node_idx = i;
                }
            }
        }
        curr_dist_to_goal = min_dist;

    }

    void getNeighboursByMeanPosition (Node& node, std::vector<int>& neighbour_list) {
        Position mean_node_pos((node.X(0)+node.X(1))/2, (node.Y(0)+node.Y(1))/2);
        for (int i=0; i<node_list.size(); i++) {
            Position mean_neighbour_pos((node_list[i].X(0)+node_list[i].X(1))/2, (node_list[i].Y(0)+node_list[i].Y(1))/2);
            float dist = mean_node_pos.getDistance(mean_neighbour_pos);
            if (dist < NEIGHBOUR_RADIUS && dist != 0) {
                neighbour_list.push_back(i);
            }
        }
    }

    void propogateCostChange (int n_idx, std::array<float,2> cost_drop) {
        for(int child_idx: node_list[n_idx].getChildrenIdx()) {
            std::array<float,2> old_child_cost = node_list[child_idx].cost();
            std::array<float,2> new_child_cost = {old_child_cost[0] - cost_drop[0], old_child_cost[1] - cost_drop[1]};
            node_list[child_idx].setCost(new_child_cost);
            propogateCostChange(child_idx, cost_drop);
        }
    }

    void applyCostChange (int n_idx, std::array<float,2> old_cost, std::array<float,2> new_cost) {
        std::array<float,2> cost_drop = {old_cost[0]-new_cost[0], old_cost[1]-new_cost[1]};
        propogateCostChange (n_idx, cost_drop);
    }

    float meanCost (std::array<float,2> cost) {
        return (cost[0]+cost[1])/2;
    }

    void connectNodes (int parent_idx, int child_idx) {
        node_list[parent_idx].addChild(child_idx);
        node_list[child_idx].makeParent(parent_idx);
    }

    void disconnectNode (int child_idx) {
        int parent_idx = node_list[child_idx].getParent();
        node_list[child_idx].makeParent(-1);
        node_list[parent_idx].removeChild(child_idx);
    }

    void rewireTree(int n_new_idx, MapManager& Map) {
        Node& n_new = node_list[n_new_idx];
        std::vector<int> neighbour_idx_list;
        getNeighboursByMeanPosition(n_new, neighbour_idx_list);
        ROS_DEBUG("\t6. Rewiring to new: %s", n_new.getNodeInfo().c_str());

        for(int n_near_idx: neighbour_idx_list) {
            Node& n_near = node_list[n_near_idx];

            std::array<float, 2> old_n_near_cost = n_near.cost();
            ROS_DEBUG("\t Considering: %s with mean cost: %f", n_near.getNodeInfo().c_str(), meanCost(old_n_near_cost));

            // only 1 robot moved
            if (n_new.getPosition(0) == n_near.getPosition(0) || n_new.getPosition(1) == n_near.getPosition(1)) {
                ROS_DEBUG("\t\tOnly one robot moved");
                int moving_robot_num;
                if (n_new.getPosition(0) == n_near.getPosition(0)) {
                    moving_robot_num = 1;
                } else {
                    moving_robot_num = 0;
                }
                if (isEdgeValid(moving_robot_num, n_new, n_near, Map)) {
                    std::array<float,2> updated_n_near_cost = n_new.cost();
                    updated_n_near_cost[moving_robot_num] += getEdgeCost(moving_robot_num, n_new, n_near);
                    // if new cost is lesser
                    if (meanCost(updated_n_near_cost) < meanCost(old_n_near_cost)) {
                        // disconnect from old parent
                        disconnectNode(n_near_idx);
                        // connect n_near to n_new
                        connectNodes(n_new_idx, n_near_idx);
                        // update cost
                        n_near.setCost(updated_n_near_cost);
                        n_near.setInMotion(moving_robot_num);
                        applyCostChange(n_near_idx, old_n_near_cost, updated_n_near_cost);
                        ROS_DEBUG("\t Rewired %s \n\t\tto: %s",
                                  n_near.getNodeInfo().c_str(),
                                  n_new.getNodeInfo().c_str());
                    }
                }
            } else { // else if both robots moved
                ROS_DEBUG("\t\tBoth robots moved");
                // (*) Step1: n_new to n_interm  (*) Step2: n_interm to n_near
                int best_step1_robot_num = -1;
                int best_step2_robot_num = -1;
                std::array<float,2> best_n_near_cost = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
                Node n_interm;
                // Find best interm node
                for (int step1_robot_num = 0; step1_robot_num <= 1; step1_robot_num++) {
                    // construct intermediate node candidate
                    Node n_interm_candidate;
                    int step2_robot_num = (step1_robot_num == 0) ? 1 : 0;
                    Position step1_move_pos = n_near.getPosition(step1_robot_num);
                    Position step1_fixed_pos = n_new.getPosition(step2_robot_num);
                    n_interm_candidate.setPosition(step1_robot_num, step1_move_pos);
                    n_interm_candidate.setPosition(step2_robot_num, step1_fixed_pos);
                    n_interm_candidate.setInMotion(step1_robot_num);
                    // check if candidate valid
                    if (isEdgeValid(step1_robot_num, n_new, n_interm_candidate, Map) && isEdgeValid(step2_robot_num, n_interm_candidate, n_near, Map)) {
                        // compute candidate interm cost
                        std::array<float,2> interm_cost = n_new.cost();
                        interm_cost[step1_robot_num] += getEdgeCost(step1_robot_num, n_new, n_interm_candidate);
                        n_interm_candidate.setCost(interm_cost);
                        // compute proposed near cost
                        std::array<float,2> updated_near_cost = interm_cost;
                        updated_near_cost[step2_robot_num] += getEdgeCost(step2_robot_num, n_interm_candidate, n_near);
                        // check if near cost is lesser
                        if (meanCost(updated_near_cost) < meanCost(best_n_near_cost) && meanCost(updated_near_cost) < meanCost(old_n_near_cost)) {
                            best_step1_robot_num = step1_robot_num;
                            best_step2_robot_num = step2_robot_num;
                            best_n_near_cost = updated_near_cost;
                            n_interm = n_interm_candidate;
                        }
                    }
                }
                // if best interm node found
                if (best_step1_robot_num != -1) {
                    ROS_DEBUG("\t\tBest interm node: %s", n_interm.getNodeInfo().c_str());
                    node_list.push_back(n_interm);
                    int n_interm_idx = node_list.size()-1;
                    // connect interm to new
                    connectNodes(n_new_idx, n_interm_idx);
                    // reconnect near to interm
                    disconnectNode(n_near_idx);
                    connectNodes(n_interm_idx, n_near_idx);
                    // update near cost change
                    node_list[n_near_idx].setInMotion(best_step2_robot_num);
                    node_list[n_near_idx].setCost(best_n_near_cost);
                    applyCostChange(n_near_idx, old_n_near_cost, best_n_near_cost);
                    ROS_DEBUG("\t Rewired %s \n\t\tto: %s \n\t\tvia interm: %s",
                              n_near.getNodeInfo().c_str(),
                              n_new.getNodeInfo().c_str(),
                              n_interm.getNodeInfo().c_str());

                } else {
                    ROS_DEBUG("\t\tNo good interm node");
                }
            }
        }
    }

    void printPath() {
        ROS_INFO("FINAL PATH:");
        int n_idx = goal_node_idx;
        ROS_INFO("%s", target_goal_node.getNodeInfo().c_str());
        bool move = false;
        int move_count = 0;
        int steps = 0;
        while(n_idx != -1) {
            if (node_list[n_idx].getParent()!= -1) {
                if (move != node_list[n_idx].isInMotion(0))
                    move_count++;
                move = node_list[n_idx].isInMotion(0);
                steps++;
            } else {
                move = node_list[n_idx].isInMotion(0);
            }
            ROS_INFO("%s", node_list[n_idx].getNodeInfo().c_str());
            n_idx = node_list[n_idx].getParent();
        }
        ROS_INFO("Steps: %i", steps);
        ROS_INFO("Move change count: %i", move_count-1);

    }

    void getBestPath(std::vector<Node>& best_path) {
        best_path.clear();
        if (goal_node_idx == -1) {
            return;
        }
        int n_idx = goal_node_idx;
        while(n_idx != -1) {
            best_path.insert(best_path.begin(), node_list[n_idx]);
            n_idx = node_list[n_idx].getParent();
        }
    }

    int start_node_idx;
    int goal_node_idx;
    Node target_goal_node;
    Node start_node;
    Node current_pos_node;

    float curr_dist_to_goal;
    std::vector<Node> node_list;

    ros::Subscriber current_pos_subscriber;
    ros::Subscriber goal_pos_subscriber;
    bool goal_found;
    bool start_initialized;
    bool goal_initialized;
};

}

#endif //LEAP_FROG_PLANNER_PLANNER_H
