//
// Created by vishnu on 3/15/21.
//

#ifndef LEAP_FROG_PLANNER_NODE_H
#define LEAP_FROG_PLANNER_NODE_H

#include <array>
#include "robot.h"

namespace LeapFrog{

class Node {
public:

    Node() {
        parent_idx = -1;
        children.clear();
    }

    Node(Robot robot0, Robot robot1) {
        robots = {{robot0, robot1}};
        parent_idx = -1;
        children.clear();
    }

    Node(const Node& n) : robots(n.robots), parent_idx(n.parent_idx), children(n.children) {}

    Node& operator = (const Node& n ) {
        robots = n.robots;
        parent_idx = n.parent_idx;
        children = n.children;
    }

    Position getPosition(int robot_num) {
        return robots[robot_num].position;
    }

    void setPosition(int robot_num, Position pos) {
        robots[robot_num].position = pos;
    }

    float X(int robot_num) {
        return robots[robot_num].position.x;
    }

    float Y(int robot_num) {
        return robots[robot_num].position.y;
    }

    void setX(int robot_num, float x) {
        robots[robot_num].position.x = x;
    }

    void setY(int robot_num, float y) {
        robots[robot_num].position.y = y;
    }

    bool isInMotion(int robot_num) {
        return robots[robot_num].in_motion;
    }

    void setInMotion(int robot_num, bool m) {
        robots[robot_num].in_motion = m;
    }

    std::array<float,2> cost() {
        std::array<float, 2> cost{{robots[0].cost, robots[1].cost}};
        return cost;
    }

    float cost(int robot_num) {
        return robots[robot_num].cost;
    }

    void setCost(std::array<float,2> cost) {
        robots[0].cost = cost[0];
        robots[1].cost = cost[1];
    }

    void setCost(int robot_num, float cost) {
        robots[robot_num].cost = cost;
    }

    int getParent() {
        return parent_idx;
    }

    void makeParent(int p) {
        parent_idx = p;
    }

    std::vector<int> getChildrenIdx() {
        return children;
    }

    void addChild(int c) {
        children.push_back(c);
    }

    void removeChild(int child_idx) {
        for(int i=0; i<children.size(); i++) {
            if (children[i] == child_idx) {
                children.erase(children.begin()+i);
                break;
            }
        }
    }

    float getDistance(int robot_num, Node& n) {
        return sqrt(pow((n.X(robot_num) - X(robot_num)), 2)
             + pow((n.Y(robot_num) - Y(robot_num)), 2));
    }

    int getChildrenIdx(int i) {
        return children[i];
    }

    std::string getNodeInfo() {
        std::stringstream ss;
        ss.precision(2);
        ss<<getNodePositionInfo();
        ss<<"Cost: ("<<cost(0)<<","<<cost(1)<<")\t";
        ss<<"Move: ("<<isInMotion(0)<<","<<isInMotion(1)<<")\t";
        ss<<"Parent: "<<parent_idx;
        return ss.str();
    }

    std::string getNodePositionInfo() {
        std::stringstream ss;
        ss.precision(2);
        ss<<"Robot 0: ("<<X(0)<<","<<Y(0)<<")\t\t";
        ss<<"Robot 1: ("<<X(1)<<","<<Y(1)<<")\t";
        return ss.str();
    }

private:
    std::array<LeapFrog::Robot, 2> robots;
    int parent_idx;
    std::vector<int> children;
};
}

#endif //LEAP_FROG_PLANNER_NODE_H
