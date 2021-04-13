//
// Created by vishnu on 3/15/21.
//

#ifndef LEAP_FROG_PLANNER_ROBOT_H
#define LEAP_FROG_PLANNER_ROBOT_H
#include <string>
#include <sstream>

namespace LeapFrog {

struct Position {

    float x;
    float y;

    Position() : x(0.0), y(0.0) {}

    Position(float x_, float y_) : x(x_), y(y_) {}

    Position(const Position &p) : x(p.x), y(p.y) {}

    float getDistance(Position& p) {
        return sqrt(pow((p.x - x), 2) + pow((p.y - y), 2));
    }

    void operator = (const Position &p ) {
        x = p.x;
        y = p.y;
    }

    bool operator== (const Position &p)
    {
        return (x == p.x) && (y == p.y);
    }

};

class Robot {

public:

    Robot() : position(), in_motion(false), cost(0.0) {}

    Robot(Position pos, bool v_): position(pos), in_motion(v_), cost(0) {}

    Robot(const Robot& r) : position(r.position), in_motion(r.in_motion), cost(r.cost) {}

    Robot& operator = (const Robot& n ) {
        position = n.position;
        in_motion = n.in_motion;
        cost = n.cost;
    }

    Position position;
    bool in_motion;
    float cost;
    
};
}

#endif //LEAP_FROG_PLANNER_ROBOT_H
