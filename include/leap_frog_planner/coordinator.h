//
// Created by vishnu on 4/26/21.
//

#ifndef LEAP_FROG_PLANNER_COORDINATOR_H
#define LEAP_FROG_PLANNER_COORDINATOR_H
#include <ros/ros.h>
#include "constants.h"
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <vector>
#include "node.h"

namespace LeapFrog {

class Coordinator {

public:
    Coordinator (ros::NodeHandle& n_h) {

        robot0_goal_publisher = n_h.advertise<geometry_msgs::Pose>(ROBOT0_GOAL_TOPIC, 10);
        robot1_goal_publisher = n_h.advertise<geometry_msgs::Pose>(ROBOT1_GOAL_TOPIC, 10);

        robot0_complete_subscriber =  n_h.subscribe(ROBOT0_COMPLETE_TOPIC, 1000, &Coordinator::robotCompleteCallback, this);
        robot1_complete_subscriber =  n_h.subscribe(ROBOT1_COMPLETE_TOPIC, 1000, &Coordinator::robotCompleteCallback, this);

        robot_complete = false;
        path_idx = 0;
    }

    void execute_path (std::vector<Node> path_) {
        if (!path_.empty()) {
            path = path_;
            path_idx = 0;
            robot_complete = true;
        }
        if (robot_complete) {
            robot_complete = false;
            path_idx++;
            if (path_idx < path.size()) {
                int robot_num = path[path_idx].isInMotion(0) ? 0 : 1;
                geometry_msgs::Pose pose;
                pose.position.x = path[path_idx].X(robot_num);
                pose.position.y = path[path_idx].Y(robot_num);
                if (robot_num == 0) {
                    robot0_goal_publisher.publish(pose);
                } else {
                    robot1_goal_publisher.publish(pose);
                }
                ROS_INFO("Robot: %i to (%f,%f)", robot_num, pose.position.x, pose.position.y);
            }
        }
    }

private:

    void robotCompleteCallback(std_msgs::Bool msg) {
        robot_complete = true;
    }

    ros::Publisher robot0_goal_publisher;
    ros::Publisher robot1_goal_publisher;
    ros::Subscriber robot0_complete_subscriber;
    ros::Subscriber robot1_complete_subscriber;

    std::vector<Node> path;
    int path_idx;
    bool robot_complete;

};

}
#endif //LEAP_FROG_PLANNER_COORDINATOR_H
