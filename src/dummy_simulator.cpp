//
// Created by vishnu on 4/25/21.
//

#include <ros/ros.h>
#include <sstream>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>
#include "leap_frog_planner/constants.h"
#include <visualization_msgs/Marker.h>

/*
 * 1. Listen to goal topic
 * 2. Navigate to goal
 * 3. Notify arrival at goal
 * 4. Always publish position marker
 */
geometry_msgs::Pose goal_pose;
geometry_msgs::Pose current_pose;
int robot_num;
bool reached = false;
visualization_msgs::Marker position_marker;

void robotGoalCallback (geometry_msgs::Pose msg) {
    goal_pose = msg;
    reached = false;

}

void robotStartCallback(geometry_msgs::PoseWithCovarianceStamped msg) {
    current_pose = msg.pose.pose;
    if (robot_num == 1) {
        current_pose.position.x += LeapFrog::INTERROBOT_START_OFFSET[0];
        current_pose.position.y += LeapFrog::INTERROBOT_START_OFFSET[1];
    }
}

bool moveToGoal () {

    float speed = 0.02;
    float diff_x = goal_pose.position.x - current_pose.position.x;
    float diff_y = goal_pose.position.y - current_pose.position.y;
    float magn = std::sqrt(diff_x*diff_x + diff_y*diff_y);
    if (magn < speed*2) {
        current_pose.position.x = goal_pose.position.x;
        current_pose.position.y = goal_pose.position.y;
        return true;
    }

    current_pose.position.x += diff_x * speed/magn;
    current_pose.position.y += diff_y * speed/magn;

    return false;

}

void initMarker() {
    position_marker.header.frame_id = "/map";
    position_marker.header.stamp = ros::Time::now();
    if (robot_num == 0) {
        position_marker.ns = "robot0_pos";
    } else {
        position_marker.ns = "robot1_pos";
    }
    position_marker.action = visualization_msgs::Marker::ADD;
    position_marker.pose.orientation.w = 1.0;
    position_marker.id = 0;
    position_marker.type = visualization_msgs::Marker::POINTS;
    position_marker.scale.x = 1;
    position_marker.scale.y = 1;

    geometry_msgs::Point p;
    p.z = 0;
    position_marker.points.push_back(p);

    position_marker.color.a = 1.0;
    if (robot_num ==0) {
        position_marker.color.r = 1.0;
    } else {
        position_marker.color.b = 1.0;
    }
}

int main(int argc, char **argv) {

    if (argc < 2) {
        ROS_ERROR("Robot number not provided");
        exit(1);
    } else {
        robot_num = std::stoi(argv[1]);
        if (robot_num != 0 && robot_num !=1) {
            ROS_ERROR("Invalid robot num");
            exit(1);
        }
    }

    std::string node_name = std::string("robot") + std::to_string(robot_num) + std::string("_dummy_simulator");
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    std::string goal_topic = robot_num == 0 ? LeapFrog::ROBOT0_GOAL_TOPIC : LeapFrog::ROBOT1_GOAL_TOPIC;
    std::string complete_topic = robot_num == 0 ? LeapFrog::ROBOT0_COMPLETE_TOPIC : LeapFrog::ROBOT1_COMPLETE_TOPIC;

    ros::Subscriber start_subscriber = n.subscribe(LeapFrog::PLANNER_START_TOPIC, 1000, &robotStartCallback);
    ros::Subscriber goal_subscriber = n.subscribe(goal_topic, 1000, &robotGoalCallback);
    ros::Publisher complete_publisher = n.advertise<std_msgs::Bool>(complete_topic, 10);
    ros::Publisher marker_publisher = n.advertise<visualization_msgs::Marker>(LeapFrog::ROBOT_MARKER_TOPIC, 10);

    std_msgs::Bool complete_msg;
    complete_msg.data = true;
    initMarker();

    while (ros::ok()) {

        ROS_INFO("Current pose: (%f,%f) \tGoal pose: (%f,%f) \tGoal reached: %d", current_pose.position.x, current_pose.position.y,
                 goal_pose.position.x, goal_pose.position.y, reached);

        if (!reached) {
            reached = moveToGoal();
            if (reached) {
                complete_publisher.publish(complete_msg);
            }
        }

        position_marker.points[0].x = current_pose.position.x;
        position_marker.points[0].y = current_pose.position.y;
        marker_publisher.publish(position_marker);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
