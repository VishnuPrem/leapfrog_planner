#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "ros/console.h"
#include "leap_frog_planner/planner.h"
#include "leap_frog_planner/robot.h"

int main(int argc, char **argv)
{
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    srand(time(0));
    ros::init(argc, argv, "leap_frog_planner");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(10);

    LeapFrog::Position r0_s(-8,-6);
    LeapFrog::Position r0_e(-8,9);
    LeapFrog::Position r1_s(-6,-8);
    LeapFrog::Position r1_e(-6,7);

    LeapFrog::MapManager m(n);
    LeapFrog::Planner planner(r0_s, r0_e, r1_s, r1_e);
    planner.getPath(2000, m);

    ROS_INFO("PLANNER COMPLETE");


    while (ros::ok())
    {
        planner.publishTree(m);
        m.publishRobotAnim();
        m.publishMap();

        ros::spinOnce();
        loop_rate.sleep();
    }

//    ros::spin();
    return 0;
}

//    LeapFrog::Position p0(2,5);
//    LeapFrog::Position p1(-3,6);
//    LeapFrog::Position p2(3,1);
//    LeapFrog::Position p3(7.1,5.8);
//
//    if(m.isObstacleinTriangle(p1,p2,p3)) {
//        ROS_INFO("isObstacleinTriangle TRUE");
//    }

//LeapFrog::Position fixed_robot_pos(-6,-8);
//LeapFrog::Position moving_robot_old_pos(-8,-6);
//LeapFrog::Position moving_robot_new_pos(-3.9,-9);
//float x1,y1,x2,y2,x0,y0;
//x0 = fixed_robot_pos.x;
//y0 = fixed_robot_pos.y;
//x1 = moving_robot_old_pos.x;
//y1 = moving_robot_old_pos.y;
//x2 = moving_robot_new_pos.x;
//y2 = moving_robot_new_pos.y;
//float num = abs((x2-x1)*(y1-y0) - (y2-y1)*(x1-x0));
//float den = moving_robot_old_pos.getDistance(moving_robot_new_pos);
//float min_dist_bw_fixed_and_moving = num/den ;
//ROS_INFO("Min dist %f num %f den %f", min_dist_bw_fixed_and_moving, num, den);