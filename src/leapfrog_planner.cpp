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

    LeapFrog::Position r0_s(2,-6);
    LeapFrog::Position r0_e(-8,9);
    LeapFrog::Position r1_s(4,-8);
    LeapFrog::Position r1_e(-6,7);

    LeapFrog::MapManager m(n);
    LeapFrog::VisualizationManager v(n);

    LeapFrog::Planner planner(r0_s, r0_e, r1_s, r1_e);
    planner.getPath(2000, m, v);

    ROS_INFO("PLANNER COMPLETE");


    while (ros::ok())
    {
        v.publishTrees();
        v.publishRobotAnim();
        m.publishMap();

        ros::spinOnce();
        loop_rate.sleep();
    }

//    ros::spin();
    return 0;
}
