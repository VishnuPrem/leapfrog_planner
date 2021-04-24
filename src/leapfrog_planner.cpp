#include <ros/ros.h>
#include <sstream>
#include <ros/console.h>
#include "leap_frog_planner/planner.h"
#include "leap_frog_planner/robot.h"

int main(int argc, char **argv)
{
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

//    srand(time(0));
    ros::init(argc, argv, "leap_frog_planner");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    LeapFrog::MapManager m(n);
    LeapFrog::VisualizationManager v(n);
    LeapFrog::Planner planner(n);
    ROS_INFO("Planner Ready");
    while (ros::ok())
    {
        planner.Plan(2000, m, v);
        v.publishTrees();
        v.publishRobotAnim();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
