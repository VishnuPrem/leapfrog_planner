#include <ros/ros.h>
#include <sstream>
#include <ros/console.h>
#include "leap_frog_planner/planner.h"
#include "leap_frog_planner/node.h"
#include "leap_frog_planner/coordinator.h"

int main(int argc, char **argv)
{
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::init(argc, argv, "leap_frog_planner");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    LeapFrog::MapManager m(n);
    LeapFrog::VisualizationManager v(n);
    LeapFrog::Planner planner(n);
    LeapFrog::Coordinator coordinator(n);

    ROS_INFO("Planner Ready");
    while (ros::ok())
    {
        std::vector<LeapFrog::Node> best_path = planner.Plan(m, v);
        v.publishTrees();
        v.publishRobotAnim();
        coordinator.execute_path(best_path);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
