# Leapfrog Planner for Cooperative Localization

The localization estimates obtained in featureless environments with few landmarks can have high uncertainty and cause SLAM algorithms
to fail catas-trophically. This work proposes a solution to this problem by introducing a pair of robots that can coordinate with each 
other during navigation.   If one robot remains stationary while the other moves, then the moving robot can use the stationary robot as
a feature to track. This allows the moving robot to get a more accurate estimate of its current position irrespective of the quality of 
the features in the environment.  By repeatedly alternating the roles of being stationary and moving, the pair of robots can successfully 
localize while navigating.  This method of synchronized alternating motion is termed as ’leapfrogging’. This repo contains a ROS package for
the path planning aspect of leapfrogging i.e. an algorithm to generate a path for a pair of two robots so as to ensure that together the 
robots can generate thebest possible estimate of the map and their own position.

## Usage
In seperate terminals, run:
```
$ roscore
$ rviz
$ rosrun map_server map_server map/pennov4.yaml 
$ rosrun leap_frog_plaer leap_frog_planner_node 
$ rosrun leap_frog_plaer dummy_simulator 0
$ rosrun leap_frog_plaer dummy_simulator 1
```
Open rviz using the configuration in ./leapfrog_config.rviz

In rviz, set a 2D Pose Estimate marker to initialise the simulators.

In rviz, set a 2D Nav Goal marker to initiate the planner.

Once the plan is generated, the robots will move along the planned path. Once the goal is reached, a new 2D Nav Goal marker can be dropped to restart the planner

