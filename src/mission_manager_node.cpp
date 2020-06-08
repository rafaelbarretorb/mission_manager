// Copyright 2020
#include <ros/ros.h>
#include <mission_manager/mission_manager.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "mission_manager");
  ros::NodeHandle nh;

  double final_pose_x = 0.0;
  double final_pose_y = -1.0;
  double final_pose_yaw = 0.0;

  goals_sequence_path_planner::Node final_pose;

  // Goal 1
  goals_sequence_path_planner::Node goal_1;
  goal_1.x.data = 6.0;
  goal_1.y.data = 0.0;
  goal_1.yaw.data = 3.1416;
  goal_1.optimize.data = true;

  // Goals list
  goals_sequence_path_planner::NodeArray goals;
  goals.nodes.push_back(goal_1);


  // ros::spin();
  return 0;
}