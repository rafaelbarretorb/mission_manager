// Copyright 2020
#include <ros/ros.h>
#include <mission_manager/mission_manager.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "mission_manager");
  ros::NodeHandle nh;

  double final_pose_x = 0.0;
  double final_pose_y = 0.5;
  double final_pose_yaw = 0.0;

  goals_sequence_path_planner::Node final_pose;
  final_pose.x.data = final_pose_x;
  final_pose.y.data = final_pose_y;
  final_pose.yaw.data = final_pose_yaw;
  final_pose.optimize.data = true;

  // Goal 1
  goals_sequence_path_planner::Node goal_1;
  goal_1.x.data = 3.4;
  goal_1.y.data = 0.3;
  goal_1.yaw.data = 3.1416/2;
  goal_1.optimize.data = true;

  // Goal 2
  goals_sequence_path_planner::Node goal_2;
  goal_2.x.data = 3.4;
  goal_2.y.data = 2.0;
  goal_2.yaw.data = -3.1416/2;
  goal_2.optimize.data = true;

  // Goal 1
  // goals_sequence_path_planner::Node goal_1;
  // goal_1.x.data = 3.4;
  // goal_1.y.data = 2.0;
  // goal_1.yaw.data = -3.1416/2;
  // goal_1.optimize.data = true;

  // // // Goal 1
  // goals_sequence_path_planner::Node goal_1;
  // goal_1.x.data = 3.4;
  // goal_1.y.data = 2.0;
  // goal_1.yaw.data = -3.1416/2;
  // goal_1.optimize.data = true;

  // // Goal 2
  // goals_sequence_path_planner::Node goal_2;
  // goal_2.x.data = 1.0;
  // goal_2.y.data = 2.0;
  // goal_2.yaw.data = 3.1416/2;
  // goal_2.optimize.data = true;

  // // Goal 3
  // goals_sequence_path_planner::Node goal_3;
  // goal_3.x.data = 3.4;
  // goal_3.y.data = 0.3;
  // goal_3.yaw.data = 3.1416/2;
  // goal_3.optimize.data = true;

  // Goals list
  goals_sequence_path_planner::NodeArray goals;
  goals.nodes.push_back(goal_1);
  goals.nodes.push_back(goal_2);
  // goals.nodes.push_back(goal_3);
  // goals.nodes.push_back(goal_4);

  mission_manager::MissionManager manager(&nh, final_pose, goals);


  // ros::spin();
  return 0;
}