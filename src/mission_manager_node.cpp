// Copyright 2020
#include <ros/ros.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "mission_manager");
  ros::NodeHandle nh;

  double final_pose_x = 0.0;
  double final_pose_y = 0.0;
  double final_pose_yaw = 0.0;

  // Request Path Planning Sequence
  // Request

  ros::spin();
  return 0;
}