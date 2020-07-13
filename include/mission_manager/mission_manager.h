
// Copyright 2020

#ifndef MISSION_MANAGER_MISSION_MANAGER_H_
#define MISSION_MANAGER_MISSION_MANAGER_H_

#include <ros/ros.h>

// TF2
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// ROS Messages
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

// Action
#include <actionlib/client/simple_action_client.h>
#include <trajectory_tracking_control/ExecuteTrajectoryTrackingAction.h>

#include <goals_sequence_path_planner/SeqGoalsPathPlanner.h>
#include <goals_sequence_path_planner/Node.h>
#include <goals_sequence_path_planner/NodeArray.h>
#include <goals_sequence_path_planner/PathArray.h>

namespace mission_manager {

typedef actionlib::SimpleActionClient<trajectory_tracking_control::ExecuteTrajectoryTrackingAction>
    ExecuteTrajectoryTrackingActionClient;

class MissionManager {
 public:
  MissionManager(ros::NodeHandle *nodehandle,
                 const goals_sequence_path_planner::Node &final_node,
                 const goals_sequence_path_planner::NodeArray &goals_list);

  void execute();

  void requestPaths();

  void requestTrajectoryController();

  void odomFilteredCb(const nav_msgs::Odometry &msg);

  void getInitialPose();

 protected:
  ros::NodeHandle nh_;
  ros::Subscriber initial_pose_sub_;
  ros::Publisher pub_paths_;
  // Goal Sequence Path Planning Service Client
  ros::ServiceClient path_planner_;

  tf2_ros::Buffer tfBuffer_;

  geometry_msgs::Pose start_pose_;
  goals_sequence_path_planner::Node start_node_;
  goals_sequence_path_planner::Node final_node_;
  goals_sequence_path_planner::NodeArray goals_list_;
  goals_sequence_path_planner::PathArray paths_;

  ExecuteTrajectoryTrackingActionClient controller_ac_;

  bool localization_initialized_;
};
};  // namespace mission_manager
#endif  // MISSION_MANAGER_MISSION_MANAGER_H_
