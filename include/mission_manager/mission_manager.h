
// Copyright 2020

#ifndef MISSION_MANAGER_MISSION_MANAGER_H_
#define MISSION_MANAGER_MISSION_MANAGER_H_

#include <ros/ros.h>

#include <goals_sequence_path_planner/SeqGoalsPathPlanner.h>
#include <goals_sequence_path_planner/Node.h>
#include <goals_sequence_path_planner/NodeArray.h>

namespace mission_manager {
class MissionManager {
 public:
  MissionManager(ros::NodeHandle *nodehandle,
                 const goals_sequence_path_planner::Node &final_node,
                 const goals_sequence_path_planner::NodeArray &goals_list);

  void execute();

  void requestPaths();

  void requestTrajectoryController();

 protected:
  ros::NodeHandle nh_;
  // Goal Sequence Path Planning Service Client
  ros::ServiceClient path_planner_;

  goals_sequence_path_planner::Node final_pose_;
  goals_sequence_path_planner::NodeArray goals_list_;
};
};  // namespace mission_manager
#endif  // MISSION_MANAGER_MISSION_MANAGER_H_
