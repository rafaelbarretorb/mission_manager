
// Copyright 2020

#ifndef MISSION_MANAGER_MISSION_MANAGER_H_
#define MISSION_MANAGER_MISSION_MANAGER_H_

#include <ros/ros.h>

#include <goals_sequence_path_planner/SeqGoalsPathPlanner.h>

namespace mission_manager {
class MissionManager {
 public:
  MissionManager();

  

  void requestPaths();

  void requestTrajectoryController();

 protected:
  // Goal Sequence Path Planning Service Client
  ros::ServiceClient path_planner_;
  
};
};  // namespace mission_manager
#endif  // MISSION_MANAGER_MISSION_MANAGER_H_
