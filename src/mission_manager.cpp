
// Copyright 2020

#include <mission_manager/mission_manager.h>


namespace mission_manager {

MissionManager::MissionManager(ros::NodeHandle *nodehandle,
                               const goals_sequence_path_planner::Node &final_node,
                               const goals_sequence_path_planner::NodeArray &goals_list) :
                               nh_(*nodehandle),
                               final_pose_(final_node),
                               goals_list_(goals_list) {
}

void MissionManager::execute() {}

void MissionManager::requestPaths() {}

void MissionManager::requestTrajectoryController() {}
}  // namespace mission_manager
