
// Copyright 2020

#include <mission_manager/mission_manager.h>


namespace mission_manager {

MissionManager::MissionManager(ros::NodeHandle *nodehandle,
                               const goals_sequence_path_planner::Node &final_node,
                               const goals_sequence_path_planner::NodeArray &goals_list) :
                               nh_(*nodehandle),
                               final_node_(final_node),
                               goals_list_(goals_list),
                               controller_ac_("trajectory_tracking_control", true) {
  localization_initialized_ = false;
  initial_pose_sub_ = nh_.subscribe("odometry/filtered", 1000, &MissionManager::odomFilteredCb, this);
  pub_paths_ = nh_.advertise<goals_sequence_path_planner::PathArray>("/global_planner", 100);
  tf2_ros::TransformListener tfListener(tfBuffer_);

  ros::Rate rate(10);

  // getInitialPose();

  // while (!localization_initialized_ && ros::ok()) {
  //   rate.sleep();
  //   ROS_INFO("Waiting initial pose");
  // }

  ROS_INFO("Waiting for trajectory tracking control action server to start.");


  // Service: Path Planning
  path_planner_ = nh_.serviceClient<goals_sequence_path_planner::SeqGoalsPathPlanner>("seq_of_goals_path_planner");

  // Action Trajectory Tracking Control
  controller_ac_.waitForServer();

  execute();
}

void MissionManager::execute() {
  requestPaths();

  // Publish Paths Visualization
  pub_paths_.publish(paths_);

  requestTrajectoryController();
}

void MissionManager::requestPaths() {
  goals_sequence_path_planner::SeqGoalsPathPlanner srv;
  srv.request.final_point = final_node_;
  srv.request.sequence_of_goals = goals_list_;
  srv.request.start_point = start_node_;

  paths_.paths.clear();

  if (path_planner_.call(srv)) {
    paths_ = srv.response.paths;
  } else {
    ROS_ERROR("Failed to call service Path Planning");
  }
}

void MissionManager::requestTrajectoryController() {
  trajectory_tracking_control::ExecuteTrajectoryTrackingGoal goal;
  goal.average_velocity = 0.5;
  goal.sampling_time = 0.1;  // TODO(RAFAEL)
  goal.path = paths_.paths[0];

  controller_ac_.sendGoal(goal);
}

void MissionManager::odomFilteredCb(const nav_msgs::Odometry &msg) {
  if (!localization_initialized_) {
    start_node_.x.data = msg.pose.pose.position.x;
    start_node_.y.data = msg.pose.pose.position.y;

    double roll, pitch, yaw;

    // get Yaw
    geometry_msgs::Quaternion msg_quat;
    msg_quat = start_pose_.orientation;
    tf2::Quaternion quat(msg_quat.x, msg_quat.y, msg_quat.z, msg_quat.w);
    tf2::Matrix3x3 m(quat);
    m.getRPY(roll, pitch, yaw);

    start_node_.yaw.data = yaw;
    start_node_.optimize.data = true;

    ROS_INFO("Initial Pose: (%f, %f, %f)", start_node_.x.data, start_node_.y.data, yaw);

    localization_initialized_ = true;
  }
  ROS_INFO("Callback");
}

void MissionManager::getInitialPose() {
  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer_.lookupTransform("odom", "map", ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  start_pose_.position.x = transformStamped.transform.translation.x;
  start_pose_.position.y = transformStamped.transform.translation.y;

  start_pose_.orientation.x = transformStamped.transform.rotation.x;
  start_pose_.orientation.y = transformStamped.transform.rotation.y;
  start_pose_.orientation.z = transformStamped.transform.rotation.z;
  start_pose_.orientation.w = transformStamped.transform.rotation.w;

  // Start Node
  start_node_.x.data = start_pose_.position.x;
  start_node_.x.data = start_pose_.position.y;

  double roll, pitch, yaw;

  // get Yaw
  geometry_msgs::Quaternion msg;
  msg = start_pose_.orientation;
  tf2::Quaternion quat(msg.x, msg.y, msg.z, msg.w);
  tf2::Matrix3x3 m(quat);
  m.getRPY(roll, pitch, yaw);

  start_node_.yaw.data = yaw;
  start_node_.optimize.data = true;

  localization_initialized_ = true;
}
}  // namespace mission_manager
