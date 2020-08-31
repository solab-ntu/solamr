#include <fake_local_planner/fake_local_planner_ros.h>
#include <mbf_msgs/ExePathResult.h>// MBF return codes
#include <pluginlib/class_list_macros.h>// pluginlib macros
 
// register this planner both as a BaseLocalPlanner and as a MBF's CostmapController plugin
PLUGINLIB_EXPORT_CLASS(fake_local_planner::FakeLocalPlannerROS, nav_core::BaseLocalPlanner)
PLUGINLIB_EXPORT_CLASS(fake_local_planner::FakeLocalPlannerROS, mbf_costmap_core::CostmapController)

namespace fake_local_planner
{

FakeLocalPlannerROS::FakeLocalPlannerROS() 
{
}

FakeLocalPlannerROS::~FakeLocalPlannerROS()
{
}

void FakeLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  ROS_INFO("[Fake local planner] Initialize");
  ros::NodeHandle nh("~/" + name);
}

bool FakeLocalPlannerROS::isGoalReached()
/*
Must have interface
*/
{
  //ROS_DEBUG("[Fake local planner] isGoalReached, do nothing");
  return true;
}

bool FakeLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
/*
Must have interface 
*/
{
  //ROS_DEBUG("[Fake local planner] setPlan, do nothing");
  return true;
}

bool FakeLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
/*
Must have interface 
*/
{
  std::string dummy_message;
  geometry_msgs::PoseStamped dummy_pose;
  geometry_msgs::TwistStamped dummy_velocity, cmd_vel_stamped;
  uint32_t outcome = computeVelocityCommands(dummy_pose, dummy_velocity, cmd_vel_stamped, dummy_message);
  cmd_vel = cmd_vel_stamped.twist;
  return outcome == mbf_msgs::ExePathResult::SUCCESS;
}

uint32_t FakeLocalPlannerROS::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                     const geometry_msgs::TwistStamped& velocity,
                                                     geometry_msgs::TwistStamped &cmd_vel,
                                                     std::string &message)
{return mbf_msgs::ExePathResult::SUCCESS;}

}


