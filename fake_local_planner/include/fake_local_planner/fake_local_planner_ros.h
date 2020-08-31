#ifndef TEB_LOCAL_PLANNER_ROS_H_
#define TEB_LOCAL_PLANNER_ROS_H_

#include <ros/ros.h>

// base local planner base class and utilities
#include <nav_core/base_local_planner.h>
//#include <mbf_costmap_core/costmap_controller.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/costmap_model.h>

// message types
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <costmap_converter/ObstacleMsg.h>

// transforms
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

// costmap
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_converter/costmap_converter_interface.h>


// dynamic reconfigure
#include <fake_local_planner/TebLocalPlannerReconfigureConfig.h>
#include <dynamic_reconfigure/server.h>

// boost classes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

namespace fake_local_planner
{

/**
  * @class TebLocalPlannerROS
  * @brief Implements both nav_core::BaseLocalPlanner and mbf_costmap_core::CostmapController abstract
  * interfaces, so the teb_local_planner plugin can be used both in move_base and move_base_flex (MBF).
  * @todo Escape behavior, more efficient obstacle handling
  */
//class FakeLocalPlannerROS : public nav_core::BaseLocalPlanner, public mbf_costmap_core::CostmapController
class FakeLocalPlannerROS : public nav_core::BaseLocalPlanner
{

public:
  /**
    * @brief Default constructor of the teb plugin
    */
  FakeLocalPlannerROS();

  /**
    * @brief  Destructor of the plugin
    */
  ~FakeLocalPlannerROS();

  /**
    * @brief Initializes the teb plugin
    * @param name The name of the instance
    * @param tf Pointer to a tf buffer
    * @param costmap_ros Cost map representing occupied and free space
    */
  void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
    * @brief Set the plan that the teb local planner is following
    * @param orig_global_plan The plan to pass to the local planner
    * @return True if the plan was updated successfully, false otherwise
    */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  /**
    * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
    * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
    * @return True if a valid trajectory was found, false otherwise
    */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  /**
    * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base.
    * @remark Extended version for MBF API
    * @param pose the current pose of the robot.
    * @param velocity the current velocity of the robot.
    * @param cmd_vel Will be filled with the velocity command to be passed to the robot base.
    * @param message Optional more detailed outcome as a string
    * @return Result code as described on ExePath action result:
    *         SUCCESS         = 0
    *         1..9 are reserved as plugin specific non-error results
    *         FAILURE         = 100   Unspecified failure, only used for old, non-mfb_core based plugins
    *         CANCELED        = 101
    *         NO_VALID_CMD    = 102
    *         PAT_EXCEEDED    = 103
    *         COLLISION       = 104
    *         OSCILLATION     = 105
    *         ROBOT_STUCK     = 106
    *         MISSED_GOAL     = 107
    *         MISSED_PATH     = 108
    *         BLOCKED_PATH    = 109
    *         INVALID_PATH    = 110
    *         TF_ERROR        = 111
    *         NOT_INITIALIZED = 112
    *         INVALID_PLUGIN  = 113
    *         INTERNAL_ERROR  = 114
    *         121..149 are reserved as plugin specific errors
    */
  uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped& pose, const geometry_msgs::TwistStamped& velocity,
                                   geometry_msgs::TwistStamped &cmd_vel, std::string &message);

  /**
    * @brief  Check if the goal pose has been achieved
    * 
    * The actual check is performed in computeVelocityCommands(). 
    * Only the status flag is checked here.
    * @return True if achieved, false otherwise
    */
  bool isGoalReached();

  /**
    * @brief Dummy version to satisfy MBF API
    */
  //bool isGoalReached(double xy_tolerance, double yaw_tolerance) { return isGoalReached(); };

  /**
    * @brief Requests the planner to cancel, e.g. if it takes too much time
    * @remark New on MBF API
    * @return True if a cancel has been successfully requested, false if not implemented.
    */
  bool cancel() { return false; };
};
  
}; // end namespace fake_local_planner

#endif // TEB_LOCAL_PLANNER_ROS_H_


