#include <fake_obstacle_layer/fake_obstacle_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(fake_obstacle_layer_namespace::FakeObstacleLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace fake_obstacle_layer_namespace
{

FakeObstacleLayer::FakeObstacleLayer() {}

void FakeObstacleLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  markedPosesSub = nh.subscribe("markedPoses", 10, &FakeObstacleLayer::markedPosesCallback, this);
  
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &FakeObstacleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void FakeObstacleLayer::markedPosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{

  // ROS_INFO("frame_id: %s", (msg->header.frame_id).c_str());
  // ROS_INFO("num of marked_poses: %d", (signed int) msg->poses.size());

  Eigen::Vector3f pose;
  // clear pose array
  poses.clear();

  unsigned int i = msg->poses.size();

  if (msg->poses.size() <= 0)
    return;

  for (i=0; i<msg->poses.size(); i++) {
    pose[0] = msg->poses[i].position.x;
    pose[1] = msg->poses[i].position.y;
    poses.push_back(pose);
  }

}

void FakeObstacleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void FakeObstacleLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double* min_x,
  double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  if (poses.size() <= 0)
    return;
  
  double x, y;
  unsigned int i;

  for (i=0; i<=poses.size(); i++) {
    x = poses[i][0];
    y = poses[i][1];
    *min_x = std::min(*min_x, x);
    *min_y = std::min(*min_y, y);
    *max_x = std::max(*max_x, x);
    *max_y = std::max(*max_y, y);
  }
}

void FakeObstacleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  if (poses.size() <= 0)
    return;

  unsigned int mx;
  unsigned int my;
  unsigned int i;

  for (i=0; i<poses.size(); i++) {
    
    if (master_grid.worldToMap(poses[i][0], poses[i][1], mx, my)){
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
    }
  }
}

} // end namespace
