#ifndef FAKE_OBSTACLE_LAYER_H_
#define FAKE_OBSTACLE_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Core>
#include <vector>

namespace fake_obstacle_layer_namespace
{

class FakeObstacleLayer : public costmap_2d::Layer
{
public:
  FakeObstacleLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:
  void markedPosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  ros::Subscriber markedPosesSub;
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

  std::vector<Eigen::Vector3f> poses;

};
}
#endif
