#ifndef ASTAR_PLANNER_HPP_
#define ASTAR_PLANNER_HPP_

#include <chrono>
#include <iostream>
#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "astar_planner/Astar.hpp"

namespace astar_planner
{

class AstarPlanner : public nav2_core::GlobalPlanner
{
public:
  AstarPlanner() = default;
  ~AstarPlanner() = default;

  // plugin configure
  void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // plugin cleanup
  void cleanup() override;

  // plugin activate
  void activate() override;

  // plugin deactivate
  void deactivate() override;

  // This method creates path for given start and goal pose.
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

  geometry_msgs::msg::Pose getWorldCoords(
    const Eigen::Vector2i & pose, const nav2_costmap_2d::Costmap2D * costmap);

private:
  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // node ptr
  nav2_util::LifecycleNode::SharedPtr node_;

  // Global Costmap
  nav2_costmap_2d::Costmap2D * costmap_;

  // The global frame of the costmap
  std::string global_frame_;
  std::string name_;

  Astar astar_;
};

}  // namespace astar_planner

#endif  // ASTAR_PLANNER_HPP_
