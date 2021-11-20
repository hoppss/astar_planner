#include <cmath>
#include <string>
#include <memory>
#include <chrono>

#include "nav2_util/node_utils.hpp"

#include "astar_planner/astar_planner_ros.hpp"

namespace astar_planner
{

void AstarPlanner::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // // Parameter initialization
  // nav2_util::declare_parameter_if_not_declared(
  //   node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
  //     0.1));
  // node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void AstarPlanner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void AstarPlanner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void AstarPlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

nav_msgs::msg::Path AstarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  std::chrono::steady_clock::time_point a =std::chrono::steady_clock::now();

  // reget costmap size info
  astar_.setCostmap(costmap_);

  nav_msgs::msg::Path plan;
  plan.poses.clear();
  plan.header.stamp = node_->now();
  plan.header.frame_id = global_frame_;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    RCLCPP_ERROR(node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return plan;
  }

  // Set starting point
  unsigned int mx_start, my_start, mx_goal, my_goal;
  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start);

  // Set goal point
  costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal);

  std::vector<Eigen::Vector2i> grid_path;

  if (astar_.createPath(
      Eigen::Vector2i(mx_start, my_start), Eigen::Vector2i(mx_goal, my_goal),
      grid_path) && !grid_path.empty())
  {
    for (int i = 0; i < grid_path.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose = getWorldCoords(grid_path[i], costmap_);
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      pose.header.stamp = node_->now();
      pose.header.frame_id = global_frame_;
      plan.poses.push_back(pose);
    }
  }

  plan.poses.push_back(goal);

  std::chrono::steady_clock::time_point b = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(b - a);
  std::cout << "Astar used " << time_span.count() * 1000 << " milliseconds" << std::endl;

  return plan;
}

geometry_msgs::msg::Pose AstarPlanner::getWorldCoords(
  const Eigen::Vector2i & pose, const nav2_costmap_2d::Costmap2D * costmap)
{
  geometry_msgs::msg::Pose msg;
  msg.position.x =
    static_cast<float>(costmap->getOriginX()) + ( static_cast<float>(pose(0)) + 0.5) *
    costmap->getResolution();
  msg.position.y =
    static_cast<float>(costmap->getOriginY()) + ( static_cast<float>(pose(1)) + 0.5) *
    costmap->getResolution();
  return msg;
}

}  // namespace astar_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(astar_planner::AstarPlanner, nav2_core::GlobalPlanner)