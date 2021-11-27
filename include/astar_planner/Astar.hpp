#ifndef ASTAR_PLANNER_ASTAR_HPP_
#define ASTAR_PLANNER_ASTAR_HPP_

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <queue>
#include <utility>
#include <algorithm>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "astar_planner/node.hpp"

namespace astar_planner
{

class Astar
{
public:
  Astar();
  ~Astar();

  void setCostmap(nav2_costmap_2d::Costmap2D * costmap);
  // void initNeighborhood(const MotionModel & model);

  float interpretCost(int i, int j, nav2_costmap_2d::Costmap2D * costmap);

  bool createPath(
    const Eigen::Vector2i & start, const Eigen::Vector2i & end,
    std::vector<Eigen::Vector2i> & path);

  bool backtracePath(NodePtr goal, std::vector<Eigen::Vector2i> & path);


  inline int getIndex(int i, int j)
  {
    return i + j * xs_;
  }

  inline int getIndex(const Eigen::Vector2i & pose)
  {
    return pose(0) + pose(1) * xs_;
  }

  void expansiionNeighbors(
    const int & current_index, std::vector<int> & neighbors_index,
    std::vector<float> & neighbors_cost);

  void clearQueue();

  float getHeuristicCost(const NodePtr from, const NodePtr & to);

  // vis, get a costmap to show openlist and closelist
  nav_msgs::msg::OccupancyGrid visualize();

private:
  nav2_costmap_2d::Costmap2D * costmap_;
  int xs_;
  int ys_;
  int ns_;
  bool allow_unknown_;

  Graph graph_;
  NodeQueue open_list_;
  MotionModel motion_model_;
  // NeighborsModelVector neighbors_grid_offsets_;
  // std::vector<float> neighbors_traversal_cost_;


};

}  // namespace

#endif
