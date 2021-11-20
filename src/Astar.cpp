#include <exception>

#include "astar_planner/Astar.hpp"


namespace astar_planner
{

Astar::Astar()
{
  allow_unknown_ = false;

  // motion_model_ = MotionModel::VON_NEUMANN;     // 4 - connect
  motion_model_ = MotionModel::MOORE;        // 8 - connect
}

Astar::~Astar()
{
}

void Astar::setCostmap(nav2_costmap_2d::Costmap2D * costmap)
{
  costmap_ = costmap;
  xs_ = costmap_->getSizeInCellsX();
  ys_ = costmap_->getSizeInCellsY();
  ns_ = xs_ * ys_;

  // clear ptr vector
  clearQueue();

  // reset Graph
  {
    Graph g;
    std::swap(graph_, g);
    graph_.reserve(100000);
  }

  // create Graph, init all grids as Node object
  graph_.reserve(ns_);

  for (int j = 0; j < ys_; ++j) {
    for (int i = 0; i < xs_; ++i) {

      float v = OCCUPIED;
      if (j == 0 || i == 0 || i == xs_ - 1 || j == ys_ - 1) {
        // outline costmap edge
        // j = 0 下边第一行
        // i = 0 左边第一列
        // i = xs - 1  最后一列
        // j = ys - 1  最上一行
        v = OCCUPIED;
      } else {
        v = interpretCost(i, j, costmap_);
      }
      graph_.emplace_back(getIndex(i, j), i, j, v);
    }
  }

  initNeighborhood(motion_model_); // motion_model and traversal cost
}

float Astar::interpretCost(int i, int j, nav2_costmap_2d::Costmap2D * costmap)
{
  // int index = j * xs_ + i;
  if (allow_unknown_ && costmap->getCost(i, j) == UNKNOWN) {
    return MAX_NON_OBSTACLE;
  } else if (!allow_unknown_ && costmap->getCost(i, j) == UNKNOWN) {
    return OCCUPIED;
  } else if (costmap->getCost(i, j) == INSCRIBED) {
    return OCCUPIED;
  } else {
    return static_cast<float>(costmap->getCost(i, j));
  }
}


void Astar::initNeighborhood(const MotionModel & model)
{
  neighbors_grid_offsets_.clear();
  neighbors_traversal_cost_.clear();

  switch (model) {
    case MotionModel::UNKNOWN:
      throw std::runtime_error("Unknown neighborhood type selected.");
    case MotionModel::VON_NEUMANN:
      neighbors_grid_offsets_ = {-1, +1, -xs_, +xs_};
      neighbors_traversal_cost_ = {1.0, 1.0, 1.0, 1.0};
      assert(neighbors_grid_offsets_.size() == neighbors_traversal_cost_.size());
      break;
    case MotionModel::MOORE:
      neighbors_grid_offsets_ = {-1, +1, -xs_, +xs_, -xs_ - 1,
        -xs_ + 1, +xs_ - 1, +xs_ + 1};
      neighbors_traversal_cost_ = {1.0, 1.0, 1.0, 1.0, sqrt2, sqrt2, sqrt2, sqrt2};
      assert(neighbors_grid_offsets_.size() == neighbors_traversal_cost_.size());
      break;
    default:
      throw std::runtime_error(
              "Invalid neighborhood type selected. "
              "Von-Neumann and Moore are valid for Node2D.");
  } // switch
}


void Astar::clearQueue()
{
  NodeQueue q;
  std::swap(open_list_, q);
}

float Astar::getHeuristicCost(const NodePtr from, const NodePtr & to)
{

  float dx = std::abs(to->getCoordX() - from->getCoordX());
  float dy = std::abs(to->getCoordY() - from->getCoordY());

  // L2
  // return std::hypot(dx, dy);

  // diagnose L2
  return sqrt2_2 * std::min(dx, dy) + dx + dy;
}


bool Astar::createPath(
  const Eigen::Vector2i & start, const Eigen::Vector2i & end,
  std::vector<Eigen::Vector2i> & path)
{
  // reset costmap basic info
  setCostmap(costmap_);

  //
  int start_i = getIndex(start);
  int end_i = getIndex(end);

  if (start_i < 0 || start_i > ns_ || end_i < 0 || end_i > ns_) {
    std::cerr << "Astar: start or end index illegal " << start << " " << end << std::endl;
  }

  // 清空open_list
  clearQueue();

  // 起点, 加入graph 和 openlist
  NodePtr start_ptr =  &(graph_[start_i]);
  NodePtr end_ptr  = &(graph_[end_i]);

  start_ptr->update(0.0, getHeuristicCost(start_ptr, end_ptr));  // update g+h
  open_list_.push(start_ptr);


  // 中间变量
  unsigned int i = 0;  // cnt iterate number
  bool find_path = false;

  NodePtr current;

  // 主循环
  while (!open_list_.empty()) {
    i++;
    std::cout << "astar " << i << std::endl;

    // get top node and mark close_lists
    current = open_list_.top();
    open_list_.pop();
    current->visited();  // visited is closed list

    // check goal
    if (current->getIndex() == end_i) {
      std::cout << "\t\t\tFIND GOAL!\t\t\t" << std::endl;
      find_path = true;
      break;
    }

    // get neighbors / update /

    // std::vector<int> neighbor_i;
    // std::vector<float> neighbor_cost;
    // std::vector<NodePtr> neighbor_ptr;
    NodePtr tmp;

    for (int i = 0; i < neighbors_grid_offsets_.size(); ++i) {
      int t_i = current->getIndex() + neighbors_grid_offsets_[i];
      if (t_i < ns_ && t_i >= 0) {
        tmp = &(graph_[t_i]);
        if (tmp->wasVisited() || tmp->getCost() >= OCCUPIED) {
          continue;  // 已经在close_list, 或者是障碍物， 忽略
        } else {
          float tmp_g = current->G() + neighbors_traversal_cost_[i];
          if (tmp->G() > tmp_g) {
            tmp->update(tmp_g, getHeuristicCost(tmp, end_ptr));
            tmp->parent_ = current;
            // neighbor_ptr.push_back(tmp);
            open_list_.push(tmp);
          }
        }
      }
    }


  }  // while

  if (find_path) {
    std::cout << "FIND GOAL! iterations cnt " << i << std::endl;
  }
  std::vector<Eigen::Vector2i> path_reverse;
  if (!backtracePath(end_ptr, path_reverse)) {
    std::cout << "FATAL ERROR, find path but backtrace path field" << std::endl;
    return false;
  } else {
    path.clear();
    for (int i = path_reverse.size() - 1; i >= 0; i--) {
      path.push_back(path_reverse[i]);
    }
  }

  return true;
}

bool Astar::backtracePath(NodePtr goal, std::vector<Eigen::Vector2i> & path)
{
  if (!goal->parent_) {
    return false;
  }

  NodePtr current_node = goal;

  while (current_node->parent_) {
    path.push_back(current_node->getCoord());
    current_node = current_node->parent_;
  }

  return path.size() > 0;
}


} // namespace
