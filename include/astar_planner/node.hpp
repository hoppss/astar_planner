#ifndef ASTAR_PLANNER__NODE_HPP_
#define ASTAR_PLANNER__NODE_HPP_

#include <iostream>
#include <string>
#include <limits>
#include <queue>
#include <unordered_map>
#include <functional>
#include <algorithm>
#include <memory>

#include <eigen3/Eigen/Core>

namespace astar_planner
{
const float sqrt2 = 1.414;
const float sqrt2_2 = -0.5857;

const float UNKNOWN = 255;
const float OCCUPIED = 254;
const float INSCRIBED = 253;
const float MAX_NON_OBSTACLE = 252;
const float FREE = 0;

const float NEUTRAL_COST = 50.0;
const float COST_FACTOR = 0.8;
const float sqrt2_10 = 14.14;

typedef unsigned char COSTTYPE;

class Node;
class NodeComparator;
typedef Node * NodePtr;

// typedef std::vector<NodePtr> Graph;
typedef std::vector<Node> Graph;

typedef std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> NodeQueue;

typedef std::vector<int> NeighborsModelVector;

enum class LISTYPE
{
  UNKNOW=0,
  OPENLIST,
  CLOSELIST,
};


class Node
{
public:
  // Node(const unsigned int index)
  // : index_(index), parent_(nullptr)
  // {
  //   cell_cost_ = std::numeric_limits<float>::quiet_NaN();

  //   f_ = g_ = std::numeric_limits<float>::max();
  //   h_ = 0.0;

  //   was_visited_ = false;
  //   // type_ = LISTYPE::UNKNOW;
  // }

  Node(unsigned int index, int i, int j, float cost)
  : index_(index), parent_(nullptr)
  {
    coord_(0) = i; // x
    coord_(1) = j; // y
    cell_cost_ = cost;

    f_ = g_ = std::numeric_limits<float>::max();
    h_ = 0.0;

    type_ = LISTYPE::UNKNOW;

    float weight_g_ = 1.0;   // weight of accumulate cost
    float weight_h_ = 1.0;   // weight of heuristic cost
  }

  ~Node() = default;

  inline bool operator==(const NodePtr & rhs)
  {
    return index_ == rhs->index_;
  }

  inline unsigned int getIndex() const
  {
    return index_;
  }

  inline float getCost() const
  {
    return cell_cost_;
  }

  inline void setCost(const float & cost)
  {
    cell_cost_ = cost;
  }

  inline bool wasVisited()
  {
    return type_ == LISTYPE::CLOSELIST;
  }

  inline void visited()
  {
    type_ = LISTYPE::CLOSELIST;
  }

  inline void queued(){
    type_ = LISTYPE::OPENLIST;
  }

  inline bool wasQueued() {
    return type_ == LISTYPE::OPENLIST;
  }

  inline bool wasUnknown() {
    return type_ == LISTYPE::UNKNOW;
  }

  inline Eigen::Vector2i getCoord()
  {
    return coord_;
  }

  inline float getCoordX()
  {
    return coord_(0);
  }

  inline float getCoordY()
  {
    return coord_(1);
  }

  void setCoordinate(int i, int j)
  {
    coord_(0) = i;
    coord_(1) = j;
  }

  float F() const
  {
    return f_;
  }
  float G() const
  {
    return g_;
  }
  float H() const
  {
    return h_;
  }

  void update(float _g, float _h)
  {
    g_ = _g;
    h_ = _h * weight_h_;
    f_ = g_ + h_;
  }

  NodePtr parent_;

private:
  unsigned int index_;
  float cell_cost_; // costmap cell cost
  Eigen::Vector2i coord_;        // x,y index in costmap


  float f_;     // total cost
  float g_;     // accumulated_cost
  float h_;     // heuristic

  LISTYPE type_;  // default is UNKOWN;

  float weight_g_;
  float weight_h_;
};

class NodeComparator
{
public:
  bool operator()(const NodePtr & a, const NodePtr & b) const
  {
    // if (a->F() > b->F()) {return true;} else {
    //   return a->G() > b->G();
    // }

    return a->F() > b->F();
  }
};


enum class MotionModel
{
  UNKNOWN = 0,
  VON_NEUMANN = 1,
  MOORE = 2,
  DUBIN = 3,
  REEDS_SHEPP = 4,
};

inline std::string toString(const MotionModel & n)
{
  switch (n) {
    case MotionModel::VON_NEUMANN:
      return "Von Neumann";
    case MotionModel::MOORE:
      return "Moore";
    case MotionModel::DUBIN:
      return "Dubin";
    case MotionModel::REEDS_SHEPP:
      return "Reeds-Shepp";
    default:
      return "Unknown";
  }
}

inline MotionModel fromString(const std::string & n)
{
  if (n == "VON_NEUMANN") {
    return MotionModel::VON_NEUMANN;
  } else if (n == "MOORE") {
    return MotionModel::MOORE;
  } else if (n == "DUBIN") {
    return MotionModel::DUBIN;
  } else if (n == "REEDS_SHEPP") {
    return MotionModel::REEDS_SHEPP;
  } else {
    return MotionModel::UNKNOWN;
  }
}

}  // namespace
#endif
