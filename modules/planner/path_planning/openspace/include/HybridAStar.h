/*
*******************************************************************************
* HybridAStar.h:
* Hybrid A star algorithm, to generate the coarse collision-free trajectory
* This header file can be read by C++ compilers
*
* by Hu.ZH, Bin Li(CrossOcean.ai)
*******************************************************************************
*/

#ifndef _HYBRIDASTAR_H_
#define _HYBRIDASTAR_H_

#include "CollisionChecking.h"
#include "common/math/Geometry/include/Reeds_Shepp.h"
#include "hybridstlastar.h"
#include "openspacedata.h"

namespace ASV::planning {

constexpr int DEBUG_LISTS = 0;
constexpr int DEBUG_LIST_LENGTHS_ONLY = 0;

class HybridState4DNode {
  using HybridAStar_Search =
      HybridAStarSearch<HybridState4DNode, SearchConfig, CollisionChecking>;

  enum MovementType {
    STRAIGHT_FORWARD = 0,
    LEFTTURN_FORWARD,
    RIGHTTURN_FORWARD,
    STRAIGHT_REVERSE,
    RIGHTTURN_REVERSE,
    LEFTTURN_REVERSE
  };

 public:
  HybridState4DNode()
      : x_(0.0), y_(0.0), theta_(0.0), type_(STRAIGHT_FORWARD) {}

  HybridState4DNode(float px, float py, float ptheta,
                    MovementType ptype = STRAIGHT_FORWARD)
      : x_(px), y_(py), theta_(ptheta), type_(ptype) {}

  float x() const noexcept { return x_; }
  float y() const noexcept { return y_; }
  float theta() const noexcept { return theta_; }
  MovementType type() const noexcept { return type_; }

  // Here's the heuristic function that estimates the distance from a Node
  // to the Goal.
  float GoalDistanceEstimate(const HybridState4DNode &nodeGoal) {
    // return abs(x - nodeGoal.x) + abs(y - nodeGoal.y);

    return (x_ - nodeGoal.x()) * (x_ - nodeGoal.x()) +
           (y_ - nodeGoal.y()) * (y_ - nodeGoal.y());
    // return 0;
  }  // GoalDistanceEstimate

  bool IsGoal(const HybridState4DNode &nodeGoal) {
    return IsSameState(nodeGoal);
  }  // IsGoal

  // This generates the successors to the given Node.
  bool GetSuccessors(HybridAStar_Search *astarsearch,
                     HybridState4DNode *parent_node,
                     const SearchConfig &search_config,
                     const CollisionChecking &collision_checking) {
    float L = search_config.move_length;
    float max_turn = search_config.turning_angle;

    auto move_step = update_movement_step(L, max_turn, this->theta_);

    // push each possible move except allowing the search to go backwards
    for (int i = 0; i != 6; ++i) {
      auto move_at_type_i = move_step.col(i);
      float new_x = this->x_ + move_at_type_i(0);
      float new_y = this->y_ + move_at_type_i(1);
      float new_theta = ASV::common::math::fNormalizeheadingangle(
          this->theta_ + move_at_type_i(2));
      MovementType new_type = static_cast<MovementType>(i);

      if (!collision_checking.InCollision(new_x, new_x, new_theta)) {
        HybridState4DNode NewNode =
            HybridState4DNode(new_x, new_y, new_theta, new_type);
        astarsearch->AddSuccessor(NewNode);
      }
    }

    return true;
  }  // GetSuccessors

  // (g value) given this node, what does it cost to move to successor.
  // In the case of our map the answer is the map terrain value at this node
  // since that is conceptually where we're moving
  float GetCost(const HybridState4DNode &successor,
                const SearchConfig &_SearchConfig) {
    int current_type = static_cast<int>(this->type_);
    int successor_type = static_cast<int>(successor.type());
    return _SearchConfig.cost_map[current_type][successor_type];

    // return (successor.x() - x_) * (successor.x() - x_) +
    //        (successor.y() - y_) * (successor.y() - y_);

  }  // GetCost

  bool IsSameState(const HybridState4DNode &rhs) {
    // same state in a maze search is simply when (x,y) are the same
    if ((std::abs(x_ - rhs.x()) <= 0.05) && (std::abs(y_ - rhs.y()) < 0.05) &&
        (std::abs(ASV::common::math::fNormalizeheadingangle(
             theta_ - rhs.theta())) < 0.01)) {
      return true;
    }
    return false;
  }  // IsSameState

  void PrintNodeInfo() {
    std::cout << "Node position : " << x_ << ", " << y_;
  }  // PrintNodeInfo

 private:
  float x_;  // the (x,y) positions of the node
  float y_;
  float theta_;
  MovementType type_;

  Eigen::Matrix<float, 3, 6> update_movement_step(float L, float varphi,
                                                  float theta) {
    Eigen::Matrix<float, 3, 6> movement_step =
        Eigen::Matrix<float, 3, 6>::Zero();

    float L_cos_theta = L * std::cos(theta);
    float L_sin_theta = L * std::sin(theta);
    float sin_varphi = std::sin(varphi) / varphi;
    float one_cos_varphi = (1.0 - std::cos(varphi)) / varphi;
    // S+
    movement_step(0, 0) = L_cos_theta;
    movement_step(1, 0) = L_sin_theta;
    movement_step(2, 0) = 0;
    // L+
    movement_step(0, 1) =
        L_cos_theta * sin_varphi - L_sin_theta * one_cos_varphi;
    movement_step(1, 1) =
        L_sin_theta * sin_varphi + L_cos_theta * one_cos_varphi;
    movement_step(2, 1) = varphi;
    // R+
    movement_step(0, 2) =
        L_cos_theta * sin_varphi + L_sin_theta * one_cos_varphi;
    movement_step(1, 2) =
        L_sin_theta * sin_varphi - L_cos_theta * one_cos_varphi;
    movement_step(2, 2) = -varphi;

    // reverse
    for (int i = 3; i != 6; ++i) {
      movement_step(0, i) = -movement_step(0, i - 3);
      movement_step(1, i) = -movement_step(1, i - 3);
      movement_step(2, i) = movement_step(2, i - 3);
    }

    return movement_step;
  }  // update_movement_step

};  // end class HybridState4DNode

class HybridAStar {
  using HybridAStar_Search =
      HybridAStarSearch<HybridState4DNode, SearchConfig, CollisionChecking>;

 public:
  HybridAStar(const CollisionData &collisiondata,
              const HybridAStarConfig &hybridastarconfig)
      : collision_checker_(collisiondata),
        rscurve_(1.0 / collisiondata.MAX_CURVATURE) {
    searchconfig_ = GenerateSearchConfig(collisiondata, hybridastarconfig);
  }
  virtual ~HybridAStar() = default;

  HybridAStar &update_obstacles(
      const std::vector<Obstacle_Vertex> &Obstacles_Vertex,
      const std::vector<Obstacle_LineSegment> &Obstacles_LineSegment,
      const std::vector<Obstacle_Box2d> &Obstacles_Box2d) {
    collision_checker_.set_Obstacles_Vertex(Obstacles_Vertex)
        .set_Obstacles_LineSegment(Obstacles_LineSegment)
        .set_Obstacles_Box2d(Obstacles_Box2d);
    return *this;
  }  // update_obstacles

  // update the start and ending points
  void setup_start_end(const float start_x, const float start_y,
                       const float start_theta, const float end_x,
                       const float end_y, const float end_theta) {
    HybridState4DNode nodeStart(start_x, start_y, start_theta);
    HybridState4DNode nodeEnd(end_x, end_y, end_theta);
    astarsearch_.SetStartAndGoalStates(nodeStart, nodeEnd);
  }

  auto hybridastar_trajecotry() const { return hybridastar_trajecotry_; }

  void performsearch() {
    unsigned int SearchState;
    unsigned int SearchSteps = 0;

    do {
      SearchState = astarsearch_.SearchStep(searchconfig_, collision_checker_);

      SearchSteps++;

      if constexpr (DEBUG_LISTS == 1) {
        std::cout << "Steps:" << SearchSteps << "\n";

        int len = 0;

        std::cout << "Open:\n";
        HybridState4DNode *p = astarsearch_.GetOpenListStart();
        while (p) {
          len++;
          if constexpr (DEBUG_LIST_LENGTHS_ONLY == 0)
            ((HybridState4DNode *)p)->PrintNodeInfo();
          p = astarsearch_.GetOpenListNext();
        }

        std::cout << "Open list has " << len << " nodes\n";

        len = 0;

        std::cout << "Closed:\n";
        p = astarsearch_.GetClosedListStart();
        while (p) {
          len++;
          if constexpr (DEBUG_LIST_LENGTHS_ONLY == 0) p->PrintNodeInfo();
          p = astarsearch_.GetClosedListNext();
        }

        std::cout << "Closed list has " << len << " nodes\n";
      }

    } while (SearchState == HybridAStar_Search::SEARCH_STATE_SEARCHING);

    if (SearchState == HybridAStar_Search::SEARCH_STATE_SUCCEEDED) {
      std::cout << "Search found goal state\n";

      HybridState4DNode *node = astarsearch_.GetSolutionStart();

      std::cout << "Displaying solution\n";

      while (node) {
        node->PrintNodeInfo();
        hybridastar_trajecotry_.push_back(
            {node->x(), node->y(), node->theta()});
        node = astarsearch_.GetSolutionNext();
      }

      // Once you're done with the solution you can free the nodes up
      astarsearch_.FreeSolutionNodes();
    } else if (SearchState == HybridAStar_Search::SEARCH_STATE_FAILED) {
      std::cout << "Search terminated. Did not find goal state\n";
    }

    // Display the number of loops the search went through
    std::cout << "SearchSteps : " << SearchSteps << "\n";

    astarsearch_.EnsureMemoryFreed();

  }  // performsearch

 private:
  CollisionChecking collision_checker_;
  ASV::common::math::ReedsSheppStateSpace rscurve_;

  SearchConfig searchconfig_;
  HybridAStar_Search astarsearch_;
  std::vector<std::array<double, 3>> hybridastar_trajecotry_;

  SearchConfig GenerateSearchConfig(
      const CollisionData &collisiondata,
      const HybridAStarConfig &hybridastarconfig) {
    SearchConfig searchconfig;

    float L = hybridastarconfig.move_length;
    float Ct = hybridastarconfig.penalty_turning;
    float Cr = hybridastarconfig.penalty_reverse;
    float Cs = hybridastarconfig.penalty_switch;

    searchconfig.move_length = L;
    searchconfig.turning_angle = L * collisiondata.MAX_CURVATURE;
    //
    for (int i = 0; i != 3; ++i) {
      for (int j = 0; j != 3; ++j) {
        if (i == j)
          searchconfig.cost_map[i][j] = L;
        else
          searchconfig.cost_map[i][j] = L * Ct;
      }
      for (int j = 3; j != 6; ++j)
        searchconfig.cost_map[i][j] = L * Ct * Cr * Cs;
    }
    for (int i = 3; i != 6; ++i) {
      for (int j = 0; j != 3; ++j) searchconfig.cost_map[i][j] = L * Ct * Cs;
      for (int j = 3; j != 6; ++j) {
        if (i == j)
          searchconfig.cost_map[i][j] = L * Cr;
        else
          searchconfig.cost_map[i][j] = L * Cr * Ct;
      }
    }

    return searchconfig;
  }  // GenerateSearchConfig

};  // end class HybridAStar

}  // namespace ASV::planning

#endif /* _HYBRIDASTAR_H_ */