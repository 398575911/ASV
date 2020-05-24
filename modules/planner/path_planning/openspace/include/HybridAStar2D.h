/*
*******************************************************************************
* HybridAStar2D.h:
* Hybrid A star algorithm, to generate the coarse collision-free trajectory
* This header file can be read by C++ compilers
*
* by Hu.ZH, Bin Lin(CrossOcean.ai)
*******************************************************************************
*/

#ifndef _HYBRIDASTAR2D_H_
#define _HYBRIDASTAR2D_H_

#include "CollisionChecking.h"
#include "hybridstlastar.h"
#include "openspacedata.h"

#include "common/logging/include/easylogging++.h"
#include "common/math/Geometry/include/Reeds_Shepp.h"

namespace ASV::planning {

// 2d node for search a collison-free path
class HybridState2DNode {
  using HybridAStar_Search =
      HybridAStarSearch<HybridState2DNode, SearchConfig2D,
                        CollisionChecking_Astar>;

 public:
  HybridState2DNode() : x_(0), y_(0), theta_(0) {}

  HybridState2DNode(float px, float py, float ptheta)
      : x_(px), y_(py), theta_(ptheta) {}

  float x() const noexcept { return x_; }
  float y() const noexcept { return y_; }
  float theta() const noexcept { return theta_; }

  // Here's the heuristic function that estimates the distance from a Node
  // to the Goal.
  float GoalDistanceEstimate(const HybridState2DNode &nodeGoal,
                             std::nullptr_t) {
    return std::abs(x_ - nodeGoal.x()) + std::abs(y_ - nodeGoal.y());
    // return std::hypot(x_ - nodeGoal.x(), y_ - nodeGoal.y());

  }  // GoalDistanceEstimate

  bool IsGoal(const HybridState2DNode &nodeGoal,
              const SearchConfig2D &search_config) {
    return ((std::abs(x_ - nodeGoal.x()) <= 0.5) &&
            (std::abs(y_ - nodeGoal.y()) <= 0.5));
  }  // IsGoal

  // This generates the successors to the given Node.
  bool GetSuccessors(HybridAStar_Search *astarsearch,
                     HybridState2DNode *parent_node,
                     const SearchConfig2D &search_config,
                     const CollisionChecking_Astar &collisionchecker) {
    constexpr static int _DIRECTION_NUM = 8;
    constexpr static int _DIMENSION = 2;
    constexpr static int move_direction[_DIRECTION_NUM][_DIMENSION] = {
        {0, 1},    // move up
        {1, 1},    // move up-right
        {1, 0},    // move right
        {1, -1},   // move right-down
        {0, -1},   // move down
        {-1, -1},  // move down-left
        {-1, 0},   // move left
        {-1, 1},   // move left-up
    };

    (void)parent_node;

    float L = search_config.move_length;
    float max_turn = search_config.turning_angle;

    // push each possible move except allowing the search to go backwards
    for (int i = 0; i != _DIRECTION_NUM; ++i) {
      float new_x = this->x_ + L * move_direction[i][0];
      float new_y = this->y_ + L * move_direction[i][1];

      for (int j = -2; j != 3; j++) {
        float new_theta = ASV::common::math::fNormalizeheadingangle(
            this->theta_ + 0.5 * max_turn * j);

        if (!collisionchecker.InCollision(new_x, new_y, new_theta)) {
          HybridState2DNode NewNode =
              HybridState2DNode(new_x, new_y, new_theta);
          astarsearch->AddSuccessor(NewNode);
        }
      }
    }

    return true;
  }  // GetSuccessors

  // (g value) given this node, what does it cost to move to successor.
  float GetCost(const HybridState2DNode &successor,
                const SearchConfig2D &search_config) {
    (void)search_config;
    return std::abs(successor.x() - x_) + std::abs(successor.y() - y_) +
           10 * std::abs(ASV::common::math::fNormalizeheadingangle(
                    successor.theta() - theta_));
  }  // GetCost

  bool IsSameState(const HybridState2DNode &rhs,
                   const SearchConfig2D &search_config) {
    // same state in a maze search is simply when (x,y) are the same
    return ((std::abs(x_ - rhs.x()) <= 0.05) &&
            (std::abs(y_ - rhs.y()) <= 0.05));

  }  // IsSameState

 private:
  // the index (x_,y_,theta_) of the node
  float x_;
  float y_;
  float theta_;

  bool Issamestate(const HybridState2DNode &lhs, const HybridState2DNode &rhs) {
    return ((std::abs(lhs.x() - rhs.x()) <= 0.05) &&
            (std::abs(lhs.y() - rhs.y()) <= 0.05));

  }  // Issamestate

};  // end class HybridState2DNode

class HybridAStar2D {
  using HybridAStar_2dNode_Search =
      HybridAStarSearch<HybridState2DNode, SearchConfig2D,
                        CollisionChecking_Astar>;

 public:
  HybridAStar2D(const SearchConfig2D &searchconfig)
      : searchconfig_(searchconfig), astar_2d_search_(3000) {}
  virtual ~HybridAStar2D() = default;

  // update the start and ending points
  HybridAStar2D &setup_2d_start_end(const float start_x, const float start_y,
                                    const float start_theta, const float end_x,
                                    const float end_y, const float end_theta) {
    HybridState2DNode nodeStart(start_x, start_y, start_theta);
    HybridState2DNode nodeEnd(end_x, end_y, end_theta);
    astar_2d_search_.SetStartAndGoalStates(nodeStart, nodeEnd);

    return *this;
  }  // setup_2d_start_end

  void perform_2dnode_search(const CollisionChecking_Astar &collision_checker) {
    unsigned int SearchState;
    unsigned int SearchSteps = 0;

    do {
      // perform a hybrid A* search
      SearchState =
          astar_2d_search_.SearchStep(searchconfig_, collision_checker);
      SearchSteps++;

      // get the current node
      std::vector<std::array<double, 3>> closedlist_trajecotry;
      HybridState2DNode *current_p = astar_2d_search_.GetCurrentNode();
      while (current_p) {
        closedlist_trajecotry.push_back(
            {current_p->x(), current_p->y(), current_p->theta()});
        current_p = astar_2d_search_.GetCurrentNodePrev();
      }
      std::reverse(closedlist_trajecotry.begin(), closedlist_trajecotry.end());

      // try a collision-free RS curve
      if (!closedlist_trajecotry.empty()) {
        hybridastar_2d_trajecotry_ = closedlist_trajecotry;
      }

    } while (SearchState == HybridAStar_2dNode_Search::SEARCH_STATE_SEARCHING);

    if (SearchState == HybridAStar_2dNode_Search::SEARCH_STATE_SUCCEEDED) {
      CLOG(INFO, "Hybrid_Astar") << "find 2d solution!";

      HybridState2DNode *node = astar_2d_search_.GetSolutionStart();
      hybridastar_2d_trajecotry_.clear();
      while (node) {
        hybridastar_2d_trajecotry_.push_back(
            {node->x(), node->y(), node->theta()});
        node = astar_2d_search_.GetSolutionNext();
      }
      // Once you're done with the solution you can free the nodes up
      astar_2d_search_.FreeSolutionNodes();
    }

    // Display the number of loops the search went through
    CLOG(INFO, "Hybrid_Astar") << "SearchSteps: " << SearchSteps;

    // astarsearch_.FreeSolutionNodes();
    astar_2d_search_.EnsureMemoryFreed();

  }  // perform_2dnode_search

  auto hybridastar_2dtrajecotry() const noexcept {
    return hybridastar_2d_trajecotry_;
  }  // hybridastar_2dtrajecotry

 private:
  SearchConfig2D searchconfig_;

  HybridAStar_2dNode_Search astar_2d_search_;

  std::vector<std::array<double, 3>> hybridastar_2d_trajecotry_;

};  // namespace ASV::planning

}  // namespace ASV::planning

#endif /* _HYBRIDASTAR2D_H_ */