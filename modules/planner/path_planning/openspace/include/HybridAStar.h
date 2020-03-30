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
#include "modules/planner/common/include/stlastar.h"
#include "openspacedata.h"

namespace ASV::planning {

constexpr int DEBUG_LISTS = 0;
constexpr int DEBUG_LIST_LENGTHS_ONLY = 0;

class HybridState4DNode {
 public:
  HybridState4DNode()
      : x_(0.0),
        y_(0.0),
        theta_(0.0),
        r_(0),
        CollisionChecking_(nullptr),
        ReedsSheppStateSpace_(nullptr) {}
  HybridState4DNode(double px, double py, double ptheta, int pr)
      : x_(px),
        y_(py),
        theta_(ptheta),
        r_(pr),
        CollisionChecking_(nullptr),
        ReedsSheppStateSpace_(nullptr) {}

  double x() const noexcept { return x_; }
  double y() const noexcept { return y_; }
  double theta() const noexcept { return theta_; }
  int r() const noexcept { return r_; }

  void set_collision(CollisionChecking *_CollisionChecking) {
    this->CollisionChecking_ = _CollisionChecking;
  }

  void set_rscurve(
      ASV::common::math::ReedsSheppStateSpace *_ReedsSheppStateSpace) {
    this->ReedsSheppStateSpace_ = _ReedsSheppStateSpace;
  }

  // Here's the heuristic function that estimates the distance from a Node
  // to the Goal.
  float GoalDistanceEstimate(HybridState4DNode &nodeGoal) {
    // return abs(x - nodeGoal.x) + abs(y - nodeGoal.y);
    return (x_ - nodeGoal.x()) * (x_ - nodeGoal.x()) +
           (y_ - nodeGoal.y()) * (y_ - nodeGoal.y());
    // return 0;
  }  // GoalDistanceEstimate

  bool IsGoal(HybridState4DNode &nodeGoal) {
    if (std::hypot(x_ - nodeGoal.x(), y_ - nodeGoal.y()) < 0.1) {
      return true;
    }

    return false;
  }  // IsGoal

  // This generates the successors to the given Node. It uses a helper function
  // called AddSuccessor to give the successors to the AStar class. The A*
  // specific initialisation is done for each node internally, so here you just
  // set the state information that is specific to the application
  bool GetSuccessors(ASV::planning::AStarSearch<HybridState4DNode> *astarsearch,
                     HybridState4DNode *parent_node) {
    double parent_x = -1.0;
    double parent_y = -1.0;

    if (parent_node) {
      parent_x = parent_node->x();
      parent_y = parent_node->y();
    }

    // push each possible move except allowing the search to go backwards

    int x_int = static_cast<int>(x_);
    int y_int = static_cast<int>(y_);

    if (GetMap(x_int - 1, y_int) < 9) {
      HybridState4DNode NewNode = HybridState4DNode(x_int - 1, y_int, 0, 0);
      NewNode.set_collision(this->CollisionChecking_);
      NewNode.set_rscurve(this->ReedsSheppStateSpace_);
      astarsearch->AddSuccessor(NewNode);
    }

    if (GetMap(x_int, y_int - 1) < 9) {
      HybridState4DNode NewNode = HybridState4DNode(x_int, y_int - 1, 0, 0);
      NewNode.set_collision(this->CollisionChecking_);
      NewNode.set_rscurve(this->ReedsSheppStateSpace_);
      astarsearch->AddSuccessor(NewNode);
    }

    if (GetMap(x_int + 1, y_int) < 9) {
      HybridState4DNode NewNode = HybridState4DNode(x_int + 1, y_int, 0, 0);
      NewNode.set_collision(this->CollisionChecking_);
      NewNode.set_rscurve(this->ReedsSheppStateSpace_);
      astarsearch->AddSuccessor(NewNode);
    }

    if (GetMap(x_int, y_int + 1) < 9) {
      HybridState4DNode NewNode = HybridState4DNode(x_int, y_int + 1, 0, 0);
      NewNode.set_collision(this->CollisionChecking_);
      NewNode.set_rscurve(this->ReedsSheppStateSpace_);
      astarsearch->AddSuccessor(NewNode);
    }

    return true;
  }  // GetSuccessors

  // (g value) given this node, what does it cost to move to successor.
  // In the case of our map the answer is the map terrain value at this node
  // since that is conceptually where we're moving
  float GetCost(HybridState4DNode &successor) {
    // return (float)GetMap(successor.x, successor.y);
    // return (float)GetMap(x, y);
    return (successor.x() - x_) * (successor.x() - x_) +
           (successor.y() - y_) * (successor.y() - y_);
    // return 1;
  }  // GetCost

  bool IsSameState(HybridState4DNode &rhs) {
    // same state in a maze search is simply when (x,y) are the same
    if (std::hypot(x_ - rhs.x(), y_ - rhs.y()) < 0.1) {
      return true;
    } else {
      return false;
    }
  }  // IsSameState

  void PrintNodeInfo() {
    char str[100];
    sprintf(str, "Node position : (%f,%f)\n", x_, y_);

    std::cout << str;
  }  // PrintNodeInfo

  static int GetMap(int x, int y) {
    constexpr static int _MAP_WIDTH = 20;
    constexpr static int _MAP_HEIGHT = 30;

    constexpr static int world_map[_MAP_WIDTH * _MAP_HEIGHT] = {

        // 00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 00
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 9, 1,  // 01
        1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 9, 1, 1,  // 02
        1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 9, 1, 1,  // 03
        1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 9, 1, 1,  // 04
        1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1,  // 05
        1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1,  // 06
        1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 9, 9, 1,  // 07
        1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1,  // 08
        1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 9, 1, 1, 1, 9, 1, 1, 9, 9, 1,  // 09
        1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1,  // 10
        1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1,  // 11
        1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 9, 1, 1,  // 12
        1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 9, 1, 1,  // 13
        1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 9, 1, 1,  // 14
        1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1,  // 15
        1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 9, 9, 9, 1, 1, 1, 1, 1,  // 16
        1, 1, 1, 1, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 1, 1, 9, 9, 9,  // 17
        1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1,  // 18
        1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 9, 1,  // 19
        1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 9, 9, 9, 1,  // 20
        1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1,  // 21
        9, 9, 9, 9, 9, 9, 1, 1, 1, 1, 9, 9, 9, 9, 9, 9, 9, 1, 1, 1,  // 22
        1, 1, 9, 1, 1, 1, 1, 9, 9, 9, 9, 1, 1, 9, 1, 1, 1, 1, 1, 1,  // 23
        1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 24
        1, 9, 9, 9, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1,  // 25
        1, 1, 9, 1, 1, 1, 1, 9, 9, 9, 9, 9, 9, 9, 9, 1, 1, 1, 9, 1,  // 26
        1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1,  // 27
        1, 1, 1, 1, 1, 1, 1, 9, 9, 9, 9, 9, 9, 1, 1, 1, 1, 1, 9, 1,  // 28
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 29

    };

    // int world_map[MAP_WIDTH * MAP_HEIGHT] = {

    //     // 00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19
    //     1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 00
    //     1, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 1,  // 01
    //     1, 9, 9, 1, 1, 9, 9, 9, 1, 9, 1, 9, 1, 9, 1, 9, 9, 9, 1, 1,  // 02
    //     1, 9, 9, 1, 1, 9, 9, 9, 1, 9, 1, 9, 1, 9, 1, 9, 9, 9, 1, 1,  // 03
    //     1, 9, 1, 1, 1, 1, 9, 9, 1, 9, 1, 9, 1, 1, 1, 1, 9, 9, 1, 1,  // 04
    //     1, 9, 1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1,  // 05
    //     1, 9, 9, 9, 9, 1, 1, 1, 1, 1, 1, 9, 9, 9, 9, 1, 1, 1, 1, 1,  // 06
    //     1, 9, 9, 9, 9, 9, 9, 9, 9, 1, 1, 1, 9, 9, 9, 9, 9, 9, 9, 1,  // 07
    //     1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1,  // 08
    //     1, 9, 1, 9, 9, 9, 9, 9, 9, 9, 1, 1, 9, 9, 9, 9, 9, 9, 9, 1,  // 09
    //     1, 9, 1, 1, 1, 1, 9, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 10
    //     1, 9, 9, 9, 9, 9, 1, 9, 1, 9, 1, 9, 9, 9, 9, 9, 1, 1, 1, 1,  // 11
    //     1, 9, 1, 9, 1, 9, 9, 9, 1, 9, 1, 9, 1, 9, 1, 9, 9, 9, 1, 1,  // 12
    //     1, 9, 1, 9, 1, 9, 9, 9, 1, 9, 1, 9, 1, 9, 1, 9, 9, 9, 1, 1,  // 13
    //     1, 9, 1, 1, 1, 1, 9, 9, 1, 9, 1, 9, 1, 1, 1, 1, 9, 9, 1, 1,  // 14
    //     1, 9, 1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1,  // 15
    //     1, 9, 9, 9, 9, 1, 1, 1, 1, 1, 1, 9, 9, 9, 9, 1, 1, 1, 1, 1,  // 16
    //     1, 1, 9, 9, 9, 9, 9, 9, 9, 1, 1, 1, 9, 9, 9, 1, 9, 9, 9, 9,  // 17
    //     1, 9, 1, 1, 1, 9, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1,  // 18
    //     1, 9, 1, 1, 1, 9, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 9, 1,  // 19
    //     1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 9, 9, 9, 1,  // 20
    //     1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1,  // 21
    //     9, 9, 9, 9, 9, 9, 1, 1, 1, 1, 9, 9, 9, 9, 9, 9, 9, 1, 1, 1,  // 22
    //     1, 1, 9, 1, 1, 1, 1, 9, 9, 9, 9, 1, 1, 9, 1, 1, 1, 1, 1, 1,  // 23
    //     1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 24
    //     1, 9, 9, 9, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1,  // 25
    //     1, 1, 9, 1, 1, 1, 1, 9, 9, 9, 9, 9, 9, 9, 9, 1, 1, 1, 9, 1,  // 26
    //     1, 1, 9, 1, 1, 1, 1, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 1,  // 27
    //     1, 1, 1, 1, 1, 1, 1, 9, 9, 9, 9, 9, 9, 1, 1, 1, 1, 1, 9, 1,  // 28
    //     1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 29

    // };

    if (x < 0 || x >= _MAP_WIDTH || y < 0 || y >= _MAP_HEIGHT) {
      return 9;
    }

    return world_map[(y * _MAP_WIDTH) + x];
  }  // GetMap

 private:
  double x_;  // the (x,y) positions of the node
  double y_;
  double theta_;
  int r_;

  CollisionChecking *CollisionChecking_;
  ASV::common::math::ReedsSheppStateSpace *ReedsSheppStateSpace_;

  void testPrivate() {}
};  // end class HybridState4DNode

class HybridAStar {
 public:
  HybridAStar(const CollisionData &collisiondata)
      : nodeStart_(5, 6, 0, 0),
        nodeEnd_(11, 24, 0, 0),
        collision_checker_(collisiondata),
        rscurve_(3) {
    nodeStart_.set_collision(&collision_checker_);
    nodeStart_.set_rscurve(&rscurve_);
    astarsearch_.SetStartAndGoalStates(nodeStart_, nodeEnd_);
  }
  virtual ~HybridAStar() = default;

  auto results() const { return results_; }

  void performsearch() {
    unsigned int SearchState;
    unsigned int SearchSteps = 0;

    do {
      SearchState = astarsearch_.SearchStep();

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

    } while (SearchState ==
             AStarSearch<HybridState4DNode>::SEARCH_STATE_SEARCHING);

    if (SearchState == AStarSearch<HybridState4DNode>::SEARCH_STATE_SUCCEEDED) {
      std::cout << "Search found goal state\n";

      HybridState4DNode *node = astarsearch_.GetSolutionStart();

      std::cout << "Displaying solution\n";

      while (node) {
        node->PrintNodeInfo();
        results_.push_back({node->x(), node->y(), node->theta()});
        node = astarsearch_.GetSolutionNext();
      }

      // Once you're done with the solution you can free the nodes up
      astarsearch_.FreeSolutionNodes();
    } else if (SearchState ==
               AStarSearch<HybridState4DNode>::SEARCH_STATE_FAILED) {
      std::cout << "Search terminated. Did not find goal state\n";
    }

    // Display the number of loops the search went through
    std::cout << "SearchSteps : " << SearchSteps << "\n";

    astarsearch_.EnsureMemoryFreed();

  }  // performsearch

 private:
  HybridState4DNode nodeStart_;
  HybridState4DNode nodeEnd_;
  CollisionChecking collision_checker_;
  ASV::common::math::ReedsSheppStateSpace rscurve_;
  AStarSearch<HybridState4DNode> astarsearch_;
  std::vector<std::tuple<double, double, double>> results_;

};  // end class HybridAStar

}  // namespace ASV::planning

#endif /* _HYBRIDASTAR_H_ */