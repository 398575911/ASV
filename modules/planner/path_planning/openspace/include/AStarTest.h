

#ifndef _ASTAR_H_
#define _ASTAR_H_

#include "modules/planner/common/include/stlastar.h"

namespace ASV::planning {

class HybridState4DNode {
 public:
  HybridState4DNode() : x_(0), y_(0) {}
  HybridState4DNode(int px, int py) : x_(px), y_(py) {}
  // Here's the heuristic function that estimates the distance from a Node
  // to the Goal.
  float GoalDistanceEstimate(HybridState4DNode &nodeGoal) {
    // return abs(x - nodeGoal.x) + abs(y - nodeGoal.y);
    return (x_ - nodeGoal.x()) * (x_ - nodeGoal.x()) +
           (y_ - nodeGoal.y()) * (y_ - nodeGoal.y());
    // return 0;
  }  // GoalDistanceEstimate

  bool IsGoal(HybridState4DNode &nodeGoal) {
    if ((x_ == nodeGoal.x()) && (y_ == nodeGoal.y())) {
      return true;
    }

    return false;
  }  // IsGoal

  // This generates the successors to the given Node. It uses a helper function
  // called AddSuccessor to give the successors to the AStar class. The A*
  // specific initialisation is done for each node internally, so here you just
  // set the state information that is specific to the application
  bool GetSuccessors(AStarSearch<HybridState4DNode> *astarsearch,
                     HybridState4DNode *parent_node) {
    int parent_x = -1;
    int parent_y = -1;

    if (parent_node) {
      parent_x = parent_node->x();
      parent_y = parent_node->y();
    }

    // push each possible move except allowing the search to go backwards
    // if ((GetMap(x_ - 1, y_) < 9) &&
    //     !((parent_x == x_ - 1) && (parent_y == y_))) {
    //   HybridState4DNode NewNode = HybridState4DNode(x_ - 1, y_);
    //   astarsearch->AddSuccessor(NewNode);
    // }

    // if ((GetMap(x_, y_ - 1) < 9) &&
    //     !((parent_x == x_) && (parent_y == y_ - 1))) {
    //   HybridState4DNode NewNode = HybridState4DNode(x_, y_ - 1);
    //   astarsearch->AddSuccessor(NewNode);
    // }

    // if ((GetMap(x_ + 1, y_) < 9) &&
    //     !((parent_x == x_ + 1) && (parent_y == y_))) {
    //   HybridState4DNode NewNode = HybridState4DNode(x_ + 1, y_);
    //   astarsearch->AddSuccessor(NewNode);
    // }

    // if ((GetMap(x_, y_ + 1) < 9) &&
    //     !((parent_x == x_) && (parent_y == y_ + 1))) {
    //   HybridState4DNode NewNode = HybridState4DNode(x_, y_ + 1);
    //   astarsearch->AddSuccessor(NewNode);
    // }

    if (GetMap(x_ - 1, y_) < 9) {
      HybridState4DNode NewNode = HybridState4DNode(x_ - 1, y_);
      astarsearch->AddSuccessor(NewNode);
    }

    if (GetMap(x_, y_ - 1) < 9) {
      HybridState4DNode NewNode = HybridState4DNode(x_, y_ - 1);
      astarsearch->AddSuccessor(NewNode);
    }

    if (GetMap(x_ + 1, y_) < 9) {
      HybridState4DNode NewNode = HybridState4DNode(x_ + 1, y_);
      astarsearch->AddSuccessor(NewNode);
    }

    if (GetMap(x_, y_ + 1) < 9) {
      HybridState4DNode NewNode = HybridState4DNode(x_, y_ + 1);
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
    if ((x_ == rhs.x()) && (y_ == rhs.y())) {
      return true;
    } else {
      return false;
    }
  }  // IsSameState

  void PrintNodeInfo() {
    char str[100];
    sprintf(str, "Node position : (%d,%d)\n", x_, y_);

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

  int x() const { return x_; }
  int y() const { return y_; }

 private:
  int x_;  // the (x,y) positions of the node
  int y_;

};  // namespace ASV::planningclassHybridState4DNode

}  // namespace ASV::planning

#endif /* _HYBRIDASTAR_H_ */