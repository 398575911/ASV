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
#include "modules/planner/common/include/stlastar.h"
#include "openspacedata.h"

namespace ASV::planning {

class HybridAStar {};

class HybridState4DNode {
 public:
  double x;  // the (x,y) positions of the node
  double y;

  HybridState4DNode() : x(0), y(0) {}
  HybridState4DNode(int px, int py) : x(px), y(py) {}
  // Here's the heuristic function that estimates the distance from a Node
  // to the Goal.
  float GoalDistanceEstimate(HybridState4DNode &nodeGoal) {
    // return abs(x - nodeGoal.x) + abs(y - nodeGoal.y);
    return (x - nodeGoal.x) * (x - nodeGoal.x) +
           (y - nodeGoal.y) * (y - nodeGoal.y);
    // return 0;
  }  // GoalDistanceEstimate

  bool IsGoal(HybridState4DNode &nodeGoal) {
    if ((x == nodeGoal.x) && (y == nodeGoal.y)) {
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
    int parent_x = -1;
    int parent_y = -1;

    if (parent_node) {
      parent_x = parent_node->x;
      parent_y = parent_node->y;
    }

    HybridState4DNode NewNode;

    // push each possible move except allowing the search to go backwards

    if ((GetMap(x - 1, y) < 9) && !((parent_x == x - 1) && (parent_y == y))) {
      NewNode = HybridState4DNode(x - 1, y);
      astarsearch->AddSuccessor(NewNode);
    }

    if ((GetMap(x, y - 1) < 9) && !((parent_x == x) && (parent_y == y - 1))) {
      NewNode = HybridState4DNode(x, y - 1);
      astarsearch->AddSuccessor(NewNode);
    }

    if ((GetMap(x + 1, y) < 9) && !((parent_x == x + 1) && (parent_y == y))) {
      NewNode = HybridState4DNode(x + 1, y);
      astarsearch->AddSuccessor(NewNode);
    }

    if ((GetMap(x, y + 1) < 9) && !((parent_x == x) && (parent_y == y + 1))) {
      NewNode = HybridState4DNode(x, y + 1);
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
    return (successor.x - x) * (successor.x - x) +
           (successor.y - y) * (successor.y - y);
    // return 1;
  }  // GetCost

  bool IsSameState(HybridState4DNode &rhs) {
    // same state in a maze search is simply when (x,y) are the same
    if ((x == rhs.x) && (y == rhs.y)) {
      return true;
    } else {
      return false;
    }
  }  // IsSameState

  void PrintNodeInfo() {
    char str[100];
    sprintf(str, "Node position : (%d,%d)\n", x, y);

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
  void testPrivate() {}
};  // end class HybridState4DNode

}  // namespace ASV::planning

#endif /* _HYBRIDASTAR_H_ */