/////////////////////////////////////////////////////////////////////////

// Hybrid A* Search implementation
// (C)2001 Justin Heyes-Jones
//
// Finding a path on a simple grid maze
// This shows how to do shortest path finding using A*

//////////////////////////////////////////////////////////////////////////

#include "common/plotting/include/gnuplot-iostream.h"

#include "../include/AStarTest.h"

#include <math.h>
#include <stdio.h>
#include <iostream>

constexpr int DEBUG_LISTS = 0;
constexpr int DEBUG_LIST_LENGTHS_ONLY = 0;

using namespace ASV::planning;

// Main
int main() {
  std::cout << "STL A* Search implementation\n";

  // Our sample problem defines the world as a 2d array representing a terrain
  // Each element contains an integer from 0 to 5 which indicates the cost
  // of travel across the terrain. Zero means the least possible difficulty
  // in travelling (think ice rink if you can skate) whilst 5 represents the
  // most difficult. 9 indicates that we cannot pass.

  // Create an instance of the search class...
  AStarSearch<HybridState4DNode> astarsearch;

  external_foo _external_foo(1);
  // Create a start state
  HybridState4DNode nodeStart(5, 6);
  nodeStart.setfoo(&_external_foo);

  // Define the goal state
  HybridState4DNode nodeEnd(11, 24);

  // nodeEnd.setfoo(&_external_foo);

  // Set Start and goal states

  astarsearch.SetStartAndGoalStates(nodeStart, nodeEnd);

  unsigned int SearchState;
  unsigned int SearchSteps = 0;

  do {
    SearchState = astarsearch.SearchStep();

    SearchSteps++;

    if constexpr (DEBUG_LISTS == 1) {
      std::cout << "Steps:" << SearchSteps << "\n";

      int len = 0;

      std::cout << "Open:\n";
      HybridState4DNode *p = astarsearch.GetOpenListStart();
      while (p) {
        len++;
        if constexpr (DEBUG_LIST_LENGTHS_ONLY == 0)
          ((HybridState4DNode *)p)->PrintNodeInfo();
        p = astarsearch.GetOpenListNext();
      }

      std::cout << "Open list has " << len << " nodes\n";

      len = 0;

      std::cout << "Closed:\n";
      p = astarsearch.GetClosedListStart();
      while (p) {
        len++;
        if constexpr (DEBUG_LIST_LENGTHS_ONLY == 0) p->PrintNodeInfo();
        p = astarsearch.GetClosedListNext();
      }

      std::cout << "Closed list has " << len << " nodes\n";
    }

  } while (SearchState ==
           AStarSearch<HybridState4DNode>::SEARCH_STATE_SEARCHING);

  if (SearchState == AStarSearch<HybridState4DNode>::SEARCH_STATE_SUCCEEDED) {
    std::cout << "Search found goal state\n";

    HybridState4DNode *node = astarsearch.GetSolutionStart();

    std::vector<std::pair<double, double>> x_y_ps;
    std::vector<std::pair<double, double>> x_y_startend;

    std::cout << "Displaying solution\n";

    while (node) {
      node->PrintNodeInfo();
      x_y_ps.push_back(std::make_pair(node->x(), node->y()));
      node = astarsearch.GetSolutionNext();
    }

    x_y_startend.push_back(std::make_pair(nodeStart.x(), nodeStart.y()));
    x_y_startend.push_back(std::make_pair(nodeEnd.x(), nodeEnd.y()));

    // Once you're done with the solution you can free the nodes up
    astarsearch.FreeSolutionNodes();

    int MAP_WIDTH = 20;
    int MAP_HEIGHT = 30;

    Gnuplot gp;
    gp << "set terminal x11 size 1100, 1200 0\n";
    gp << "set title 'A star search'\n";
    gp << "set tic scale 0\n";
    gp << "set palette rgbformula -7,2,-7\n";  // rainbow
    gp << "set cbrange [0:9]\n";
    gp << "set cblabel 'obstacle'\n";
    gp << "unset cbtics\n";
    gp << "set xrange [0:" << MAP_WIDTH << "]\n";
    gp << "set yrange [0:" << MAP_HEIGHT << "]\n";

    std::vector<std::tuple<double, double, int>> x_y_z;
    for (int i = 0; i != MAP_WIDTH; ++i)
      for (int j = 0; j != MAP_HEIGHT; ++j)
        x_y_z.push_back({i, j, nodeStart.GetMap(i, j)});
    gp << "plot " << gp.file1d(x_y_z) << " with image notitle, "
       << gp.file1d(x_y_startend) << " with points pointtype 7 notitle,"
       << gp.file1d(x_y_ps) << " with points lc rgb 'black' title 'path'\n";

  } else if (SearchState ==
             AStarSearch<HybridState4DNode>::SEARCH_STATE_FAILED) {
    std::cout << "Search terminated. Did not find goal state\n";
  }

  // Display the number of loops the search went through
  std::cout << "SearchSteps : " << SearchSteps << "\n";

  astarsearch.EnsureMemoryFreed();

  return 0;
}