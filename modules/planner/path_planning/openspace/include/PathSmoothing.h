/*
***********************************************************************
* PathSmoothing.h:
* improve the smoothness of path, using conjugate-gradeient descent
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _PATHSMOOTHING_H_
#define _PATHSMOOTHING_H_

#include <iostream>
#include <pyclustering/container/kdtree.hpp>
#include "openspacedata.h"
namespace ASV::planning {

class PathSmoothing {
 public:
  PathSmoothing() = default;
  ~PathSmoothing() = default;
  void test() {
    std::vector<std::vector<double>> test_sample_point_vector = {
        {4, 3}, {3, 4}, {5, 8}, {3, 3}, {3, 9}, {6, 4}, {5, 9}};

    std::vector<double> p_data = {-1, -1};

    const double radius_search = 10.0;

    // kd tree
    pyclustering::container::kdtree tree;
    for (auto &point : test_sample_point_vector) {
      tree.insert(point);
    }

    // kd search
    pyclustering::container::kdtree_searcher searcher(p_data, tree.get_root(),
                                                      radius_search);

    // FindNearestNode
    pyclustering::container::kdnode::ptr nearest_node =
        searcher.find_nearest_node();
    auto nearest_data = nearest_node->get_data();
    std::cout << "NearestNode\n";
    for (const auto &value : nearest_data) std::cout << value << std::endl;

    // FindNearestNeighbors
    std::vector<double> nearest_distances;
    std::vector<pyclustering::container::kdnode::ptr> nearest_nodes;
    searcher.find_nearest_nodes(nearest_distances, nearest_nodes);
    std::cout << "FindNearestNeighbors\n";
    for (std::size_t index = 0; index != nearest_distances.size(); index++) {
      auto each_node = nearest_nodes[index]->get_data();
      for (const auto &value : each_node) std::cout << value << " ";
      std::cout << ": " << nearest_distances[index] << std::endl;
    }
  }

 private:
};  // end class PathSmoothing
}  // namespace ASV::planning

#endif /* _PATHSMOOTHING_H_ */