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

// 2d node for search a collison-free path
class HybridState2DNode {
  using HybridAStar_Search =
      HybridAStarSearch<HybridState2DNode, SearchConfig,
                        std::array<std::array<std::array<bool, 161>, 161>, 72>>;

 public:
  HybridState2DNode() : x_(0), y_(0), theta_(0) {}

  HybridState2DNode(int px, int py, int ptheta)
      : x_(px), y_(py), theta_(ptheta) {}

  int x() const noexcept { return x_; }
  int y() const noexcept { return y_; }
  int theta() const noexcept { return theta_; }

  // Here's the heuristic function that estimates the distance from a Node
  // to the Goal.
  float GoalDistanceEstimate(const HybridState2DNode &nodeGoal,
                             std::nullptr_t) {
    return std::abs(x_ - nodeGoal.x()) + std::abs(y_ - nodeGoal.y());
  }  // GoalDistanceEstimate

  bool IsGoal(const HybridState2DNode &nodeGoal) {
    return IsSameState(nodeGoal);
  }  // IsGoal

  // This generates the successors to the given Node.
  bool GetSuccessors(HybridAStar_Search *astarsearch,
                     HybridState2DNode *parent_node,
                     const SearchConfig &search_config,
                     const std::array<std::array<std::array<bool, 161>, 161>,
                                      72> &collisionlookup) {
    constexpr static int _DIRECTION_NUM = 4;
    constexpr static int _DIMENSION = 2;
    constexpr static int move_direction[_DIRECTION_NUM][_DIMENSION] = {
        {0, 1},   // move up
        {1, 0},   // move right
        {0, -1},  // move down
        {-1, 0},  // move left
    };

    // push each possible move except allowing the search to go backwards
    for (int i = 0; i != _DIRECTION_NUM; ++i) {
      int new_x = this->x_ + move_direction[i][0];
      int new_y = this->y_ + move_direction[i][1];

      for (int j = -5; j != 6; j++) {
        int new_theta = Normalizeheading(this->theta_ + j);

        HybridState2DNode NewNode = HybridState2DNode(new_x, new_y, new_theta);

        if (0 <= new_x && new_x < 161 && 0 <= new_y && new_y < 161)
          if (!collisionlookup[new_theta][new_x][new_y]) {
            astarsearch->AddSuccessor(NewNode);
          }

        // if (parent_node) {
        //   if (!(collision_checking.InCollision(new_x, new_y, new_theta) ||
        //         Issamestate(*parent_node, NewNode))) {
        //     astarsearch->AddSuccessor(NewNode);
        //   }
        // } else {
        //   if (!collision_checking.InCollision(new_x, new_y, new_theta)) {
        //     astarsearch->AddSuccessor(NewNode);
        //   }
        // }
      }
    }

    return true;
  }  // GetSuccessors

  // (g value) given this node, what does it cost to move to successor.
  float GetCost(const HybridState2DNode &successor,
                const SearchConfig &search_config) {
    return search_config.move_length;
  }  // GetCost

  bool IsSameState(const HybridState2DNode &rhs) {
    // same state in a maze search is simply when (x,y) are the same
    if ((x_ == rhs.x()) && (y_ == rhs.y()) && (theta_ = rhs.theta())) {
      return true;
    }
    return false;
  }  // IsSameState

  void PrintNodeInfo() {
    std::cout << "Node position : " << x_ << ", " << y_ << ", " << theta_
              << std::endl;
  }  // PrintNodeInfo

 private:
  // the index (x_,y_,theta_) of the node
  int x_;
  int y_;
  int theta_;

  bool Issamestate(const HybridState2DNode &lhs, const HybridState2DNode &rhs) {
    // same state in a maze search is simply when (x,y) are the same
    if ((lhs.x() == rhs.x()) && (lhs.y() == rhs.y()) &&
        (lhs.theta() == rhs.theta())) {
      return true;
    }
    return false;
  }  // Issamestate

  template <int max_heading = 72>
  int Normalizeheading(int heading) {
    if (heading >= max_heading)
      return heading - max_heading;
    else if (heading < 0)
      return heading + max_heading;
    else
      return heading;
  }

};  // end class HybridState2DNode

class HybridState4DNode {
  using HybridAStar_Search =
      HybridAStarSearch<HybridState4DNode, SearchConfig, CollisionChecking,
                        ASV::common::math::ReedsSheppStateSpace>;

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
  float GoalDistanceEstimate(
      const HybridState4DNode &nodeGoal,
      const ASV::common::math::ReedsSheppStateSpace &RSCurve) {
    std::array<double, 3> _rsstart = {this->x_, this->y_, this->theta_};
    std::array<double, 3> _rsend = {nodeGoal.x(), nodeGoal.y(),
                                    nodeGoal.theta()};

    double rsdistance = RSCurve.rs_distance(_rsstart, _rsend);
    double l1distance =
        std::abs(x_ - nodeGoal.x()) + std::abs(y_ - nodeGoal.y());
    return std::max(rsdistance, l1distance);

  }  // GoalDistanceEstimate

  bool IsGoal(const HybridState4DNode &nodeGoal) {
    if ((std::abs(x_ - nodeGoal.x()) <= 0.05) &&
        (std::abs(y_ - nodeGoal.y()) <= 0.05) &&
        (std::abs(ASV::common::math::fNormalizeheadingangle(
             theta_ - nodeGoal.theta())) < 0.02)) {
      return true;
    }
    return false;
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

      HybridState4DNode NewNode =
          HybridState4DNode(new_x, new_y, new_theta, new_type);

      if (parent_node) {
        if (!(collision_checking.InCollision(new_x, new_y, new_theta) ||
              Issamestate(*parent_node, NewNode))) {
          astarsearch->AddSuccessor(NewNode);
        }
      } else {
        if (!collision_checking.InCollision(new_x, new_y, new_theta)) {
          astarsearch->AddSuccessor(NewNode);
        }
      }
    }

    return true;
  }  // GetSuccessors

  // (g value) given this node, what does it cost to move to successor.
  float GetCost(const HybridState4DNode &successor,
                const SearchConfig &_SearchConfig) {
    int current_type = static_cast<int>(this->type_);
    int successor_type = static_cast<int>(successor.type());

    return _SearchConfig.cost_map[current_type][successor_type];

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
    std::cout << "Node position : " << x_ << ", " << y_ << ", " << theta_
              << ", " << type_ << std::endl;
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

  bool Issamestate(const HybridState4DNode &lhs, const HybridState4DNode &rhs) {
    // same state in a maze search is simply when (x,y) are the same
    if ((std::abs(lhs.x() - rhs.x()) <= 0.05) &&
        (std::abs(lhs.y() - rhs.y()) <= 0.05) &&
        (std::abs(ASV::common::math::fNormalizeheadingangle(
             lhs.theta() - rhs.theta())) < 0.01)) {
      return true;
    }
    return false;
  }  // Issamestate

};  // end class HybridState4DNode

class HybridAStar {
  using HybridAStar_4dNode_Search =
      HybridAStarSearch<HybridState4DNode, SearchConfig, CollisionChecking,
                        ASV::common::math::ReedsSheppStateSpace>;
  using HybridAStar_2dNode_Search =
      HybridAStarSearch<HybridState2DNode, SearchConfig,
                        std::array<std::array<std::array<bool, 161>, 161>, 72>>;

 public:
  HybridAStar(const CollisionData &collisiondata,
              const HybridAStarConfig &hybridastarconfig)
      : startpoint_({0, 0, 0}),
        endpoint_({0, 0, 0}),
        collision_checker_(collisiondata),
        rscurve_(1.0 / collisiondata.MAX_CURVATURE),
        searchconfig_({
            0,     // move_length
            0,     // turning_angle
            {{0}}  // cost_map
        }),
        astar_4d_search_(5000),
        astar_2d_search_(5000),
        CollisionLookup_({false}) {
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
  HybridAStar &setup_start_end(const float start_x, const float start_y,
                               const float start_theta, const float end_x,
                               const float end_y, const float end_theta) {
    startpoint_ = {start_x, start_y, start_theta};
    endpoint_ = {end_x, end_y, end_theta};

    HybridState4DNode nodeStart(start_x, start_y, start_theta);
    HybridState4DNode nodeEnd(end_x, end_y, end_theta);
    astar_4d_search_.SetStartAndGoalStates(nodeStart, nodeEnd, rscurve_);

    return *this;
  }  // setup_start_end

  // update the start and ending points
  HybridAStar &setup_2d_start_end(const float start_x, const float start_y,
                                  const float start_theta, const float end_x,
                                  const float end_y, const float end_theta) {
    HybridState2DNode nodeStart(start_x, start_y, start_theta);
    HybridState2DNode nodeEnd(end_x, end_y, end_theta);
    astar_2d_search_.SetStartAndGoalStates(nodeStart, nodeEnd);
    GenerateCollisionLookup(endpoint_, searchconfig_.move_length);

    return *this;
  }  // setup_2d_start_end

  void perform_4dnode_search() {
    unsigned int SearchState;
    unsigned int SearchSteps = 0;

    std::cout << "4d\n";

    do {
      // perform a hybrid A* search
      SearchState = astar_4d_search_.SearchStep(searchconfig_,
                                                collision_checker_, rscurve_);
      SearchSteps++;

      // get the current node
      std::vector<std::array<double, 3>> closedlist_trajecotry;
      HybridState4DNode *current_p = astar_4d_search_.GetCurrentNode();
      while (current_p) {
        closedlist_trajecotry.push_back(
            {current_p->x(), current_p->y(), current_p->theta()});
        current_p = astar_4d_search_.GetCurrentNodePrev();
      }
      std::reverse(closedlist_trajecotry.begin(), closedlist_trajecotry.end());
      // for (const auto &value : closedlist_trajecotry)
      //   std::cout << value.at(0) << " " << value.at(2) << std::endl;

      // try a collision-free RS curve
      if (!closedlist_trajecotry.empty()) {
        hybridastar_trajecotry_ = closedlist_trajecotry;
        auto rscurve_generated = TryRSCurve(closedlist_trajecotry.back());
        if (!collision_checker_.InCollision(rscurve_generated)) {
          hybridastar_trajecotry_.insert(hybridastar_trajecotry_.end(),
                                         rscurve_generated.begin(),
                                         rscurve_generated.end());
          astar_4d_search_.CancelSearch();
          std::cout << "find a collision free RS curve!\n";
        }
      }

    } while (SearchState == HybridAStar_4dNode_Search::SEARCH_STATE_SEARCHING);

    if (SearchState == HybridAStar_4dNode_Search::SEARCH_STATE_SUCCEEDED) {
      HybridState4DNode *node = astar_4d_search_.GetSolutionStart();
      hybridastar_trajecotry_.clear();
      while (node) {
        hybridastar_trajecotry_.push_back(
            {node->x(), node->y(), node->theta()});
        node = astar_4d_search_.GetSolutionNext();
      }
      // Once you're done with the solution you can free the nodes up
      astar_4d_search_.FreeSolutionNodes();
    }

    // Display the number of loops the search went through
    std::cout << "SearchSteps : " << SearchSteps << "\n";
    // astarsearch_.FreeSolutionNodes();
    astar_4d_search_.EnsureMemoryFreed();

  }  // perform_4dnode_search

  void perform_2dnode_search() {
    unsigned int SearchState;
    unsigned int SearchSteps = 0;

    do {
      // perform a hybrid A* search
      SearchState =
          astar_2d_search_.SearchStep(searchconfig_, CollisionLookup_);
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
      std::cout << "find 2d solution\n";
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
    std::cout << "SearchSteps : " << SearchSteps << "\n";
    // astarsearch_.FreeSolutionNodes();
    astar_2d_search_.EnsureMemoryFreed();

  }  // perform_2dnode_search

  auto hybridastar_trajecotry() const noexcept {
    return hybridastar_trajecotry_;
  }  // hybridastar_trajecotry

  auto hybridastar_2dtrajecotry() const noexcept {
    return hybridastar_2d_trajecotry_;
  }  // hybridastar_2dtrajecotry

  std::array<float, 3> startpoint() const noexcept { return startpoint_; }
  std::array<float, 3> endpoint() const noexcept { return endpoint_; }

 private:
  std::array<float, 3> startpoint_;
  std::array<float, 3> endpoint_;

  CollisionChecking collision_checker_;
  ASV::common::math::ReedsSheppStateSpace rscurve_;
  SearchConfig searchconfig_;
  HybridAStar_4dNode_Search astar_4d_search_;
  HybridAStar_2dNode_Search astar_2d_search_;
  std::array<std::array<std::array<bool, 161>, 161>, 72> CollisionLookup_;

  // search results
  std::vector<std::array<double, 3>> hybridastar_trajecotry_;
  std::vector<std::array<double, 3>> hybridastar_2d_trajecotry_;

  // generate the config for search
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

  // try a RS curve from current node to goal
  std::vector<std::array<double, 3>> TryRSCurve(
      const std::array<double, 3> &rs_start) {
    std::array<double, 3> rs_end = {static_cast<double>(endpoint_[0]),
                                    static_cast<double>(endpoint_[1]),
                                    static_cast<double>(endpoint_[2])};

    return rscurve_.rs_state(rs_start, rs_end, 0.4);

  }  // TryRSCurve

  // tables * length * length
  void GenerateCollisionLookup(const std::array<float, 3> &endpoint,
                               const float grid_resolution) {
    const int half_length = 80;
    const float heading_resolution = M_PI / 36.0;

    for (int i = 0; i != 72; ++i) {
      for (int j = 0; j != (161); j++)
        for (int k = 0; k != (161); k++) {
          float grid_x = endpoint.at(0) + grid_resolution * (j - half_length);
          float grid_y = endpoint.at(1) + grid_resolution * (k - half_length);
          float grid_theta = ASV::common::math::fNormalizeheadingangle(
              endpoint.at(2) + heading_resolution * i);
          CollisionLookup_[i][j][k] = true;

          // if (collision_checker_.InCollision(grid_x, grid_y, grid_theta)) {
          //   CollisionLookup_[i][j][k] = true;
          // } else
          //   CollisionLookup_[i][j][k] = false;
        }
    }
  }  // GenerateCollisionLookup

};  // end class HybridAStar

}  // namespace ASV::planning

#endif /* _HYBRIDASTAR_H_ */