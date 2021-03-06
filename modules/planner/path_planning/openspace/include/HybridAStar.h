/*
*******************************************************************************
* HybridAStar.h:
* Hybrid A star algorithm, to generate the coarse collision-free trajectory
* This header file can be read by C++ compilers
*
* by Hu.ZH, Bin Lin(CrossOcean.ai)
*******************************************************************************
*/

#ifndef _HYBRIDASTAR_H_
#define _HYBRIDASTAR_H_

#include "CollisionChecking.h"
#include "hybridstlastar.h"
#include "openspacedata.h"

#include "common/logging/include/easylogging++.h"
#include "common/math/Geometry/include/Reeds_Shepp.h"

namespace ASV::planning {

class HybridState4DNode {
  using HybridAStar_Search =
      HybridAStarSearch<HybridState4DNode, SearchConfig,
                        CollisionChecking_Astar,
                        ASV::common::math::ReedsSheppStateSpace>;

 public:
  enum MovementType {
    STRAIGHT_FORWARD = 0,
    LEFTTURN_FORWARD,
    RIGHTTURN_FORWARD,
    STRAIGHT_REVERSE,
    RIGHTTURN_REVERSE,
    LEFTTURN_REVERSE
  };

  HybridState4DNode()
      : x_(0.0), y_(0.0), theta_(0.0), type_(STRAIGHT_FORWARD) {}

  HybridState4DNode(float px, float py, float ptheta,
                    MovementType ptype = STRAIGHT_FORWARD)
      : x_(px), y_(py), theta_(ptheta), type_(ptype) {}

  float x() const noexcept { return x_; }
  float y() const noexcept { return y_; }
  float theta() const noexcept { return theta_; }
  MovementType type() const noexcept { return type_; }
  bool IsForward() const noexcept { return (type_ <= 2); }

  // Here's the heuristic function that estimates the distance from a Node
  // to the Goal.
  float GoalDistanceEstimate(
      const HybridState4DNode &nodeGoal,
      const ASV::common::math::ReedsSheppStateSpace &RSCurve) {
    std::array<double, 3> _rsstart = {this->x_, this->y_, this->theta_};
    std::array<double, 3> _rsend = {nodeGoal.x(), nodeGoal.y(),
                                    nodeGoal.theta()};

    float rsdistance =
        static_cast<float>(RSCurve.rs_distance(_rsstart, _rsend));
    float l1distance = (std::fabs(this->x_ - nodeGoal.x()) +
                        std::fabs(this->y_ - nodeGoal.y()));

    return std::fmax(rsdistance, l1distance);

  }  // GoalDistanceEstimate

  bool IsGoal(const HybridState4DNode &nodeGoal,
              const SearchConfig &search_config) {
    return IsSameState(nodeGoal, search_config);
  }  // IsGoal

  // This generates the successors to the given Node.
  bool GetSuccessors(HybridAStar_Search *astarsearch,
                     HybridState4DNode *parent_node,
                     const SearchConfig &search_config,
                     const CollisionChecking_Astar &collision_checking) {
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
              Issamestate(*parent_node, NewNode, search_config))) {
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

  bool IsSameState(const HybridState4DNode &rhs,
                   const SearchConfig &search_config) {
    // same state in a maze search is simply when (x,y) are the same
    return ((std::abs(x_ - rhs.x()) <= search_config.x_resolution) &&
            (std::abs(y_ - rhs.y()) <= search_config.y_resolution) &&
            (std::abs(ASV::common::math::fNormalizeheadingangle(
                 theta_ - rhs.theta())) <= search_config.theta_resolution));

  }  // IsSameState

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

  bool Issamestate(const HybridState4DNode &lhs, const HybridState4DNode &rhs,
                   const SearchConfig &search_config) {
    // same state in a maze search is simply when (x,y) are the same
    if ((std::abs(lhs.x() - rhs.x()) <= search_config.x_resolution) &&
        (std::abs(lhs.y() - rhs.y()) <= search_config.y_resolution) &&
        (std::abs(ASV::common::math::fNormalizeheadingangle(
             lhs.theta() - rhs.theta())) < search_config.theta_resolution)) {
      return true;
    }
    return false;
  }  // Issamestate

};  // end class HybridState4DNode

class HybridAStar {
  using HybridAStar_4dNode_Search =
      HybridAStarSearch<HybridState4DNode, SearchConfig,
                        CollisionChecking_Astar,
                        ASV::common::math::ReedsSheppStateSpace>;

  using vecpath = std::vector<std::tuple<double, double, double, bool>>;

 public:
  HybridAStar(const CollisionData &collisiondata,
              const HybridAStarConfig &hybridastarconfig)
      : startpoint_({0, 0, 0}),
        endpoint_({0, 0, 0}),
        rscurve_(1.0 / collisiondata.MAX_CURVATURE),
        searchconfig_({
            0,     // move_length
            0,     // turning_angle
            0,     // x_resolution
            0,     // y_resolution
            0,     // theta_resolution
            {{0}}  // cost_map
        }),
        astar_4d_search_(5000) {
    searchconfig_ = GenerateSearchConfig(collisiondata, hybridastarconfig);
  }
  virtual ~HybridAStar() = default;

  // update the start and ending points
  bool setup_start_end(const float end_x, const float end_y,
                       const float end_theta, const float start_x,
                       const float start_y, const float start_theta,
                       const bool start_direction = true) {
    if (!IsSameNode(start_x, start_y, start_theta, end_x, end_y, end_theta)) {
      startpoint_ = {start_x, start_y, start_theta};
      endpoint_ = {end_x, end_y, end_theta};

      HybridState4DNode::MovementType start_type =
          HybridState4DNode::MovementType::STRAIGHT_FORWARD;
      if (!start_direction)
        start_type = HybridState4DNode::MovementType::STRAIGHT_REVERSE;
      HybridState4DNode nodeStart(start_x, start_y, start_theta, start_type);
      HybridState4DNode nodeEnd(end_x, end_y, end_theta);
      astar_4d_search_.SetStartAndGoalStates(nodeStart, nodeEnd, rscurve_);
      return false;
    }
    return true;

  }  // setup_start_end

  vecpath perform_4dnode_search(
      const CollisionChecking_Astar &collision_checker) {
    unsigned int SearchState;
    vecpath hybridastar_trajecotry;
    do {
      // perform a hybrid A* search
      SearchState = astar_4d_search_.SearchStep(searchconfig_,
                                                collision_checker, rscurve_);

      // get the current node
      HybridState4DNode *current_p = astar_4d_search_.GetCurrentNode();
      if (current_p) {
        std::array<double, 3> closedlist_end = {
            static_cast<double>(current_p->x()),
            static_cast<double>(current_p->y()),
            static_cast<double>(current_p->theta())};

        std::array<double, 3> rscurve_end = {static_cast<double>(endpoint_[0]),
                                             static_cast<double>(endpoint_[1]),
                                             static_cast<double>(endpoint_[2])};

        // try a rs curve
        auto rscurve_generated = rscurve_.rs_state(closedlist_end, rscurve_end,
                                                   searchconfig_.move_length);

        // check the collision for the generated RS curve
        if (!collision_checker.InCollision(rscurve_generated)) {
          vecpath closedlist_trajecotry = {
              {static_cast<double>(current_p->x()),
               static_cast<double>(current_p->y()),
               static_cast<double>(current_p->theta()),
               current_p->IsForward()}};
          while (current_p) {
            double curr_node_x = static_cast<double>(current_p->x());
            double curr_node_y = static_cast<double>(current_p->y());
            double curr_node_theta = static_cast<double>(current_p->theta());

            current_p = astar_4d_search_.GetCurrentNodePrev();
            if (current_p) {
              double pre_node_x = static_cast<double>(current_p->x());
              double pre_node_y = static_cast<double>(current_p->y());
              double pre_node_theta = static_cast<double>(current_p->theta());
              if (!IsSameNode(pre_node_x, pre_node_y, pre_node_theta,
                              curr_node_x, curr_node_y, curr_node_theta)) {
                bool move_dir =
                    IsForward(pre_node_x, pre_node_y, pre_node_theta,
                              curr_node_x, curr_node_y, curr_node_theta);
                closedlist_trajecotry.push_back(
                    {pre_node_x, pre_node_y, pre_node_theta, move_dir});
              }
            }
          }  // end while
          // reverse the trajectory
          std::reverse(closedlist_trajecotry.begin(),
                       closedlist_trajecotry.end());
          // check the forward/reverse switch
          FindSwitch(closedlist_trajecotry);
          // std::cout << "before closed\n";
          // for (const auto &value : closedlist_trajecotry)
          //   std::cout << std::get<0>(value) << ", " << std::get<1>(value)
          //             << ", " << std::get<2>(value) << ", "
          //             << std::get<3>(value) << std::endl;

          // combine two kinds of trajectory
          auto rscurve_trajectory = rscurve_.rs_trajectory(
              closedlist_end, rscurve_end, 1.0 * searchconfig_.move_length);
          closedlist_trajecotry.insert(closedlist_trajecotry.end(),
                                       rscurve_trajectory.begin(),
                                       rscurve_trajectory.end());

          // remove the same node
          RemoveSameState(closedlist_trajecotry);

          hybridastar_trajecotry = closedlist_trajecotry;
          astar_4d_search_.CancelSearch();

          // search status
          // CLOG(INFO, "Hybrid_Astar") << "find a collision free RS curve!";

        }  // end if collision checking
      }
    } while (SearchState == HybridAStar_4dNode_Search::SEARCH_STATE_SEARCHING);

    if (SearchState == HybridAStar_4dNode_Search::SEARCH_STATE_SUCCEEDED) {
      HybridState4DNode *node = astar_4d_search_.GetSolutionStart();

      if (node) {
        vecpath closedlist_trajecotry;
        while (node) {
          double pre_node_x = static_cast<double>(node->x());
          double pre_node_y = static_cast<double>(node->y());
          double pre_node_theta = static_cast<double>(node->theta());

          node = astar_4d_search_.GetSolutionNext();

          if (node) {
            double cur_node_x = static_cast<double>(node->x());
            double cur_node_y = static_cast<double>(node->y());
            double cur_node_theta = static_cast<double>(node->theta());
            if (!IsSameNode(pre_node_x, pre_node_y, pre_node_theta, cur_node_x,
                            cur_node_y, cur_node_theta)) {
              bool move_dir = IsForward(pre_node_x, pre_node_y, pre_node_theta,
                                        cur_node_x, cur_node_y, cur_node_theta);
              closedlist_trajecotry.push_back(
                  {pre_node_x, pre_node_y, pre_node_theta, move_dir});
            }
          }
        }  // end while loop
        // Do not forget the last one
        node = astar_4d_search_.GetSolutionEnd();
        closedlist_trajecotry.push_back(
            {static_cast<double>(node->x()), static_cast<double>(node->y()),
             static_cast<double>(node->theta()), node->IsForward()});
        // check the forward/reverse switch
        FindSwitch(closedlist_trajecotry);
        // remove the same node
        RemoveSameState(closedlist_trajecotry);
        hybridastar_trajecotry = closedlist_trajecotry;
        // search status
        // CLOG(INFO, "Hybrid_Astar") << "find 4d hybrid A star solution!";
      }

      // Once you're done with the solution you can free the nodes up
      astar_4d_search_.FreeSolutionNodes();
    }  // end if(SearchState)

    astar_4d_search_.EnsureMemoryFreed();
    return hybridastar_trajecotry;

  }  // perform_4dnode_search

  std::array<float, 3> startpoint() const noexcept { return startpoint_; }
  std::array<float, 3> endpoint() const noexcept { return endpoint_; }

 private:
  std::array<float, 3> startpoint_;
  std::array<float, 3> endpoint_;

  ASV::common::math::ReedsSheppStateSpace rscurve_;
  SearchConfig searchconfig_;
  HybridAStar_4dNode_Search astar_4d_search_;

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
    searchconfig.x_resolution = 0.05;
    searchconfig.y_resolution = 0.05;
    searchconfig.theta_resolution = 0.02;

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

  // check the movement direction between two nodes
  bool IsForward(const double pre_x, const double pre_y, const double pre_theta,
                 const double cur_x, const double cur_y,
                 const double cur_theta) const {
    double movement_theta = std::atan2(cur_y - pre_y, cur_x - pre_x);
    double mean_theta = 0.5 * (pre_theta + cur_theta);
    return std::abs(ASV::common::math::Normalizeheadingangle(
               movement_theta - mean_theta)) < 0.5 * M_PI;

  }  // IsForward

  void FindSwitch(vecpath &trajectory) {
    std::size_t num_trajectory = trajectory.size();
    // modify the direction based on the movement and heading
    if (num_trajectory >= 2) {
      vecpath new_trajectory = {trajectory[0]};
      for (std::size_t index = 1; index < num_trajectory; ++index) {
        bool move_dir = std::get<3>(new_trajectory.back());
        if (move_dir != std::get<3>(trajectory[index])) {
          new_trajectory.push_back({std::get<0>(trajectory[index]),
                                    std::get<1>(trajectory[index]),
                                    std::get<2>(trajectory[index]), move_dir});
          new_trajectory.push_back(trajectory[index]);

        } else {
          new_trajectory.push_back(trajectory[index]);
        }
      }
      trajectory = new_trajectory;
    }
  }  // FindSwitch

  // remove the same node in the trajectory
  void RemoveSameState(vecpath &trajectory) {
    std::size_t num_trajectory = trajectory.size();
    if (num_trajectory >= 2) {
      vecpath new_trajectory = {trajectory[0]};
      for (std::size_t index = 1; index < num_trajectory; ++index) {
        auto Na = new_trajectory.back();
        auto Nb = trajectory[index];

        if (!IsSameNode(std::get<0>(Na), std::get<1>(Na), std::get<2>(Na),
                        std::get<0>(Nb), std::get<1>(Nb), std::get<2>(Nb)) ||
            (std::get<3>(Na) != std::get<3>(Nb)))
          new_trajectory.push_back(Nb);
      }
      trajectory = new_trajectory;
    }
  }  // RemoveSameState

  // check if two node is closed
  bool IsSameNode(const double lhs_x, const double lhs_y,
                  const double lhs_theta, const double rhs_x,
                  const double rhs_y, const double rhs_theta) const {
    return ((std::abs(lhs_x - rhs_x) <= searchconfig_.x_resolution) &&
            (std::abs(lhs_y - rhs_y) <= searchconfig_.y_resolution) &&
            (std::abs(ASV::common::math::Normalizeheadingangle(
                 lhs_theta - rhs_theta)) <= searchconfig_.theta_resolution));
  }  // IsSameNode

};  // end class HybridAStar

}  // namespace ASV::planning

#endif /* _HYBRIDASTAR_H_ */