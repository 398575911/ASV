/*
*******************************************************************************
* OpenSpacePlanner.h:
* Path planner used in the low-speed vessel, including hybrid A star,
* trajectory smoother and collision checking. This planner is designed to
* be used in both fully-actuated and underactuated vessels.
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
*******************************************************************************
*/

#ifndef _OPENSPACEPLANNER_H_
#define _OPENSPACEPLANNER_H_

namespace ASV::planning {

class OpenSpacePlanner {
 public:
  OpenSpacePlanner() {}
  virtual ~OpenSpacePlanner() = default;

 private:
};  // end class OpenSpacePlanner

}  // namespace ASV::planning

#endif /* _OPENSPACEPLANNER_H_ */