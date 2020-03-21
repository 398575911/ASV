/*
***********************************************************************
* test_RScurve.cc:
* Utility test for the JSON
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include <iostream>
#include "../include/Reeds_Shepp.h"
using namespace std;

int main() {
  double q0[3] = {2, 2, 0};
  double q1[3] = {6, 8, 0};

  ReedsSheppStateSpace r;

  //-------------------------------------------get each curve length  +:forword
  //-back--------------------------------------
  for (int i = 0; i < 5; i++) {
    std::cout << r.reedsShepp(q0, q1).length_[i] << std::endl;
  }

  //----------------------------get curve type-------------------------
  std::vector<int> RStypes;
  RStypes = r.xingshentype(q0, q1);
  for (int i = 0; i < RStypes.size(); i++) {
    std::cout << RStypes[i] << std::endl;
  }

  //-------------------------------------------q0 to q1 discrete
  // point---------------------------------------
  std::vector<std::vector<double> > finalpath;
  finalpath = r.xingshensample(q0, q1, 0.1);

  for (int i = 0; i < finalpath.size(); i++) {
    std::cout << finalpath[i][0] << " " << finalpath[i][1] << " "
              << finalpath[i][2] << std::endl;
  }
}
