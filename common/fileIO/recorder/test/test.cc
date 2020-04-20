/*
***********************************************************************
* testdatabase.cc:
* uint test for database
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include "../include/dataparser.h"

const std::string folderp = "../../data/";
const std::string config_path = "../../config/dbconfig.json";
const double starting_time = 0;
const double end_time = 10;

int main() {
  // record
  ASV::common::plan_route_db_data plan_route_db_data{
      -1,            // local_time
      1,             // setpoints_X
      2,             // setpoints_Y
      3,             // setpoints_heading
      4,             // setpoints_longitude
      5,             // setpoints_latitude
      1,             // speed
      2,             // captureradius
      "3434",        // utm_zone
      {2, 3, 5, 1},  // WPX
      {4, 4, 4, 4},  // WPY
      {5},           // WPLONG
      {6}            // WPLAT
  };

  ASV::common::plan_lattice_db_data plan_lattice_db_data{
      -1,  // local_time
      1,   // lattice_x
      2,   // lattice_y
      3,   // lattice_theta
      4,   // lattice_kappa
      5,   // lattice_speed
      6    // lattice_dspeed
  };

  // parse
  ASV::common::planner_parser planner_parser(folderp, config_path);
  auto read_route = planner_parser.parse_route_table(starting_time, end_time);
  auto read_lattice =
      planner_parser.parse_lattice_table(starting_time, end_time);

  for (const auto &value : read_route) {
    std::cout << "local_time " << value.local_time << std::endl;
    std::cout << "setpoints_X " << value.setpoints_X << std::endl;
    std::cout << "setpoints_Y " << value.setpoints_Y << std::endl;
    std::cout << "setpoints_heading " << value.setpoints_heading << std::endl;
    std::cout << "setpoints_longitude " << value.setpoints_longitude
              << std::endl;
    std::cout << "setpoints_latitude " << value.setpoints_latitude << std::endl;
    std::cout << "speed " << value.speed << std::endl;
    std::cout << "captureradius " << value.captureradius << std::endl;
    std::cout << "captureradius " << value.captureradius << std::endl;
    std::cout << "utm_zone " << value.utm_zone << std::endl;
    std::cout << "WPX ";
    for (const auto &v : value.WPX) std::cout << v << " ";
    std::cout << std::endl;
    std::cout << "WPY ";
    for (const auto &v : value.WPY) std::cout << v << " ";
    std::cout << std::endl;
    std::cout << "WPLONG ";
    for (const auto &v : value.WPLONG) std::cout << v << " ";
    std::cout << std::endl;
    std::cout << "WPLAT ";
    for (const auto &v : value.WPLAT) std::cout << v << " ";
    std::cout << std::endl;
  }
}
