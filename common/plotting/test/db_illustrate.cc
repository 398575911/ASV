/*
***********************************************************************
* db_illustrate.cc:
* illustrate the a collection of ASV results, stored in the database
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include "common/fileIO/recorder/include/dataparser.h"
#include "common/plotting/include/gnuplot-iostream.h"

// setup the time interval
const double starting_time = 0;
const double end_time = 60;

const std::string folderp = "../../data/";
const std::string config_path =
    "/home/scar1et/Coding/ASV/common/fileIO/recorder/config/dbconfig.json";
int figure_id = 0;

// plot the results stored in planner database
void plot_planner() {
  // parse
  ASV::common::planner_parser planner_parser(folderp, config_path);
  auto read_route = planner_parser.parse_route_table(starting_time, end_time);
  auto read_lattice =
      planner_parser.parse_lattice_table(starting_time, end_time);

  std::cout << "results of route planning\n";
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
}  // plot_planner

// plot the results stored in perception database
void plot_perception() {}  // plot_perception

// plot the results stored in GPS database
void plot_GPS() {
  // parse
  ASV::common::GPS_parser GPS_parser(folderp, config_path);
  auto read_gps = GPS_parser.parse_gps_table(starting_time, end_time);
  auto read_imu = GPS_parser.parse_imu_table(starting_time, end_time);

  /******************** plotting of GPS position ********************/
  std::vector<std::pair<double, double>> xy_pts_A;
  std::vector<std::pair<double, double>> xy_pts_B;
  std::vector<std::pair<double, double>> xy_pts_C;
  std::vector<std::pair<double, double>> xy_pts_D;
  for (std::size_t index = 0; index != read_gps.size(); index++) {
    auto gps_i = read_gps[index];
    xy_pts_A.push_back(std::make_pair(gps_i.local_time, gps_i.latitude));
    xy_pts_B.push_back(std::make_pair(gps_i.local_time, gps_i.longitude));
    xy_pts_C.push_back(std::make_pair(gps_i.local_time, gps_i.UTM_x));
    xy_pts_D.push_back(std::make_pair(gps_i.local_time, gps_i.UTM_y));
  }

  ++figure_id;
  Gnuplot gp;
  gp << "set terminal x11 size 1000, 1000 " + std::to_string(figure_id) + "\n";
  gp << "set multiplot layout 4, 1 title 'time series of GPS position' font "
        "',14'\n";
  // latitude
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'latitude (deg)'\n";
  gp << "plot " << gp.file1d(xy_pts_A)
     << " with lines lt 1 lw 2 lc rgb '#4393C3' notitle\n";

  // longitude
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'longitude (deg)'\n";
  gp << "plot " << gp.file1d(xy_pts_B)
     << " with lines lt 1 lw 2 lc rgb '#4393C3' notitle\n";

  // UTM_x
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'UTM_x (m)'\n";
  gp << "plot " << gp.file1d(xy_pts_C)
     << " with lines lt 1 lw 2 lc rgb '#4393C3' notitle\n";

  // UTM_y
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'UTM_y (m)'\n";
  gp << "set xlabel 'time (s)'\n";
  gp << "plot " << gp.file1d(xy_pts_D)
     << " with lines lt 1 lw 2 lc rgb '#4393C3' notitle\n";

  gp << "unset multiplot\n";

  /********************* plotting of rotation ************************/
  std::vector<std::pair<double, double>> xy_pts_A_gps;
  std::vector<std::pair<double, double>> xy_pts_A_imu;
  std::vector<std::pair<double, double>> xy_pts_B_gps;
  std::vector<std::pair<double, double>> xy_pts_B_imu;
  std::vector<std::pair<double, double>> xy_pts_C_gps;
  std::vector<std::pair<double, double>> xy_pts_C_imu;
  for (std::size_t index = 0; index != read_gps.size(); index++) {
    auto gps_i = read_gps[index];
    xy_pts_A_gps.push_back(std::make_pair(gps_i.local_time, gps_i.roll));
    xy_pts_B_gps.push_back(std::make_pair(gps_i.local_time, gps_i.pitch));
    xy_pts_C_gps.push_back(std::make_pair(gps_i.local_time, gps_i.heading));
  }
  for (std::size_t index = 0; index != read_imu.size(); index++) {
    auto imu_i = read_imu[index];
    xy_pts_A_imu.push_back(std::make_pair(imu_i.local_time, imu_i.roll));
    xy_pts_B_imu.push_back(std::make_pair(imu_i.local_time, imu_i.pitch));
    xy_pts_C_imu.push_back(std::make_pair(imu_i.local_time, imu_i.yaw));
  }

  ++figure_id;
  gp << "set terminal x11 size 1000, 900 " + std::to_string(figure_id) + "\n";
  gp << "set multiplot layout 3, 1 title 'time series of rotation' font "
        "',14'\n";
  // roll
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'roll (deg)'\n";
  gp << "plot " << gp.file1d(xy_pts_A_gps)
     << " with linespoints linetype 1 lw 2 lc rgb '#4393C3' pointtype 7 "
        "pointsize 1 title 'gps', "
     << gp.file1d(xy_pts_A_imu)
     << " with lines lt 1 lw 2 lc rgb 'black' title 'imu'\n";

  // pitch
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'pitch (deg)'\n";
  gp << "plot " << gp.file1d(xy_pts_B_gps)
     << " with linespoints linetype 1 lw 2 lc rgb '#4393C3' pointtype 7 "
        "pointsize 1 title 'gps', "
     << gp.file1d(xy_pts_B_imu)
     << " with lines lt 1 lw 2 lc rgb 'black' title 'imu'\n";

  // heading
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'heading (deg)'\n";
  gp << "set xlabel 'time (s)'\n";
  gp << "plot " << gp.file1d(xy_pts_C_gps)
     << " with linespoints linetype 1 lw 2 lc rgb '#4393C3' pointtype 7 "
        "pointsize 1 title 'gps', "
     << gp.file1d(xy_pts_C_imu)
     << " with lines lt 1 lw 2 lc rgb 'black' title 'imu'\n";

  gp << "unset multiplot\n";

  /********************* plotting of acceleration ************************/

}  // plot_GPS

// plot the results stored in wind database
void plot_wind() {
  // parse
  ASV::common::wind_parser wind_parser(folderp, config_path);
  auto read_wind = wind_parser.parse_table(starting_time, end_time);

  // plotting
  std::vector<std::pair<double, double>> xy_pts_A;
  std::vector<std::pair<double, double>> xy_pts_B;
  for (std::size_t index = 0; index != read_wind.size(); index++) {
    auto wind_i = read_wind[index];
    xy_pts_A.push_back(std::make_pair(wind_i.local_time, wind_i.speed));
    xy_pts_B.push_back(std::make_pair(wind_i.local_time, wind_i.orientation));
  }

  ++figure_id;
  Gnuplot gp;
  gp << "set terminal x11 size 1000, 800 " + std::to_string(figure_id) + "\n";
  gp << "set multiplot layout 2, 1 title 'time series of wind' font ',14'\n";
  // wind speed
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set xlabel 'time (s)'\n";
  gp << "set ylabel 'speed (m/s)'\n";

  gp << "plot " << gp.file1d(xy_pts_A)
     << " with lines lt 1 lw 2 lc rgb '#4393C3' notitle\n";

  // wind orientation
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set xlabel 'time (s)'\n";
  gp << "set ylabel 'orientation (rad)'\n";
  gp << "plot " << gp.file1d(xy_pts_B)
     << " with lines lt 1 lw 2 lc rgb '#4393C3' notitle\n";

  gp << "unset multiplot\n";
}  // plot_wind

// plot the results stored in estimator database
void plot_estimator() {
  // parse
  ASV::common::estimator_parser estimator_parser(folderp, config_path);
  auto read_measurement =
      estimator_parser.parse_measurement_table(starting_time, end_time);
  auto read_error = estimator_parser.parse_error_table(starting_time, end_time);
  auto read_state = estimator_parser.parse_state_table(starting_time, end_time);

  /******************** comparision of estimator position ********************/
  std::vector<std::pair<double, double>> xy_pts_A_meas;
  std::vector<std::pair<double, double>> xy_pts_A_state;
  std::vector<std::pair<double, double>> xy_pts_B_meas;
  std::vector<std::pair<double, double>> xy_pts_B_state;
  std::vector<std::pair<double, double>> xy_pts_C_meas;
  std::vector<std::pair<double, double>> xy_pts_C_state;
  for (std::size_t index = 0; index != read_measurement.size(); index++) {
    auto meas_i = read_measurement[index];
    xy_pts_A_meas.push_back(std::make_pair(meas_i.local_time, meas_i.meas_x));
    xy_pts_B_meas.push_back(std::make_pair(meas_i.local_time, meas_i.meas_y));
    xy_pts_C_meas.push_back(
        std::make_pair(meas_i.local_time, meas_i.meas_theta * 180.0 / M_PI));
  }
  for (std::size_t index = 0; index != read_state.size(); index++) {
    auto state_i = read_state[index];
    xy_pts_A_state.push_back(
        std::make_pair(state_i.local_time, state_i.state_x));
    xy_pts_B_state.push_back(
        std::make_pair(state_i.local_time, state_i.state_y));
    xy_pts_C_state.push_back(
        std::make_pair(state_i.local_time, state_i.state_theta * 180.0 / M_PI));
  }

  ++figure_id;
  Gnuplot gp;
  gp << "set terminal x11 size 1000, 900 " + std::to_string(figure_id) + "\n";
  gp << "set multiplot layout 3, 1 title 'comparision of estimator position' "
        "font ',14'\n";
  // X
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'X (m)'\n";
  gp << "plot " << gp.file1d(xy_pts_A_meas)
     << " with linespoints linetype 1 lw 2 lc rgb '#4393C3' pointtype 7 "
        "pointsize 1 title 'meas', "
     << gp.file1d(xy_pts_A_state)
     << " with lines lt 1 lw 2 lc rgb 'black' title 'state'\n";

  // Y
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'Y (m)'\n";
  gp << "plot " << gp.file1d(xy_pts_B_meas)
     << " with linespoints linetype 1 lw 2 lc rgb '#4393C3' pointtype 7 "
        "pointsize 1 title 'meas', "
     << gp.file1d(xy_pts_B_state)
     << " with lines lt 1 lw 2 lc rgb 'black' title 'state'\n";

  // theta
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'heading (deg)'\n";
  gp << "set xlabel 'time (s)'\n";
  gp << "plot " << gp.file1d(xy_pts_C_meas)
     << " with linespoints linetype 1 lw 2 lc rgb '#4393C3' pointtype 7 "
        "pointsize 1 title 'meas', "
     << gp.file1d(xy_pts_C_state)
     << " with lines lt 1 lw 2 lc rgb 'black' title 'state'\n";

  gp << "unset multiplot\n";

  /******************** comparision of estimator velocity ********************/
  xy_pts_A_meas.clear();
  xy_pts_A_state.clear();
  xy_pts_B_meas.clear();
  xy_pts_B_state.clear();
  xy_pts_C_meas.clear();
  xy_pts_C_state.clear();
  for (std::size_t index = 0; index != read_measurement.size(); index++) {
    auto meas_i = read_measurement[index];
    xy_pts_A_meas.push_back(std::make_pair(meas_i.local_time, meas_i.meas_u));
    xy_pts_B_meas.push_back(std::make_pair(meas_i.local_time, meas_i.meas_v));
    xy_pts_C_meas.push_back(std::make_pair(meas_i.local_time, meas_i.meas_r));
  }
  for (std::size_t index = 0; index != read_state.size(); index++) {
    auto state_i = read_state[index];
    xy_pts_A_state.push_back(
        std::make_pair(state_i.local_time, state_i.state_u));
    xy_pts_B_state.push_back(
        std::make_pair(state_i.local_time, state_i.state_v));
    xy_pts_C_state.push_back(
        std::make_pair(state_i.local_time, state_i.state_r));
  }

  ++figure_id;
  gp << "set terminal x11 size 1000, 900 " + std::to_string(figure_id) + "\n";
  gp << "set multiplot layout 3, 1 title 'comparision of estimator velocity' "
        "font ',14'\n";
  // X
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'u (m/s)'\n";
  gp << "plot " << gp.file1d(xy_pts_A_meas)
     << " with linespoints linetype 1 lw 2 lc rgb '#4393C3' pointtype 7 "
        "pointsize 1 title 'meas', "
     << gp.file1d(xy_pts_A_state)
     << " with lines lt 1 lw 2 lc rgb 'black' title 'state'\n";

  // Y
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'v (m/s)'\n";
  gp << "plot " << gp.file1d(xy_pts_B_meas)
     << " with linespoints linetype 1 lw 2 lc rgb '#4393C3' pointtype 7 "
        "pointsize 1 title 'meas', "
     << gp.file1d(xy_pts_B_state)
     << " with lines lt 1 lw 2 lc rgb 'black' title 'state'\n";

  // theta
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'r (rad/s)'\n";
  gp << "set xlabel 'time (s)'\n";
  gp << "plot " << gp.file1d(xy_pts_C_meas)
     << " with linespoints linetype 1 lw 2 lc rgb '#4393C3' pointtype 7 "
        "pointsize 1 title 'meas', "
     << gp.file1d(xy_pts_C_state)
     << " with lines lt 1 lw 2 lc rgb 'black' title 'state'\n";

  gp << "unset multiplot\n";

  /******************** time series of estimator error ********************/
  std::vector<std::pair<double, double>> xy_pts_A;
  std::vector<std::pair<double, double>> xy_pts_B;
  std::vector<std::pair<double, double>> xy_pts_C;
  std::vector<std::pair<double, double>> xy_pts_D;
  std::vector<std::pair<double, double>> xy_pts_E;
  std::vector<std::pair<double, double>> xy_pts_F;
  for (std::size_t index = 0; index != read_error.size(); index++) {
    auto error_i = read_error[index];
    xy_pts_A.push_back(std::make_pair(error_i.local_time, error_i.perror_x));
    xy_pts_B.push_back(std::make_pair(error_i.local_time, error_i.perror_y));
    xy_pts_C.push_back(std::make_pair(error_i.local_time, error_i.perror_mz));
    xy_pts_D.push_back(std::make_pair(error_i.local_time, error_i.verror_x));
    xy_pts_E.push_back(std::make_pair(error_i.local_time, error_i.verror_y));
    xy_pts_F.push_back(std::make_pair(error_i.local_time, error_i.verror_mz));
  }

  ++figure_id;
  gp << "set terminal x11 size 1000, 900 " + std::to_string(figure_id) + "\n";
  gp << "set multiplot layout 6, 1 title 'time series of estimator error' "
        "font ',14'\n";
  // X
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'error in X (m)'\n";
  gp << "plot " << gp.file1d(xy_pts_A)
     << " with lines lt 1 lw 2 lc rgb 'black' notitle\n";

  // Y
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'error in Y (m)'\n";
  gp << "plot " << gp.file1d(xy_pts_B)
     << " with lines lt 1 lw 2 lc rgb 'black' notitle\n";

  // theta
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'error in theta (rad)'\n";
  gp << "plot " << gp.file1d(xy_pts_C)
     << " with lines lt 1 lw 2 lc rgb 'black' notitle\n";

  // error in u
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'error in u (m/s)'\n";
  gp << "plot " << gp.file1d(xy_pts_D)
     << " with lines lt 1 lw 2 lc rgb 'black' notitle\n";

  // theta
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'error in v (m/s)'\n";
  gp << "plot " << gp.file1d(xy_pts_E)
     << " with lines lt 1 lw 2 lc rgb 'black' notitle\n";

  // theta
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set xlabel 'time (s)'\n";
  gp << "set ylabel 'error in r (rad/s)'\n";
  gp << "plot " << gp.file1d(xy_pts_F)
     << " with lines lt 1 lw 2 lc rgb 'black' notitle\n";

  gp << "unset multiplot\n";
}  // plot_estimator

// plot the results stored in stm32 database
void plot_stm32() {}  // plot_stm32

// plot the results stored in controller database
void plot_controller() {
  // parse
  ASV::common::control_parser control_parser(folderp, config_path);
  auto read_setpoint =
      control_parser.parse_setpoint_table(starting_time, end_time);
  auto read_TA = control_parser.parse_TA_table(starting_time, end_time);

  /******************** time series of tracker setpoints ********************/
  std::vector<std::pair<double, double>> xy_pts_A;
  std::vector<std::pair<double, double>> xy_pts_B;
  std::vector<std::pair<double, double>> xy_pts_C;
  std::vector<std::pair<double, double>> xy_pts_D;
  std::vector<std::pair<double, double>> xy_pts_E;
  std::vector<std::pair<double, double>> xy_pts_F;
  for (std::size_t index = 0; index != read_setpoint.size(); index++) {
    auto set_i = read_setpoint[index];
    xy_pts_A.push_back(std::make_pair(set_i.local_time, set_i.set_x));
    xy_pts_B.push_back(std::make_pair(set_i.local_time, set_i.set_y));
    xy_pts_C.push_back(std::make_pair(set_i.local_time, set_i.set_theta));
    xy_pts_D.push_back(std::make_pair(set_i.local_time, set_i.set_u));
    xy_pts_E.push_back(std::make_pair(set_i.local_time, set_i.set_v));
    xy_pts_F.push_back(std::make_pair(set_i.local_time, set_i.set_r));
  }

  ++figure_id;
  Gnuplot gp;
  gp << "set terminal x11 size 1000, 900 " + std::to_string(figure_id) + "\n";
  gp << "set multiplot layout 6, 1 title 'time series of tracker setpoints' "
        "font ',14'\n";
  // X
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'desired X (m)'\n";
  gp << "plot " << gp.file1d(xy_pts_A)
     << " with lines lt 1 lw 2 lc rgb 'black' notitle\n";

  // Y
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'desired Y (m)'\n";
  gp << "plot " << gp.file1d(xy_pts_B)
     << " with lines lt 1 lw 2 lc rgb 'black' notitle\n";

  // theta
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'desired theta (rad)'\n";
  gp << "plot " << gp.file1d(xy_pts_C)
     << " with lines lt 1 lw 2 lc rgb 'black' notitle\n";

  // error in u
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'desired u (m/s)'\n";
  gp << "plot " << gp.file1d(xy_pts_D)
     << " with lines lt 1 lw 2 lc rgb 'black' notitle\n";

  // theta
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'desired v (m/s)'\n";
  gp << "plot " << gp.file1d(xy_pts_E)
     << " with lines lt 1 lw 2 lc rgb 'black' notitle\n";

  // theta
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set xlabel 'time (s)'\n";
  gp << "set ylabel 'desired r (rad/s)'\n";
  gp << "plot " << gp.file1d(xy_pts_F)
     << " with lines lt 1 lw 2 lc rgb 'black' notitle\n";

  gp << "unset multiplot\n";

  /******************** comparision of thrust ********************/
  std::vector<std::pair<double, double>> xy_pts_de_fx;
  std::vector<std::pair<double, double>> xy_pts_de_fy;
  std::vector<std::pair<double, double>> xy_pts_de_mz;
  std::vector<std::pair<double, double>> xy_pts_es_fx;
  std::vector<std::pair<double, double>> xy_pts_es_fy;
  std::vector<std::pair<double, double>> xy_pts_es_mz;
  for (std::size_t index = 0; index != read_TA.size(); index++) {
    auto TA_i = read_TA[index];
    xy_pts_de_fx.push_back(std::make_pair(TA_i.local_time, TA_i.desired_Fx));
    xy_pts_de_fy.push_back(std::make_pair(TA_i.local_time, TA_i.desired_Fy));
    xy_pts_de_mz.push_back(std::make_pair(TA_i.local_time, TA_i.desired_Mz));
    xy_pts_es_fx.push_back(std::make_pair(TA_i.local_time, TA_i.est_Fx));
    xy_pts_es_fy.push_back(std::make_pair(TA_i.local_time, TA_i.est_Fy));
    xy_pts_es_mz.push_back(std::make_pair(TA_i.local_time, TA_i.est_Mz));
  }

  ++figure_id;
  gp << "set terminal x11 size 1000, 900 " + std::to_string(figure_id) + "\n";
  gp << "set multiplot layout 3, 1 title 'comparision of thrust' "
        "font ',14'\n";
  // X
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'Fx (N)'\n";
  gp << "plot " << gp.file1d(xy_pts_de_fx)
     << " with linespoints linetype 1 lw 2 lc rgb '#4393C3' pointtype 7 "
        "pointsize 1 title 'desired', "
     << gp.file1d(xy_pts_es_fx)
     << " with lines lt 1 lw 2 lc rgb 'black' title 'est'\n";

  // Y
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'Fy (N)'\n";
  gp << "plot " << gp.file1d(xy_pts_de_fy)
     << " with linespoints linetype 1 lw 2 lc rgb '#4393C3' pointtype 7 "
        "pointsize 1 title 'desired', "
     << gp.file1d(xy_pts_es_fy)
     << " with lines lt 1 lw 2 lc rgb 'black' title 'est'\n";

  // theta
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'Mz (N*m)'\n";
  gp << "set xlabel 'time (s)'\n";
  gp << "plot " << gp.file1d(xy_pts_de_mz)
     << " with linespoints linetype 1 lw 2 lc rgb '#4393C3' pointtype 7 "
        "pointsize 1 title 'desired', "
     << gp.file1d(xy_pts_es_mz)
     << " with lines lt 1 lw 2 lc rgb 'black' title 'est'\n";

  gp << "unset multiplot\n";

  /******************** time series of propeller (alpha) ********************/
  int m = read_TA[0].alpha.size();

  ++figure_id;
  gp << "set terminal x11 size 1000, 900 " + std::to_string(figure_id) + "\n";
  gp << "set multiplot layout " + std::to_string(m) +
            ", 1 title 'time series of propellers alpha' font ',14'\n";
  std::vector<std::pair<double, double>> xy_pts_alpha;
  for (int i = 0; i != m; ++i) {
    gp << "set xtics out\n";
    gp << "set ytics out\n";
    gp << "set ylabel 'alpha " + std::to_string(i + 1) + "(deg)'\n";
    xy_pts_alpha.clear();
    for (std::size_t index = 0; index != read_TA.size(); index++) {
      auto TA_i = read_TA[index];
      xy_pts_alpha.push_back(std::make_pair(TA_i.local_time, TA_i.alpha[i]));
    }
    gp << "plot " << gp.file1d(xy_pts_alpha)
       << " with lines lt 1 lw 2 lc rgb 'violet' notitle\n";
  }
  gp << "unset multiplot\n";

  /******************** time series of propeller (rpm) ********************/
  ++figure_id;
  gp << "set terminal x11 size 1000, 900 " + std::to_string(figure_id) + "\n";
  gp << "set multiplot layout " + std::to_string(m) +
            ", 1 title 'time series of propellers rotation' font ',14'\n";
  std::vector<std::pair<double, double>> xy_pts_rotation;
  for (int i = 0; i != m; ++i) {
    gp << "set xtics out\n";
    gp << "set ytics out\n";
    gp << "set ylabel 'rotation" + std::to_string(i + 1) + "(rpm)'\n";
    xy_pts_rotation.clear();
    for (std::size_t index = 0; index != read_TA.size(); index++) {
      auto TA_i = read_TA[index];
      xy_pts_rotation.push_back(std::make_pair(TA_i.local_time, TA_i.rpm[i]));
    }
    gp << "plot " << gp.file1d(xy_pts_rotation)
       << " with lines lt 1 lw 2 lc rgb 'violet' notitle\n";
  }
  gp << "unset multiplot\n";
}  // plot_controller

// plot the 2d planar motion
void plot_2dposition() {
  // parse
  ASV::common::estimator_parser estimator_parser(folderp, config_path);
  auto read_measurement =
      estimator_parser.parse_measurement_table(starting_time, end_time);
  auto read_state = estimator_parser.parse_state_table(starting_time, end_time);

  /******************** comparision of estimator position ********************/
  std::vector<std::pair<double, double>> xy_pts_A;

  for (std::size_t index = 0; index != read_state.size(); index++) {
    auto state_i = read_state[index];
    xy_pts_A.push_back(std::make_pair(state_i.state_y, state_i.state_x));
  }

  ++figure_id;
  Gnuplot gp;
  gp << "set terminal x11 size 800, 800 " + std::to_string(figure_id) + "\n";
  gp << "set title 'comparision of estimator position'\n";
  gp << "set xtics out\n";
  gp << "set ytics out\n";
  gp << "set ylabel 'X (m)'\n";
  gp << "set xlabel 'Y (m)'\n";
  gp << "set size ratio -1\n";

  gp << "plot " << gp.file1d(xy_pts_A)
     << " with lines lt 1 lw 2 lc rgb 'black' notitle\n";

}  // plot_2dposition

int main() {
  plot_wind();
  plot_GPS();
  plot_planner();
  plot_estimator();
  plot_controller();
  plot_planner();
  plot_2dposition();
}
