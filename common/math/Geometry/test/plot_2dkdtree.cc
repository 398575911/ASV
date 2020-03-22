// TODO

/*! gcc -Wall -g -o test test.c libkdtree.a */
#include <assert.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>
#include "../../../plotting/include/gnuplot-iostream.h"
#include "../include/kdtree.h"

static double dist_sq(double *a1, double *a2, int dims) {
  double dist_sq = 0, diff;
  while (--dims >= 0) {
    diff = (a1[dims] - a2[dims]);
    dist_sq += diff * diff;
  }
  return dist_sq;
}

int main() {
  constexpr int vcount = 10;
  double radius = 40;
  double pt[3] = {0, 20, 0};
  float x[vcount];
  float y[vcount];
  float z[vcount];

  struct kdtree *kd = kd_create(3);

  for (int i = 0; i < vcount; i++) {
    x[i] = ((float)rand() / RAND_MAX) * 200.0 - 100.0;
    y[i] = ((float)rand() / RAND_MAX) * 200.0 - 100.0;
    z[i] = 0;

    assert(kd_insert3(kd, x[i], y[i], z[i], 0) == 0);
  }

  struct kdres *set = kd_nearest_range3(kd, pt[0], pt[1], pt[2], radius);

  printf("range query returned %d items\n", kd_res_size(set));

  while (!kd_res_end(set)) {
    double pos[3], dist;
    /* get the data and position of the current result item */
    kd_res_item(set, pos);
    /* compute the distance of the current result from the pt */
    dist = sqrt(dist_sq(pt, pos, 3));

    /* print out the retrieved data */
    printf("node at (%.3f, %.3f, %.3f) is %.3f away\n", pos[0], pos[1], pos[2],
           dist);

    /* go to the next entry */
    kd_res_next(set);
  }

  // plotting
  Gnuplot gp;
  gp << "reset\n";
  gp << "set terminal x11 size 1000, 1000 0\n";
  gp << "set size ratio -1\n";
  gp << "set title 'K-nearest neighour using KDtree'\n";

  std::vector<std::tuple<double, double, double>> x_y_radius;
  std::vector<std::pair<double, double>> xy_pts_A;

  x_y_radius.push_back({pt[0], pt[1], radius});
  for (int i = 0; i < vcount; i++) {
    xy_pts_A.push_back(std::make_pair(x[i], y[i]));
  }
  gp << "plot " << gp.file1d(x_y_radius)
     << " with circles lc rgb 'blue' fs transparent solid 0.15 noborder "
        "notitle, "
     << gp.file1d(xy_pts_A) << " with points lc rgb 'black' notitle \n";

  kd_res_free(set);
  kd_free(kd);
  return 0;
}
