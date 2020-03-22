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

int main(int argc, char **argv) {
  int i, vcount = 10;
  // void *kd, *set;
  struct kdtree *kd;
  struct kdres *set;

  double pt[3] = {0, 0, 0};

  if (argc > 1 && isdigit(argv[1][0])) {
    vcount = atoi(argv[1]);
  }
  printf("inserting %d random vectors... \n", vcount);
  fflush(stdout);

  kd = kd_create(3);

  for (i = 0; i < vcount; i++) {
    float x, y, z;
    x = ((float)rand() / RAND_MAX) * 200.0 - 100.0;
    y = ((float)rand() / RAND_MAX) * 200.0 - 100.0;
    z = 0;

    printf(" %.3f, %.3f, %.3f\n", x, y, z);

    assert(kd_insert3(kd, x, y, z, 0) == 0);
  }

  set = kd_nearest_range3(kd, pt[0], pt[1], pt[2], 40);

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

  // Gnuplot gp;
  // gp << "reset\n";
  // gp << "set terminal x11 size 800, 800 0\n";
  // gp << "set size ratio -1\n";

  kd_res_free(set);
  kd_free(kd);
  return 0;
}
