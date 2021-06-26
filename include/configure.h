#include <algorithm>
#include <iostream>
#include <vector>
// TODO: rewrite the class
class Config {
 public:
  // motion resolution parameters
  // double motion_resulotion; // path interpolate resolution
  // double n_steer; // number of steer command
  // double extend_area; // [m] map extend length
  // obstacle list
  // std::vector<double> ox; // x coordinates in real world
  // std::vector<double> oy; // y coordinates in real world
  // double xyreso; // [m]
  // double yawreso; // [rad]
  // grid bound

  // TODO: rename the following variables
  int minx;  // rename to minCellX
  int miny;
  int minyaw;
  int minyawt;
  int maxx;
  int maxy;
  int maxyaw;
  int maxyawt;
  int xw;
  int yw;
  int yaww;
  int yawtw;

  // the following should be double
  double xyreso;
  double yawreso;

  Config(int minx, int miny, int minyaw, int minyawt, int maxx, int maxy,
         int maxyaw, int maxyawt, int xw, int yw, int yaww, int yawtw,
         double xyreso, double yawreso) {
    /* */
    this->minx = minx;
    this->miny = miny;
    this->minyaw = minyaw;
    this->minyawt = minyawt;
    this->maxx = maxx;
    this->maxy = maxy;
    this->maxyaw = maxyaw;
    this->maxyawt = maxyawt;
    this->xw = xw;
    this->yw = yw;
    this->yaww = yaww;
    this->yawtw = yawtw;
    this->xyreso = xyreso;
    this->yawreso = yawreso;
  }
  // double min_x_m;
  // double min_y_m;
  // double max_x_m;Config(double xyreso, double yawreso, std::vector<double>
  // ox, std::vector<double> oy) {
  //     minx = std::round(min_x_m / xyreso);
  //     miny = std::round(min_y_m / xyreso);
  //     maxx = std::round(max_x_m / xyreso);
  //     maxy = std::round(max_y_m / xyreso);

  //     xw = std::round(maxx - minx);
  //     yw = std::round(maxy - miny);

  //     minyaw = std::round(-M_PI / yawreso) - 1;
  //     maxyaw = std::round(M_PI / yawreso);

  //     yaww = std::round(maxyaw - minyaw);

  // }

  // cost related define
  // double sb_cost; // switch back penalty cost
  // double back_cost; // backward penalty cost
  // double steer_change_cost; // steer angle change penalty cost
  // double h_cost; // heuristic cost

  // obstacle map: two dimensional array
  // std::vector<std::vector<int> > obmap;
  // the dimension of x and the dimension of y should be equal

  // constructor
  // Config(double xyreso, double yawreso, std::vector<double> ox,
  // std::vector<double> oy) {
  //     minx = std::round(min_x_m / xyreso);
  //     miny = std::round(min_y_m / xyreso);
  //     maxx = std::round(max_x_m / xyreso);
  //     maxy = std::round(max_y_m / xyreso);

  //     xw = std::round(maxx - minx);
  //     yw = std::round(maxy - miny);

  //     minyaw = std::round(-M_PI / yawreso) - 1;
  //     maxyaw = std::round(M_PI / yawreso);

  //     yaww = std::round(maxyaw - minyaw);

  // }
};
