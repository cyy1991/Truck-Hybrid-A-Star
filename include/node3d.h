#include <algorithm>
#include <iostream>
#include <limits>
#include <numeric>
#include <vector>
using namespace std;
// nodes, maybe node
class Node3d {
 public:
  int xidx;
  int yidx;
  int yawidx;
  bool direction;  // direciton
  // history
  std::vector<double> xlist;
  std::vector<double> ylist;
  std::vector<double> yawlist;
  std::vector<double> yaw1list;
  std::vector<bool> directions;
  double steer;
  Node3d* parent = NULL;
  double path_cost;
  double h_cost;
  double cost;  // total cost
 public:
  // default constructor
  Node3d() {
    xidx = 0;
    yidx = 0;
    yawidx = 0;
    direction = 0;
    xlist.push_back(0);
    ylist.push_back(0);
    yawlist.push_back(0);
    yaw1list.push_back(0);
    directions.push_back(0);
    steer = 0;  // steer input
    cost = std::numeric_limits<double>::max();
  }
  Node3d(int xidx, int yidx, int yawidx, int direction,
         std::vector<double> xlist, std::vector<double> ylist,
         std::vector<double> yawlist, std::vector<double> yaw1list,
         std::vector<bool> directions, double steer, Node3d* parent,
         double path_cost, double h_cost, double cost) {
    this->xidx = xidx;
    this->yidx = yidx;
    this->yawidx = yawidx;
    this->direction = direction;
    this->xlist = xlist;
    this->ylist = ylist;
    this->yawlist = yawlist;
    this->yaw1list = yaw1list;
    this->directions = directions;
    this->steer = steer;
    this->parent = parent;
    this->path_cost = path_cost;
    this->h_cost = h_cost;
    this->cost = cost;  // This is the total cost
  }
};