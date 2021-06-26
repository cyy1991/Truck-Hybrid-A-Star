/*************************************************************************
 * File Name: truck trailer hybrid_a_star.cpp
 * Author: Yongyu Chen
 * Mail: yongyu.chen@tum.de
 ************************************************************************/

// https://gitee.com/herolin12/HybridAStarTrailer/blob/master/src/trailer_hybrid_a_star.jl
// https://gitee.com/herolin12/HybridAStarTrailer/blob/master/src/trailerlib.jl
#include <KDTreeVectorOfVectorsAdaptor.h>
#include <math.h>

#include <algorithm>
#include <iostream>
#include <nanoflann.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "configure.h"
#include "grid_a_star.h"
#include "kd_tree_common.h"
#include "kdtree.h"
#include "nanoflann.hpp"
#include "node3d.h"
#include "reeds_shepp.h"
#include "reeds_shepp_path.h"
#include "reeds_shepp_wrapper.h"
#include "trailerlib.h"
#include "vehicle.h"

using namespace std;
using namespace nanoflann;

#define XY_GRID_RESOLUTION 2.0  // [m]
// #define YAW_GRID_RESOLUTION deg2rad(15.0)  // [degree]
// #define GOAL_TYAW_TH deg2rad(5.0)
#define YAW_GRID_RESOLUTION 15.0 / 57.3
#define GOAL_TYAW_TH 5.0 / 57.3
#define MOTION_RESOLUTION 0.1
#define N_STEER 20.0
#define EXTEND_AREA 5.0
#define SKIP_COLLISION_CHECK 4

// costs
#define SB_COST 100.0          // switch back penalty cost
#define BACK_COST 5.0          // backward penalty cost
#define STEER_CHANGE_COST 5.0  // steer angle change penalty cost
#define STEER_COST 1.0         // steer angle cost
#define JACKKNIF_COST 200.0    // jackknif cost
#define H_COST 5.0             // heuristic cost

// vehicle parameter
#define WB 3.7
#define LT 8.0
#define MAX_STEER 0.6

// 使用map_server创建测试地图:
// https://blog.csdn.net/qq_43066145/article/details/107638543

struct TrailerRSPathNode {
  ReedSheppPath rs_path;
  double cost;
};

struct rs_path_node_cmp {
  bool operator()(const TrailerRSPathNode& a, const TrailerRSPathNode& b) {
    return a.cost < b.cost;
  }
};

// intialize the truck trailer model as global variable
TruckTrailer truck_trailer;

double mod2pi(double x) {
  double v = std::fmod(x, 2 * M_PI);
  if (v < -M_PI) {
    v = v + 2 * M_PI;
  } else if (v > M_PI) {
    v = v - 2 * M_PI;
  }
  return v;
}
/**
 *@brief: contruct the obstacles map
 *@todo: replace it with gridmap or costmap2d
 */
Config calc_config(std::vector<double> ox, std::vector<double> oy,
                   double xyreso, double yawreso) {
  // extend the obstacle map

  double min_x_m = *std::min_element(ox.begin(), ox.end()) - EXTEND_AREA;
  double min_y_m = *std::min_element(oy.begin(), oy.end()) - EXTEND_AREA;
  double max_x_m = *std::max_element(ox.begin(), ox.end()) + EXTEND_AREA;
  double max_y_m = *std::max_element(oy.begin(), oy.end()) + EXTEND_AREA;

  // add obstacles to the obstalces map
  ox.push_back(min_x_m);
  oy.push_back(min_y_m);
  ox.push_back(max_x_m);
  oy.push_back(max_y_m);

  // the following are indices
  int minx = std::round(min_x_m / xyreso);
  int miny = std::round(min_y_m / xyreso);
  int maxx = std::round(max_x_m / xyreso);
  int maxy = std::round(max_y_m / xyreso);
  // width of the map
  int xw = std::round(maxx - minx);
  int yw = std::round(maxy - miny);

  int minyaw = std::round(-M_PI / yawreso) - 1;
  int maxyaw = std::round(M_PI / yawreso);

  int yaww = std::round(maxyaw - minyaw);

  int minyawt = minyaw;
  int maxyawt = maxyaw;
  int yawtw = yaww;

  Config config(minx, miny, minyaw, minyawt, maxx, maxy, maxyaw, maxyawt, xw,
                yw, yaww, yawtw, xyreso, yawreso);
  return config;
}

/**
 *@brief: return  the discrete motion inputs
 *@return: motion inputs of the steer angle
 */

std::vector<std::pair<double, double>> CalcMotionInputs() {
  double steer_reso = MAX_STEER / N_STEER;
  std::vector<std::pair<double, double>> motion_inputs;
  std::vector<double> up;
  // 这里可以变成
  for (double st = steer_reso; st <= MAX_STEER; st += steer_reso) {
    up.push_back(st);
  }
  up.push_back(0.0);  // add zero steer angle situation
  for (int i = 0; i < 2 * up.size() - 1; ++i) {
    if (i < up.size()) {
      motion_inputs.push_back(std::make_pair(up[i], 1.0));
    } else {
      motion_inputs.push_back(std::make_pair(up[i], -1.0));
    }
  }

  return motion_inputs;
}

int CalcIndex(const Node3d& node, const Config& cfg) {
  int ind = (node.yawidx - cfg.minyaw) * cfg.xw * cfg.yw +
            (node.yidx - cfg.miny) * cfg.xw + (node.xidx - cfg.minx);

  // 4D Grid
  int yaw1ind = std::round(node.yaw1list.back() / cfg.yawreso);
  if (ind <= 0) std::cout << "Error [Calc Index]" << std::endl;
  return ind;
}

/**
 *@brief: calculate the rs path cost based on different criteria
 *@return: the total cost of a path
 */
double calc_rs_path_cost(ReedSheppPath* rspath, std::vector<double> yaw1) {
  double cost = 0.0;
  // 1. length cost
  for (size_t i = 0; i < rspath->segs_lengths.size(); ++i) {
    if (rspath->segs_lengths[i] >= 0) {  // forward
      cost += rspath->segs_lengths[i];
    } else {
      cost += abs(rspath->segs_lengths[i]) * BACK_COST;
    }
  }
  // 2. switch back penalty
  for (size_t i = 0; i < rspath->segs_lengths.size() - 1; ++i) {
    if (rspath->segs_lengths[i] * rspath->segs_lengths[i + 1] < 0.0) {
      cost += SB_COST;
    }
  }

  // 3. steer penalty
  for (size_t i = 0; i < rspath->segs_types.size(); ++i) {
    if (rspath->segs_types[i] != 'S') {  // curve
      cost += STEER_COST * abs(MAX_STEER);
    }
  }
  // 4. steer change penalty
  int nctypes = rspath->segs_types.size();
  vector<double> ulist(nctypes, 0.0);
  for (size_t i = 0; i < rspath->segs_types.size(); ++i) {
    if (rspath->segs_types[i] == 'R') {
      ulist[i] = -MAX_STEER;
    } else if (rspath->segs_types[i] == 'L') {
      ulist[i] = MAX_STEER;
    }
  }
  for (size_t i = 0; i < rspath->segs_types.size() - 1; ++i) {
    cost += STEER_CHANGE_COST * std::abs(ulist[i + 1] - ulist[i]);
  }
  // 5. jacknife cost
  double yaw_diff_sum = 0.0;
  for (size_t i = 0; i < rspath->phi.size(); ++i) {
    yaw_diff_sum += abs(mod2pi(rspath->phi[i] - yaw1[i]));
  }
  cost += JACKKNIF_COST * yaw_diff_sum;

  return cost;
}
// helper functions
// Writing the following function into a Separate Function
template <typename T>
vector<T> scaleVector(vector<T>& vec, T scale) {
  for (size_t i = 0; i < vec.size(); ++i) {
    vec[i] *= vec[i] * scale;
  }
}
template <typename T>
vector<T> minusVector(vector<T>& vec1, vector<T>& vec2) {
  vector<T> res;
  if (vec1.size() != vec2.size()) return {};
  for (int i = 0; i < vec1.size(); ++i) {
    res.push_back(vec1[i] - vec2[i]);
  }
}
template <typename T>
vector<T> absVector(vector<T>& vec) {
  vector<T> res;
  for (int i = 0; i < vec.size(); ++i) {
    res.push_back(abs(vec[i]));
  }
}

std::vector<double> calcTrailerYaw(std::vector<double>& yaw, double yawt0,
                                   std::vector<double>& steps) {
  std::vector<double> yawt(yaw.size());
  yawt[0] = yawt0;

  for (int i = 1; i < yaw.size(); ++i) {
    yawt[i] += yawt[i - 1] + steps[i - 1] / LT *
                                 (std::sin(yaw[i - 1]) - std::sin(yawt[i - 1]));
  }
  return yawt;
}

ReedSheppPath* AnalyticExpantion(Node3d* current, Node3d* ngoal, Config& config,
                                 std::vector<double>& ox,
                                 std::vector<double>& oy, kd_tree_t* kdtree) {
  double sx = current->xlist.back();
  double sy = current->ylist.back();
  double syaw = current->yawlist.back();
  double max_curvature = tan(MAX_STEER) / WB;
  double gx = ngoal->xlist.back();
  double gy = ngoal->ylist.back();
  double gyaw = ngoal->yaw1list.back();
  std::vector<ReedSheppPath> paths;
  double start_node[3] = {sx, sy, syaw};
  double end_node[3] = {gx, gy, gyaw};

  ReedShepp rs_path(max_curvature);
  if (!rs_path.GenerateRSPs(start_node, end_node, &paths)) {
    std::cout << "Fail to generate different combination of Reed Shepp"
              << std::endl;
  }
  std::priority_queue<TrailerRSPathNode, std::vector<TrailerRSPathNode>,
                      rs_path_node_cmp>
      pq_rspath;

  for (ReedSheppPath path : paths) {
    // vector<double> steps = MOTION_RESOLUTION * path.gear;
    vector<double> steps =
        scaleVector<double>(path.gear, (double)MOTION_RESOLUTION);
    vector<double> yaw_t =
        calcTrailerYaw(path.phi, current->yaw1list.back(), steps);
    pq_rspath.push({path, calc_rs_path_cost(&path, yaw_t)});
  }
  // 这一块相当的消耗时间 
  while (!pq_rspath.empty()) {
    auto path = pq_rspath.top();
    pq_rspath.pop();

    std::vector<double> steps =
        scaleVector(path.rs_path.gear, MOTION_RESOLUTION);

    std::vector<double> yaw_t =
        calcTrailerYaw(path.rs_path.phi, current->yaw1list.back(), steps);

    /* To Speed up, we do not need to check collision for
       all points
     */
    std::vector<double> check_x;
    std::vector<double> check_y;
    std::vector<double> check_yaw;
    std::vector<double> check_yawt;

    for (int i = 0; i < path.rs_path.x.size(); i += SKIP_COLLISION_CHECK) {
      check_x.push_back(path.rs_path.x[i]);
      check_y.push_back(path.rs_path.y[i]);
      check_yaw.push_back(path.rs_path.phi[i]);
      check_yawt.push_back(yaw_t[i]);
    }

    if (truck_trailer.CheckTrailerCollision(ox, oy, check_x, check_y, check_yaw,
                                            check_yawt, kdtree))
      return &path.rs_path;
  }

  return NULL;
}

bool UpdateNodeWithAnalysticExpantion(Node3d* current, Node3d* ngoal,
                                      Config& config, std::vector<double>& ox,
                                      std::vector<double>& oy,
                                      kd_tree_t* kdtree, double gyaw1,
                                      Node3d* fpath) {
  ReedSheppPath* apath =
      AnalyticExpantion(current, ngoal, config, ox, oy, kdtree);
  // if apath is generated successfully, we should build the node
  if (apath != NULL) {
    std::vector<double> fx(apath->x.begin() + 1, apath->x.end());
    std::vector<double> fy(apath->y.begin() + 1, apath->y.end());
    std::vector<double> fyaw(apath->phi.begin() + 1, apath->phi.end());
    std::vector<double> steps =
        scaleVector<double>(apath->gear, (double)MOTION_RESOLUTION);

    std::vector<double> yaw_t =
        calcTrailerYaw(apath->phi, current->yaw1list.back(), steps);

    if (std::abs(mod2pi(yaw_t.back() - gyaw1)) > GOAL_TYAW_TH)
      return NULL;  // no update

    double fcost = current->cost + calc_rs_path_cost(apath, yaw_t);

    std::vector<double> fyaw1(yaw_t.begin() + 1, yaw_t.end());

    int fpind = CalcIndex(*current, config);

    std::vector<double> fd(apath->gear.begin() + 1, apath->gear.end());
    // std::vector<double> directions(apath->gear.begin() + 1,
    // apath->gear.end());
    double fsteer = 0.0;
    // TODO: fpind should be the Node
    fpath = new Node3d(current->xidx, current->yidx, current->yawidx,
                       current->direction, fx, fy, fyaw, fyaw1, fd, fsteer,
                       fcost, fpind);

    return true;
  }
  return false;
}
// 计算下一个节点的方法
Node3d* CalcNextNode(Node3d* current, int c_id, double u, double d,
                     const Config& config) {
  double arc_l = XY_GRID_RESOLUTION * 1.5;
  int nlist = std::floor(arc_l / MOTION_RESOLUTION) + 1;
  std::vector<double> xlist(nlist, 0);
  std::vector<double> ylist(nlist, 0);
  std::vector<double> yawlist(nlist, 0);
  std::vector<double> yaw1list(nlist, 0);

  xlist[0] = current->xlist.back() +
             d * MOTION_RESOLUTION * cos(current->yawlist.back());
  ylist[0] = current->xlist.back() +
             d * MOTION_RESOLUTION * sin(current->yawlist.back());
  yawlist[0] = mod2pi(current->yawlist.back());
  yaw1list[0] = mod2pi(current->yaw1list.back());

  for (int i = 0; i < nlist - 1; ++i) {
    xlist[i + 1] = xlist[i] + d * MOTION_RESOLUTION * cos(yawlist[i]);
    ylist[i + 1] = ylist[i] + d * MOTION_RESOLUTION * sin(yawlist[i]);
    yawlist[i + 1] = mod2pi(yawlist[i] + d * MOTION_RESOLUTION / WB * tan(u));
    yaw1list[i + 1] = mod2pi(yaw1list[i] + d * MOTION_RESOLUTION / LT *
                                               sin(yawlist[i] - yaw1list[i]));
  }

  int xind = std::round(xlist.back() / config.xyreso);
  int yind = std::round(ylist.back() / config.xyreso);
  int yawind = std::round(yawlist.back() / config.yawreso);

  double addedcost = 0.0;
  double direction;
  if (d > 0.0) {
    direction = 1.0;
    addedcost += abs(arc_l);
  } else {
    direction = -1.0;
    addedcost += abs(arc_l) * BACK_COST;
  }
  if (direction != current->direction) {  // Switch back penalty
    addedcost += SB_COST;
  }
  // steer penalty
  addedcost += STEER_COST * abs(u);
  // steer change penalty
  addedcost += STEER_CHANGE_COST * abs(current->steer - u);
  // jacknife cost

  addedcost += JACKKNIF_COST * 1.0;
}
bool isSameGrid(const Node3d& node1, const Node3d& node2) {
  if (node1.xidx != node2.xidx) return false;
  if (node1.yidx != node2.yidx) return false;
  if (node1.yawidx != node2.yawidx) return false;
  return true;
}

bool VerifyIndex(Node3d* node, Config c, std::vector<double> ox,
                 std::vector<double> oy, double inityaw1, kd_tree_t) {
  // overflow map
  if (node->xidx - c.minx >= c.xw) {
    return false;
  } else if (node->xidx - c.minx <= 0) {
    return false;
  } else if (node->yidx - c.miny >= c.yw) {
    return false;
  } else if (node->yidx - c.miny <= 0) {
    return false;
  }
  // check collision
  // double steps =
  return true;
}

/**
 * @brief: main function for truck trailer hybrid a star
 * @param: sx: start x position [m]
 * @param: sy: start y position [m]
 * @param: gx: goal x position [m]
 * @param: gy: goal y position [m]
 * @param: ox: x position list of Obstacles [m]
 * @param: oy: y position list of Obstacles [m]
 * @param: xyreso: grid resolution [m]
 * @param: yawreso: yaw angle resolution [rad]
 */

bool HybridAStarPlan(double sx, double sy, double syaw, double syaw1, double gx,
                     double gy, double gyaw, double gyaw1,
                     std::vector<double> ox, std::vector<double> oy,
                     double xyreso, double yawreso) {
  // 1. normalize the heading angle
  syaw = mod2pi(syaw);
  gyaw = mod2pi(gyaw);

  // 2. establish the kd tree of obstacles
  vector_of_vectors_t obstalces;
  int N = ox.size();  // TODO: check if size of ox equals size of oy
  obstalces.resize(N);
  for (size_t i = 0; i < N; ++i) {
    obstalces[i].resize(2);
    obstalces[i][0] = ox[i];
    obstalces[i][1] = oy[i];
  }
  int state_dim = 2;  // x and y
  int num_samples = ox.size();

  kd_tree_t mat_index(state_dim, obstalces, 10);
  // 3. establish the configuration
  // Using map Server or other
  Config c = calc_config(ox, oy, xyreso, yawreso);

  // 4. establish the start node and goal node
  Node3d* nstart =
      new Node3d(int(sx / xyreso), int(sy / xyreso), int(syaw / yawreso), true,
                 {sx}, {sy}, {syaw}, {syaw1}, {true}, 0.0, NULL, 0.0, 0.0, 0.0);
  Node3d* ngoal =
      new Node3d(int(gx / xyreso), int(gy / xyreso), int(gyaw / yawreso), true,
                 {gx}, {gy}, {gyaw}, {gyaw1}, {true}, 0.0, NULL, 0.0, 0.0, 0.0);

  // initialize the open set and the closed set
  // TODO: add the function
  // double h_dp = calc_holonomic_with_obstacle_heuristic();

  std::vector<std::vector<double>> h_dp =
      calc_dist_policy(gx, gy, ox, oy, xyreso, 1.0);

  auto motion_inputs = CalcMotionInputs();
  struct CompareNodes {
    // Sorting 2D Nodes by increasing C value - the total estimated cost
    bool operator()(const Node3d* lhs, const Node3d* rhs) const {
      return lhs->cost > rhs->cost;
    }
  };
  // Using binomial heap to build the
  typedef boost::heap::binomial_heap<Node3d*,
                                     boost::heap::compare<CompareNodes>>
      priorityQueue;
  typedef boost::heap::binomial_heap<
      Node3d*, boost::heap::compare<CompareNodes>>::handle_type handleType;

  priorityQueue open_pq;
  // hash mappings
  std::unordered_map<int, Node3d*> open_set;
  std::unordered_map<int, Node3d*> closed_set;
  // We do not need open set handle
  std::unordered_map<int, handleType> open_set_handle;

  // learning how to use handle of binary heap
  handleType handle = open_pq.push(nstart);
  open_set[CalcIndex(*nstart, c)] = nstart;
  open_set_handle[CalcIndex(*nstart, c)] = handle;

  std::vector<std::pair<double, double>> motion_inputs = CalcMotionInputs();
  int nmotion = motion_inputs.size();
  

  // open_set_handle[]
  while (!open_pq.empty()) {
    // extract the start node from the open set
    Node3d* current = open_pq.top();
    // move current node from open to closed
    int c_id = CalcIndex(*current, c);

    delete !(open, c_id)
    closed[c_id] = current; 
    isupdated, f_path = update_node_with_analystic_expantion(current, ngoal, c, ox, oy, kdtree, gyaw1); 
    if isupdated // found 
       fnode = fpath 
       break
    end 

    init_yaw1 = current.yaw1; 


    for i in 1 : nmotion // 从不同的方向进行expantion, 产生不同的节点 
        node = calc_next_node(current, c_id, u[i], d[i], c)

        if (!VerifyIndex(node, c, ox, oy, inityaw1, kdtree)) continue; 
        node_ind  = calc_index(node, c)

        // 如果在closed set中找到, 那么直接skip 
        if haskey(closed, node_id) continue;

        if (!haskey(open, node_ind)) 
                open[node_id] = node
                enqueue!(pq, node_ind, calc_cost(node, h_dp, ngoal, c))
        else {
                          if open[node_ind].cost > node.cost
                    # If so, update the node to have a new parent
                    open[node_ind] = node
                end

        }        
        
        
  }

  // reconstruct the path 


  return true;
}

int main(int argc, char* argv[]) {
  std::cout << "Hybrid A star for truck trailer begins:" << std::endl;

  double a = 1.0;
  double sx = 14.0, sy = 10.0;

  std::vector<double> ox;
  std::vector<double> oy;
  return 0;
}

//   struct cmp {
//     bool operator()(const std::pair<std::string, double>& left,
//                     const std::pair<std::string, double>& right) const {
//       return left.second >= right.second;
//     }
//   };
//   std::priority_queue<std::pair<std::string, double>,
//                       std::vector<std::pair<std::string, double>>, cmp>
//       open_pq_;
//   std::unordered_map<std::string, std::shared_ptr<Node3d>> open_set_;
//   std::unordered_map<std::string, std::shared_ptr<Node3d>> close_set_;
//   std::unique_ptr<ReedShepp> reed_shepp_generator_;
//   std::unique_ptr<GridSearch> grid_a_star_heuristic_generator_;