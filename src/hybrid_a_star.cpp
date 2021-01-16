/*************************************************************************
 * File Name: truck trailer hybrid_a_star.cpp
 * Author: Yongyu Chen
 * Mail: yongyu.chen@tum.de
 ************************************************************************/
#include <nanoflann.hpp>
#include <KDTreeVectorOfVectorsAdaptor.h>
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <unordered_map>
#include <math.h>

#include "kdtree.h"
#include "reeds_shepp.h"
#include "reeds_shepp_wrapper.h"
#include "reeds_shepp_path.h"
#include "configure.h"
#include "node3d.h"
#include "vehicle.h"
#include "trailerlib.h"
#include "nanoflann.hpp"
#include "grid_a_star.h"


using namespace std;
using namespace nanoflann; 

#define XY_RESOLUTION 2.0 // [m]
#define YAW_GRID_RESOLUTION deg2rad(15.0) // [degree]
#define GOAL_TYAW_TH deg2rad(5.0)
#define MOTION_RESOLUTION 0.1
#define N_STEER 20.0
#define EXTEND_AREA 5.0
#define SKIP_COLLISION_CHECK 4

// costs
#define SB_COST 100.0 // switch back penalty cost 
#define BACK_COST 5.0 // backward penalty cost 
#define STEER_CHANGE_COST 5.0 // steer angle change penalty cost 
#define STEER_COST 1.0 // steer angle cost 
#define JACKKNIF_COST 200.0 // jackknif cost 
#define H_COST 5.0 // heuristic cost 

// vehicle parameter 
#define WB 3.7
#define LT 8.0  
#define MAX_STEER 0.6 

typedef KDTreeVectorOfVectorsAdaptor<vector_of_vectors_t, double> kd_tree_t;

struct TrailerRSPathNode {
    ReedSheppPath rs_path; 
    double cost;
};

struct rs_path_node_cmp {
    bool operator() (const TrailerRSPathNode& a, const TrailerRSPathNode& b) {
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
// TODO: replace it with gridmap or costmap2d
/**
 *@brief: contruct the obstacles map
 */ 
Config calc_config(std::vector<double> ox, std::vector<double>oy, 
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
    double minx = std::round(min_x_m / xyreso);
    double miny = std::round(min_y_m / xyreso);
    double maxx = std::round(max_x_m / xyreso);
    double maxy = std::round(max_y_m / xyreso);
    // width of the map
    double xw = std::round(maxx - minx);
    double yw = std::round(maxy - miny);

    double minyaw = std::round(-M_PI / yawreso) - 1;
    double maxyaw = std::round(M_PI / yawreso);

    double yaww = std::round(maxyaw - minyaw);
    
    double minyawt = minyaw;
    double maxyawt = maxyaw;
    double yawtw = yaww;

    Config config(minx, miny, minyaw, minyawt, maxx, maxy, 
                  maxyaw, maxyawt, xw, yw, yaww, yawtw, xyreso,
                  yawreso);
    return config; 
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

bool hybrid_a_star_planning(double sx, double sy, double syaw, double syaw1, 
                            double gx, double gy, double gyaw, double gyaw1, 
                            std::vector<double> ox, std::vector<double> oy, 
                            double xyreso, double yawreso) {
    // 1. normalize the heading angle 
    syaw = mod2pi(syaw);
    gyaw = mod2pi(gyaw);

    // 2. establish the kd tree of obstacles
    vector_of_vectors_t obstalces;
    int N = ox.size(); // TODO: check if size of ox equals size of oy
    obstalces.resize(N);
    for (size_t i = 0; i < N; ++i) {
        obstalces[i].resize(2);
        obstalces[i][0] = ox[i];
        obstalces[i][1] = oy[i];
    }
    int state_dim = 2; // x and y
    int num_samples = ox.size(); 
    

    kd_tree_t mat_index(state_dim, obstalces, 10);
    // 3. establish the configuration
    Config c = calc_config(ox, oy, xyreso, yawreso);

    // 4. establish the start node and goal node
    Node3d* nstart = new Node3d(int(sx / xyreso), int(sy / xyreso), int(syaw / yawreso), true,
                            {sx}, {sy}, {syaw}, {syaw1}, {true}, 
                            0.0, NULL, 0.0, 0.0, 0.0
                            );
    Node3d* ngoal = new Node3d(int(gx / xyreso), int(gy / xyreso), int(gyaw / yawreso), true, 
                            {gx}, {gy}, {gyaw}, {gyaw1}, {true}, 0.0, NULL, 0.0, 0.0, 0.0);
    

    // initialize the open set and the closed set 
    // TODO: add the function    
    // double h_dp = calc_holonomic_with_obstacle_heuristic();
    
    std::vector<std::vector<double>> h_dp 
                = calc_dist_policy(gx, gy, ox, oy, xyreso, 1.0);
    // while (!open_pq) {

    // }

    return true;
}
/**
 *@brief: return  the discrete motion inputs
 *@return: motion inputs of the steer angle 
 */ 

 std::vector<std::pair<double, double>> calc_motion_inputs() {
    double steer_reso = MAX_STEER / N_STEER;
    std::vector<std::pair<double, double>> motion_inputs; 
    std::vector<double> up; 
    for (double st = steer_reso ; st <= MAX_STEER; st += steer_reso) {
        up.push_back(st);
    }
    up.push_back(0.0);   // add zero steer angle situation
    for (int i = 0; i < 2 * up.size() - 1; ++i) {
        if (i < up.size()) {
            motion_inputs.push_back(std::make_pair(up[i], 1.0));
        } else {
            motion_inputs.push_back(std::make_pair(up[i], -1.0));
        }
    }

    return motion_inputs;
}

/**
 *@brief: calculate the rs path cost based on different criteria
 *@return: the total cost of a path 
 */ 
double calc_rs_path_cost(ReedSheppPath* rspath, std::vector<double> yaw1) {
    double cost = 0.0;
    // 1. length cost 
    for (size_t i = 0; i < rspath->segs_lengths.size(); ++i) {
        if (rspath->segs_lengths[i] >= 0) { // forward
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
        if (rspath->segs_types[i] != 'S') { // curve
            cost += STEER_COST * abs(MAX_STEER);
        }
    }
    // 4. steer change penalty
    int nctypes = rspath->segs_types.size();
    vector<double> ulist(nctypes, 0.0);
    for (size_t i = 0; i < rspath->segs_types.size(); ++i) {
        if (rspath->segs_types[i] == 'R') {
            ulist[i] = - MAX_STEER;
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
        yaw_diff_sum +=  abs(mod2pi(rspath->phi[i] - yaw1[i]));
    }
    cost += JACKKNIF_COST * yaw_diff_sum;


    return cost; 
}

ReedSheppPath analytic_expantion(Node3d* current, Node3d* ngoal, 
                                Config& config,  std::vector<double>& ox, 
                                std::vector<double>& oy, kd_tree_t& kdtree) {
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
        std::cout << "Fail to generate different combination of Reed Shepp" << std::endl;
    }
    std::priority_queue<TrailerRSPathNode, std::vector<TrailerRSPathNode>, rs_path_node_cmp> pq_rspath; 
    // pq_rspath.
    
    
    return paths[0];

}
Node3d* calc_next_node(Node3d* current, int c_id, double u, double d, const Config& config) {
    double arc_l = XY_RESOLUTION * 1.5;
    int nlist = std::floor(arc_l / MOTION_RESOLUTION) + 1;
    std::vector<double> xlist(nlist, 0);
    std::vector<double> ylist(nlist, 0); 
    std::vector<double> yawlist(nlist, 0); 
    std::vector<double> yaw1list(nlist, 0); 

    xlist[1] = current->xlist.back() + d * MOTION_RESOLUTION * cos(current->yawlist.back());

    // ylist[1] = current. 


}

// bool verify_index(Node3d* node, Config c, std::vector<double> ox,
//                   std::vector<double> oy, double inityaw1, kd_tree_t) {
//     // overflow map 
//     if (node.xidx - c.minx >= c.xw) {
//         return false;
//     } else if (node.xidx - c.minx <= 0) {
//         return false;
//     } else if (node.yind - c.miny >= c.yw) {
//         return false;
//     } else if (node.yind - c.miny <= 0) {
//         return false;
//     }
//     // check collision
//     double steps = 
// }
// bool update_node_with_analytic_expantion(Node3d* current, Node3d* ngoal, Config& config)

// bool CalcIdx(double x, double y, double theta, Configure& cfg, 
//                                 int xidx, int yidx, int thidx) {
//     double gres = cfg.xy_grid_resolution;
//     double yawres = cfg.yaw_grid_resolution;
//     xidx = std::ceil(x - cfg.min_x / gres);
//     yidx = std::ceil(y - cfg.min_y / gres);
//     double theta = mod2pi(theta); // in the range [-M_PI, M_PI]
//     thidx = std::ceil(theta - cfg.min_yaw / yawres);

//     if (xidx <= 0 || xidx > std::ceil(cfg.max_x - cfg.min_x) /gres);
//         return false;
//     else if ()
    
// }


int main(int argc, char* argv[]) {
    std::cout << "Hybrid A star for truck trailer begins:" << std::endl;
    double sx = 14.0, sy = 10.0; 
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