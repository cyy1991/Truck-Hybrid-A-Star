#include <cmath>
#include <iostream>
#include <limits>
#include <queue>
#include <vector>

using namespace std;

#define XY_GRID_RESOLUTION = 2.0  // [m]
#define YAW_GRID_RESOLUTION = deg2rad(15.0) #[rad]
#define GOAL_TYAW_TH = deg2rad(5.0) #[rad]
#define MOTION_RESOLUTION = 0.1 #[m] path interporate resolution
#define N_STEER = 20.0 #number of steer command
#define const EXTEND_AREA = 5.0 #[m] map extend length
#define SKIP_COLLISION_CHECK= 4 # skip number for collision check

#define SB_COST = 100.0 #switch back penalty cost
#define BACK_COST = 5.0 #backward penalty cost
#define STEER_CHANGE_COST = 5.0 #steer angle change penalty cost
#define STEER_COST = 1.0 #steer angle change penalty cost
#define JACKKNIF_COST = 200.0 #Jackknif cost
#define H_COST = 5.0 #Heuristic cost

#define WB = trailerlib.WB #[m] Wheel base
#define LT = trailerlib.LT #[m] length of trailer
#define MAX_STEER = trailerlib.MAX_STEER #[rad] maximum steering angle
/**
 * @brief: four dimensional node : x, y, yaw, yaw1
 *
 */
class Node4d {
 public:
  int xind;        // x index
  int yind;        // y index
  int yawind;      // yaw index
  bool direction;  //  moving direction forward : true, backword : false
  double x;        //
};
/**
 * @brief: Config struct for hybrid A* DB
 */
class Config {
 public:
  int minx;
  int miny;
  int minyaw;
  int minyawt;
  int maxx;
  int maxy;
  int maxyaw;
  int maxyawt;
  // int
};