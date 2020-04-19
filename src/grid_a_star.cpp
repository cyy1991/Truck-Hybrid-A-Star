
/*************************************************************************
 * File Name: grid_a_star.cpp
 * Author: Yongyu Chen
 * Mail: yongyu.chen@tum.de
 ************************************************************************/
// https://www.cnblogs.com/sssblog/p/11025031.html

#include <iostream>
// #include <cmath>
#include <limits>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <matplotlibcpp.h>
#include <unordered_map>

#include <boost/heap/binomial_heap.hpp>

using namespace std;
namespace plt = matplotlibcpp;
/**
 * @brief: Grid base A* shortest path planning
 */ 
class Node2d {
public:
    int x; // x index of the grid
    int y; // y index of the grid 
    double sum_cost; // cost, euclidean distance 
    double path_cost; // from start node to the current node
    double h_cost; // from 
    Node2d* p_node; // parent node, or replaced by the 

    // constructor 
    Node2d(int x_, int y_, double sum_cost_ = 0, double path_cost_ = 0, double h_cost_ = 0, 
        Node2d* p_node_ = nullptr): x(x_), y(y_),
        sum_cost(sum_cost_), path_cost(path_cost_), h_cost(h_cost_), p_node(p_node_){};
};

int calc_xyindex(double position, double min_pos, double reso) {
  return std::round((position - min_pos) / reso);
}
int calc_grid_index(Node2d& node, int xwidth, int ywidth) {
  return node.y * xwidth + node.x;  
}
double calc_grid_position(int index, double minp, double reso) {
  return index * reso + minp;
}

/**
 * @brief:construct the final path after the algorithm is done 
 */ 
std::vector<std::vector<double>> calc_final_path(Node2d* goal, double minx, double miny, 
                                                    double reso) {
    std::vector<double> rx;
    std::vector<double> ry;
    Node2d* node = goal;
    while (node->p_node != nullptr) { // while the parent node exists
        node = node->p_node;
        rx.push_back(calc_grid_position(node->x, minx, reso));
        ry.push_back(calc_grid_position(node->y, miny, reso));
    }
    return {rx, ry};
}
/**
 * @brief: construct the realtime obstacle map
 */ 
// obstacle map 是一个二维数组包含0和１ 
std::vector<std::vector<int> > calc_obstacle_map(
      vector<double>& ox, vector<double>& oy, 
      int& xwidth, int& ywidth, double& minx, 
      double& miny, double& maxx, double& maxy, 
      double reso, double vr) {

      minx = std::round(*min_element(ox.begin(), ox.end()));
      miny = std::round(*min_element(oy.begin(), oy.end()));
      maxx = std::round(*max_element(ox.begin(), ox.end()));
      maxy = std::round(*max_element(oy.begin(), oy.end()));
      // xwidth,  ywidth are the integer indices
      xwidth = std::round((maxx - minx) / reso);
      ywidth = std::round((maxy - miny) / reso);
      std::cout << "minx: " << minx << std::endl;
      std::cout << "miny: " << miny << std::endl;
      std::cout << "maxx: " << maxx << std::endl;
      std::cout << "maxy: " << maxy << std::endl;
 
      std::cout << "xwidth: " << xwidth << std::endl;
      std::cout << "ywidth: " << ywidth << std::endl;
      std::vector<std::vector<int> >obmap(ywidth, std::vector<int>(xwidth, 0));
     
      for (int i = 0; i < xwidth; i++) {
          double x = calc_grid_position(i, minx, reso);
          for (int j = 0; j < ywidth; j++) {
          double y = calc_grid_position(j, miny, reso);   
          for (int k = 0; k < ox.size(); k++) {
            double d = std::sqrt(std::pow((ox[k] - x), 2) + std::pow((oy[k] - y), 2));
            if (d <= vr) {
              obmap[i][j] = 1;
              break; 
            }
          }
        }
      }  
      for (int i = 0; i < obmap.size(); i++) {
        for (int j = 0; j < obmap[0].size(); j++) {
          std::cout << obmap[i][j] << " ";
        }
        std::cout << std::endl;
      }
      return obmap;
}
/**
 * @brief: construct the heuristic function
 */ 
double calc_heuristic(Node2d* n1, Node2d* n2, float w=1.0){
  return w * std::sqrt(std::pow(n1->x-n2->x, 2)+std::pow(n1->y-n2->y, 2));
}
/**
 * @brief: construct the path cost
 */ 
double calc_pathcost(Node2d* n1, Node2d* n2, float w=1.0){
  return w * std::sqrt(std::pow(n1->x-n2->x, 2)+std::pow(n1->y-n2->y, 2));
}
/**
 * @brief: verify the node if it is collide with other obstacles
 * 
 */ 

bool verify_node(Node2d* node, double minx, double miny, 
                double maxx, double maxy, double reso, 
                std::vector<std::vector<int>>& obmap) {
    // within the range 
    double px = calc_grid_position(node->x, minx, reso);
    double py = calc_grid_position(node->y, miny, reso);   

    if (px < minx) return false;
    else if (py < miny) return false;
    else if (px >= maxx) return false;
    else if (py >= maxy) return false;

    // collision check
    if (obmap[node->x][node->y]) {
        return false;
    }
    return true;
} 
/**
 *@brief: Get the motion model: totally 8 moving directions 
 */ 
std::vector<Node2d> get_motion_model() {
    return {Node2d(1, 0, 0, 1, 0),
            Node2d(0, 1, 0, 1, 0),
            Node2d(-1, 0, 0, 1, 0),
            Node2d(0, -1, 0, 1, 0),
            Node2d(-1, -1, 0, std::sqrt(2), 0),
            Node2d(-1, 1, 0, std::sqrt(2), 0),
            Node2d(1, -1, 0, std::sqrt(2), 0),
            Node2d(1, 1, 0, std::sqrt(2), 0)
    };
}

/**
 * @brief: main function for a star path planning 
 * @param: sx: start x position [m]
 * @param: sy: start y position [m]  
 * @param: gx: goal x position [m]
 * @param: gy: goal y position [m]
 */ 
std::vector<std::vector<double>>  a_star_planning(double sx, double sy,
                     double gx, double gy,
                     vector<double> ox_, 
                     vector<double> oy_,
                     double reso, double rr) {
    // 1. construct the obstacle map
    int xwidth;
    int ywidth;
    double minx, miny, maxx, maxy; 

    std::vector<std::vector<int> > obmap = calc_obstacle_map(
                                        ox_, oy_, xwidth, ywidth,
                                        minx, miny, maxx, maxy, 
                                        reso, rr);


    // 2. initialize the start and goal node 
    Node2d* nstart = new Node2d(calc_xyindex(sx, minx, reso), calc_xyindex(sy, miny, reso), 0.0, 0.0, 0.0);
    Node2d* ngoal = new Node2d(calc_xyindex(gx, minx, reso), calc_xyindex(gy, miny, reso), 0.0, 0.0, 0.0);

    nstart->h_cost = calc_heuristic(nstart, ngoal);
    nstart->path_cost = calc_pathcost(nstart, nstart);
    nstart->sum_cost = nstart->h_cost + nstart->path_cost;

    ngoal->h_cost = calc_heuristic(ngoal, ngoal);
    ngoal->path_cost = calc_pathcost(nstart, ngoal);
    ngoal->sum_cost = ngoal->h_cost + ngoal->path_cost; 
    std::cout << "start x index: " << nstart->x << std::endl; 
    std::cout << "start y index: " << nstart->y << std::endl; 

    std::cout << "goal x index: " << ngoal->x << std::endl;
    std::cout << "goal y index: " << ngoal->y << std::endl; 
    // 3.initialize the motion model 
    std::vector<Node2d> motion = get_motion_model();

    // 4. initialize the open set and the closed set
    auto cmp = [](const Node2d* left, const Node2d* right){return left->sum_cost > right->sum_cost;};
    struct CompareNodes {

    /// Sorting 2D nodes by increasing C value - the total estimated cost
    bool operator()(const Node2d* lhs, const Node2d* rhs) const {
    return lhs->sum_cost > rhs->sum_cost;
    }
    };
    typedef boost::heap::binomial_heap<Node2d*, boost::heap::compare<CompareNodes>> priorityQueue;
    typedef boost::heap::binomial_heap<Node2d*, boost::heap::compare<CompareNodes>>::handle_type handleType; 
    // std::priority_queue<Node2d*, std::vector<Node2d*>, decltype(cmp)> open_pq(cmp); // open set queue, frontier queue 
    priorityQueue open_pq; 
    std::unordered_map<int, Node2d*> open_set;
    std::unordered_map<int, Node2d*> closed_set;   
    std::unordered_map<int, handleType> open_set_handle;    
    handleType handle = open_pq.push(nstart);
    open_set[calc_grid_index(*nstart, xwidth, ywidth)] = nstart;
    open_set_handle[calc_grid_index(*nstart, xwidth, ywidth)] = handle;   
    // std::cout << open_pq.size() << std::endl;
    std::cout << open_set.size() << std::endl; 
    while (!open_pq.empty()) {
      // 1. extract the start node from the open set (priority queue)
      Node2d* current = open_pq.top();
      int c_id = calc_grid_index(*current, xwidth, ywidth);
      // std::cout << "open pq size: " << open_pq.size() << std::endl;
      std::cout << "c_id: " << c_id << std::endl; 

      // show graph 
      plt::plot({calc_grid_position(current->x, minx, reso)},{calc_grid_position(current->y, miny, reso)},"xc");

      if ((closed_set.size())% 10 == 0) {
        plt::pause(0.001); 
      } 


      // 2. if the current node is the goal node, break
      if (current->x == ngoal->x && current->y == ngoal->y) {
        ngoal->sum_cost = current->sum_cost;
        ngoal->p_node = current;  
        break;  
      }
      // 3. remove the item from the open set
      open_set.erase(c_id);
      open_pq.pop();
      // 4. add the current node into the closed set 
      closed_set[c_id] = current;
        
      // 5. expand the motions in eight directions
      for (int i = 0; i < motion.size(); i++) {
         Node2d* new_node = new Node2d(
         current->x + motion[i].x, 
         current->y + motion[i].y, 0.0, 
         current->path_cost + motion[i].path_cost, 0.0, current); // here, we just use the path_cost 
         int n_id = calc_grid_index(*new_node, xwidth,  ywidth);
        // std::cout << "n_id: " << n_id << std::endl;
        // 1. if the current grid contains obstacles, skip and continue 
        if (!verify_node(new_node, minx, miny, 
                maxx, maxy, reso, obmap)) {
            delete new_node;
            continue;      
        }
        // 2. if the current node has been visited, in the closed set 
        //    skip and continue 
        if (closed_set.count(n_id)) {
          delete new_node;
          continue;
        }
        // 3. if the expanded node is not in the open set, put it into the open set 
        if (!open_set.count(n_id)) {
          new_node->h_cost = calc_heuristic(new_node, ngoal);
          new_node->sum_cost = new_node->h_cost + new_node->path_cost; 
          open_set[n_id] = new_node;
          handleType handle = open_pq.push(new_node);
          open_set_handle[n_id] = handle;     
        } 
        else { // 4. If current node cost is smaller than the existed one in the open set, replace it 
          if (open_set[n_id]->path_cost > new_node->path_cost) { // 
            new_node->h_cost = calc_heuristic(new_node, ngoal);
            new_node->sum_cost = new_node->h_cost + new_node->path_cost; 
            open_set[n_id] = new_node; // replace the new_node
            open_pq.update(open_set_handle[n_id], new_node); 
            // open_pq.push(new_node);  Wrong!!!!
          }
        }
      } 
    }    
    std::vector<std::vector<double>> res;
    res = calc_final_path(ngoal, minx , miny, reso);
    delete ngoal;
    delete nstart;
    return res; 
};

/**
 * @brief: calculate the distance policy (heuristic starting from the goal node)
 * @param: gx: goal x position
 * @param: gy: goal y position
 * @param: ox: the obstacle positions (x coordinates)
 * @param: oy: the obstacle positions (y coordinates)
 * @param: reso: map xy resolution
 * @param: vr: the radius of the vehicle circle 
 * @return: the policy map based on the current obstacle configuration
 */ 
vector<vector<double>> calc_dist_policy(double gx, double gy, 
                      std::vector<double> ox_, std::vector<double> oy_,
                      double reso, double rr) {

    // 1. construct the obstacle map
    int xwidth, ywidth;
    double minx, miny, maxx, maxy;

    std::vector<std::vector<int> > obmap = calc_obstacle_map(
                                        ox_, oy_, xwidth, ywidth,
                                        minx, miny, maxx, maxy, 
                                        reso, rr);
    // 2. construct the goal node                   
    Node2d*  ngoal = new Node2d(calc_xyindex(gx, minx, reso), calc_xyindex(gy, miny, reso), 0.0, 0.0, 0.0);
    // 3. initialize the motion model
    std::vector<Node2d> motion = get_motion_model();
    // 4. initialize the open set and the closed set 
    auto cmp = [](const Node2d* left, const Node2d* right) {
      return left->sum_cost > right->sum_cost;
    }; 

    struct CompareNodes {
      bool operator()(const Node2d* lhs, const Node2d* rhs) const{
        return lhs->sum_cost > rhs->sum_cost;
      };
    };

    typedef boost::heap::binomial_heap<Node2d*, 
            boost::heap::compare<CompareNodes>> priorityQueue;
    typedef boost::heap::binomial_heap<Node2d*, 
            boost::heap::compare<CompareNodes>>::handle_type handleType;

    priorityQueue open_pq;
    std::unordered_map<int, Node2d*> open_set;
    std::unordered_map<int, Node2d*> closed_set;
    std::unordered_map<int, handleType> open_set_handle;
    handleType handle = open_pq.push(ngoal);

    open_set[calc_grid_index(*ngoal, xwidth, ywidth)] = ngoal;
    open_set_handle[calc_grid_index(*ngoal, xwidth, ywidth)] = handle;

    while (!open_pq.empty()) {
      // 1. extract the top node 
      Node2d* current = open_pq.top();
      int c_id = calc_grid_index(*current, xwidth, ywidth);
      std::cout << "c_id: " << c_id << std::endl;

      // in this case, we want to calculate the heuristic cost 
      // we do not need to check if some goal node is reached 

      // 2. remove the item from the open set 
      open_set.erase(c_id);
      open_pq.pop();

      closed_set[c_id] = current;

      // 4. expand the motions in eight directions
      for (int i = 0; i < motion.size(); ++i) {
        Node2d* new_node = new Node2d(
          current->x + motion[i].x, 
          current->y + motion[i].y, 
          0.0, 
          current->path_cost + motion[i].path_cost, 0.0, current); 

          int n_id = calc_grid_index(*new_node, xwidth, ywidth);

          // 1. if the current grid contains obstacles, skip and continue

          if (!verify_node(new_node, minx, miny, maxx, maxy, reso, obmap)) {
            delete new_node;
            continue;
          }
          // 2. if the current node has been visited, in the closed set
          // skip and continue
          if (closed_set.count(n_id)) {
             delete new_node;
             continue;
          }
          // 3. if the expanded node is not in the open set, put it into the 
          // open set
          if (!open_set.count(n_id)) {
            new_node->h_cost = calc_heuristic(new_node, ngoal);
            new_node->sum_cost = new_node->h_cost + new_node->path_cost;
            open_set[n_id] = new_node;
            handleType handle = open_pq.push(new_node);
            open_set_handle[n_id] = handle;
          } else {
            if (open_set[n_id]->path_cost > new_node->path_cost) {
              new_node->h_cost = calc_heuristic(new_node, ngoal);
              new_node->sum_cost = new_node->h_cost + new_node->path_cost;
              open_set[n_id] = new_node; // replace the new node 
              open_pq.update(open_set_handle[n_id], new_node);
            }
          }
      } 
    }
    // using path cost to 
}
/**
 * @brief: calculate the policy map
 * @param: closed: visited nodes
 * @param: reso: width of x coordinates
 * @param: minx: the minimum index of the map in the x axis 
 * @param: miny: the mininum index of the map in the y axis
 * @param: xwidth: the width along the x axis
 * @param: ywidth: the width along the y axis
 * @return: the heuristic map 
 */ 
std::vector<std::vector<double>> 
          calc_policy_map(std::unordered_map<int, Node2d*>& closed, 
          int xwidth, int ywidth, double reso, double minx, double miny) {
    std::vector<std::vector<double>> pmap(xwidth,
     vector<double>(ywidth, std::numeric_limits<double>::max()));
    for (int i = 0; i < closed.size(); ++i) {
      pmap[closed[i]->x][closed[i]->y] = closed[i]->path_cost; 
    }    
    return pmap; 
}

// https://www.cnblogs.com/zhaoheng/p/4513185.html
int main(int argc, char const *argv[])
{
  std::cout<<"Testing the A star Algorithm!"<<std::endl;
  double sx = 20.0;
  double sy = 50.0;
  double gx = 50.0;
  double gy = 50.0;

  double grid_size = 2.0;
  double robot_size = 1.0;

  vector<double> ox;
  vector<double> oy;

  // add edges
  for(double i=-10; i<60; i++){
    ox.push_back(i);
    oy.push_back(-10.0);
  }
  for(double i=-10; i<60; i++){
    ox.push_back(60.0);
    oy.push_back(i);
  }
  for(double i=-10; i<61; i++){
    ox.push_back(i);
    oy.push_back(60.0);
  }
  for(double i=-10; i<61; i++){
    ox.push_back(-10.0);
    oy.push_back(i);
  }
  for(double i=-10; i<40; i++){
    ox.push_back(20.0);
    oy.push_back(i);
  }
  for(double i=0; i<40; i++){
    ox.push_back(40.0);
    oy.push_back(60.0 - i);
  }
  plt::plot(ox, oy, ".k");
  plt::plot({sx},{sy} ,"og");
  plt::plot({gx}, {gy}, "xb");
  plt::grid("True");
  plt::axis("equal");

  
  std::vector<std::vector<double>>  path = a_star_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size);
  vector<double> rx = path[0];
  vector<double> ry = path[1];
  std::cout << "Planning finished: " << std::endl;
  plt::plot(rx, ry, "-r");
  plt::show();
  return 0;
}

