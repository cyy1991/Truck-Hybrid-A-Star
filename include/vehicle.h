#include <matplotlibcpp.h>

#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;
namespace plt = matplotlibcpp;

// TODO: add check collision function

// for quick implementation, set all variables as public, todo...

class Vehicle {
 public:
  double WB;         // rear to front wheel
  double W;          // width of car
  double LF;         // distance from rear to vehicle front end
  double LB;         // distance from rear to vehicle back end
  double MAX_STEER;  // [rad] maximum steering angle
  // the following params can be calculated by the above values
  double WBUBBLE_DIST;
  double WBUBBLE_R;
  std::vector<double> VRX;  // vehicle rectangle vertices
  std::vector<double> VRY;

 public:
  /**
   * @ default constructor
   */
  Vehicle() {
    this->WB = 3;
    this->W = 2;
    this->LF = 3.3;
    this->LB = 1.0;
    this->MAX_STEER = 0.6;
    this->WBUBBLE_DIST = (LF - LB) / 2.0;
    this->WBUBBLE_R =
        std::sqrt(std::pow(LF + LB / 2.0, 2) + std::pow(W / 2.0, 2));
    this->VRX.push_back(LF);
    this->VRX.push_back(LF);
    this->VRX.push_back(-LB);
    this->VRX.push_back(-LB);
    this->VRX.push_back(LF);

    this->VRY.push_back(W / 2);
    this->VRY.push_back(-W / 2);
    this->VRY.push_back(-W / 2);
    this->VRY.push_back(W / 2);
    this->VRY.push_back(W / 2);
  }
  Vehicle(double wb, double w, double lf, double lb, double max_steer) {
    this->WB = wb;
    this->W = w;
    this->LF = lf;
    this->LB = lb;
    this->MAX_STEER = max_steer;
    this->WBUBBLE_DIST = (LF - LB) / 2.0;
    this->WBUBBLE_R =
        std::sqrt(std::pow(LF + LB / 2.0, 2) + std::pow(W / 2.0, 2));
    this->VRX.push_back(LF);
    this->VRX.push_back(LF);
    this->VRX.push_back(-LB);
    this->VRX.push_back(-LB);
    this->VRX.push_back(LF);

    this->VRY.push_back(W / 2);
    this->VRY.push_back(-W / 2);
    this->VRY.push_back(-W / 2);
    this->VRY.push_back(W / 2);
    this->VRY.push_back(W / 2);
  }
  void plot_car(double x, double y, double yaw) {
    double c = std::cos(yaw);
    double s = std::sin(yaw);
    std::vector<double> car_outline_x;
    std::vector<double> car_outline_y;
    for (int i = 0; i < 5; i++) {
      double tx = c * VRX[i] - s * VRY[i] + x;
      double ty = s * VRX[i] + c * VRY[i] + y;
      car_outline_x.push_back(tx);
      car_outline_y.push_back(ty);
    }
    plt::plot(car_outline_x, car_outline_y, "-k");
    plt::axis("equal");
    plt::show();
  }
};
