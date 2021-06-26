#include "vehicle.h"

int main(int argc, char* argv[]) {
  Vehicle car(3.0, 2.0, 3.3, 1.0, 0.6);
  car.plot_car(0.0, 0.0, 1.0);
  // plot the car
  return 0;
}