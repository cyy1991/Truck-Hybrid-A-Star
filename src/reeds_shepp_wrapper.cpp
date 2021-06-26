#include "reeds_shepp_wrapper.h"

#include <iostream>

// define the path sampling callback
// int PathSamplingCb(double q[3], void* user_data) {
//    std::vector<double> q_temp{q[0], q[1], q[2]};
//    // https://zhuanlan.zhihu.com/p/98061960
//    std::cout << "debug1: " << std::endl;
//     std::vector<std::vector<double> >* real_user_data =
//     (std::vector<std::vector<double> >*)user_data;
//    std::cout << "debug2: " << std::endl;
//    std::cout << q_temp[0] << std::endl;
//    std::cout << q_temp[1] << std::endl;
//    std::cout << q_temp[2] << std::endl;
//    // real_user_data->push_back();
//    real_user_data->push_back(q_temp);
//    std::cout << "debug3: " << std::endl;
//    // std::cout << real_user_data->size();
//    return 1;

// }
// int PathTypeCb(int t, void* user_data) {}

ReedsSheppWrapper::ReedsSheppWrapper(double q0[3], double q1[3],
                                     double turnings_radius) {
  thisptr = new ReedsSheppStateSpace(turnings_radius);
  for (size_t i = 0; i < 3; i++) {  // x, y, theta
    _q0[i] = q0[i];
    _q1[i] = q1[i];
    std::cout << "_q0[i]: " << _q0[i] << std::endl;
    std::cout << "_q1[i]: " << _q1[i] << std::endl;
  }
}

ReedsSheppWrapper::~ReedsSheppWrapper() { delete thisptr; }

std::vector<std::vector<double> > ReedsSheppWrapper::path_sample(
    double q0[3], double q1[3], double step_size) {
  std::vector<std::vector<double> > qs;
  std::cout << "sample begins: " << std::endl;
  thisptr->sample(q0, q1, step_size, qs);
  std::cout << "sample ends: " << std::endl;
  return qs;
}

void ReedsSheppWrapper::path_type(double q0[3], double q1[3]) {
  std::vector<ReedsSheppStateSpace::ReedsSheppPathSegmentType> ts;
  std::cout << " path type begins: " << std::endl;
  thisptr->type(q0, q1, ts);
  std::cout << ts.size() << std::endl;
  for (int i = 0; i < ts.size(); i++) {
    std::cout << ts[i] << std::endl;
  }
  std::cout << " path type ends: " << std::endl;
}
