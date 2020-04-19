#ifndef REEDS_SHEPP_WRAPPER_H
#define REEDS_SHEPP_WRAPPER_H
#include "reeds_shepp.h"
#include <vector>
class ReedsSheppWrapper {
private: 
    ReedsSheppStateSpace* thisptr;
    double _q0[3];
    double _q1[3];
public:
    ReedsSheppWrapper(double q0[3], double q1[3], double turning_radius);
    ~ReedsSheppWrapper();
    /**
     *@brief: return total length of Reeds-Shepp curve from q0 to q1
     * with specified turning radiuss   
     */ 
    double path_length(double q0[3], double q1[3], double turning_radius);
    /**
     *@brief: uniformly sampled from corresponding Reeds Shepp curve 
     */ 
    std::vector< std::vector<double> > path_sample(double q0[3], 
    double q1[3], double step_size);  
    /**
     * @brief: return the tuple of path segment type for the 
     *  Reeds-Shepp curve from q0 to q1 with specified turning 
     *  radius 
     */ 
    void path_type(double q0[3], double q1[3]);  
};
#endif