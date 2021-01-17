
#ifndef _KD_TREE_COMMON_H_
#define _KD_TREE_COMMON_H_
#include <nanoflann.hpp>
#include <KDTreeVectorOfVectorsAdaptor.h>
#include <iostream>
#include <algorithm>
#include <ctime>
#include <cstdlib>
#include <chrono>
#include <vector>
using namespace std;
using namespace nanoflann;  
typedef std::vector<std::vector<double>> vector_of_vectors_t;
typedef KDTreeVectorOfVectorsAdaptor<vector_of_vectors_t, double> kd_tree_t;

#endif