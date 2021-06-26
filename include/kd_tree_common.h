
#ifndef _KD_TREE_COMMON_H_
#define _KD_TREE_COMMON_H_
#include <KDTreeVectorOfVectorsAdaptor.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <nanoflann.hpp>
#include <vector>
using namespace std;
using namespace nanoflann;
typedef std::vector<std::vector<double>> vector_of_vectors_t;
typedef KDTreeVectorOfVectorsAdaptor<vector_of_vectors_t, double> kd_tree_t;

#endif