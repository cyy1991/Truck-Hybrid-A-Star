// simple introduction about nanoflann library 
// https://cloud.tencent.com/developer/article/1475792
#include <nanoflann.hpp>
#include <KDTreeVectorOfVectorsAdaptor.h>
#include <iostream>
#include <algorithm>
#include <ctime>
#include <cstdlib>
#include <chrono>
using namespace std;
using namespace nanoflann;  

typedef std::vector<std::vector<double> > vector_of_vectors_t; 
void generateRandomPointCloud(vector_of_vectors_t &samples, const size_t N, const size_t dim, const double max_range = 10.0)
{
	std::cout << "Generating "<< N << " random points...";
	samples.resize(N);
	for (size_t i = 0; i < N; i++)
	{
		samples[i].resize(dim);
		for (size_t d = 0; d < dim; d++)
			samples[i][d] = max_range * (rand() % 1000) / (1000.0);
	}
	std::cout << "done\n";
}

int main(int argc, char* argv[]) {
    std::cout << "testing the nanoflann: " << std::endl;
    vector_of_vectors_t samples;
    double max_range = 20;
    int state_dim = 3;
    int num_samples = 1000;
    // 1. randomize seed 
    srand(static_cast<unsigned int>(time(nullptr)));
    // 2. generate points
    generateRandomPointCloud(samples, num_samples, state_dim, max_range);
    // 3. query point 
    std::vector<double> query_pt(state_dim);
    for (size_t d = 0; d < state_dim; d++) {
        query_pt[d] =max_range * (rand() % 1000) / (1000.0);
    }
    // 4. construct the kd-tree index 
    typedef KDTreeVectorOfVectorsAdaptor<vector_of_vectors_t, double> kd_tree_t;

    kd_tree_t mat_index(state_dim, samples, 10 /*max leaf*/);

    mat_index.index->buildIndex();
    // 5. knnSearch(): Perform a search for the N closest points
    size_t num_results = 5;
    std::vector<size_t> ret_index(num_results);
    std::vector<double> out_dist_sqr(num_results);

    num_results = mat_index.index->knnSearch(&query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

    // In case of less points in the tree than requested:
    ret_index.resize(num_results);
    out_dist_sqr.resize(num_results);


    for (size_t i = 0; i < num_results; i++) {
        cout << "idx["<< i << "]=" << ret_index[i] << " dist["<< i << "]=" << out_dist_sqr[i] << endl;
    }
    cout << "\n"; 
    // 6. perform a search for the points within search_radius 
    double search_radius = static_cast<double>(0.1);
    std::vector<std::pair<size_t, double>> ret_matches;
    nanoflann::SearchParams params;

    const size_t nMatches = mat_index.index->radiusSearch(&query_pt[0], search_radius, ret_matches, params);

    for (size_t i = 0; i < nMatches; i++)
			cout << "idx["<< i << "]=" << ret_matches[i].first << " dist["<< i << "]=" << ret_matches[i].second << endl;
		cout << "\n";

    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
    std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_seconds = end - start;  

    
    return 0;  
}
