#include "reeds_shepp_path.h"
#include <iostream>
#include <matplotlibcpp.h>
#include <vector>
#include <iostream>


using namespace std;
namespace plt = matplotlibcpp;

#define step_size  0.5
#define rho 5.8

std::vector<double> get_point(std::vector<double>& center, 
                    double radius, double orin) {
        double x = center[0] + radius * std::cos(orin);
        double y = center[1] + radius * std::sin(orin);
        std::vector<double> res{x, y};
        return res;
}

void plot_car(double q[3]) {
    //截取前面两个数字, https://blog.csdn.net/l1216766050/article/details/85098382
    std::vector<double> q_xy{q[0], q[1]};
    std::vector<double> a = get_point(q_xy, 
                                        step_size, q[2]);
    std::vector<double> b = get_point(q_xy, 
                        step_size / 2, q[2] + 150.0 / 180.0 * M_PI);

    std::vector<double> c = get_point(q_xy, 
                        step_size / 2, q[2] - 150.0 / 180.0 * M_PI);
    std::cout << a[0] << " " << b[0] << " " << c[0] << std::endl;
    std::cout << a[1] << " " << b[1] << " " << c[1] << std::endl;
    
    std::vector<double> trix{a[0], b[0], c[0], a[0]};
    std::vector<double> triy{a[1], b[1], c[1], a[1]};
    plt::plot(trix, triy);
}
//   
void plot_path(std::vector<double> xs, std::vector<double> ys, double q0[3], double q1[3]) {
    plt::plot(xs, ys, "b-");
    plt::plot(xs, ys, "r.");
    plot_car(q0);
    plot_car(q1);
    plt::axis("equal");
}
int main(int argc, char *argv[])
{
    std::vector<std::vector<double>> qs{
        {0.0, 0.0, 0.0}, {0.0, 0.0, M_PI},
        {0.0, 0.0, M_PI_4}, {3.0, 4.0, 0.0},
        {4.0, 4.0, M_PI_4}, {0.0, 4.0, 0.0},
        {4.0, 0.0, 0.0}, {-3.0, 4.0, M_PI},
        {-4.0, 0.0, 0.0}, {3.0, 4.0, M_PI / 3},
        {4.0, 4.0, 0.0}, {3.0, 4.0, M_PI_2}   
    };

    int rows = qs.size();
    int cols = qs[0].size();
    // std::cout << rows << std::endl;
    // std::cout << cols << std::endl;

    int j = 1;
    for (int i = 0; i < qs.size() - 1; i+=2) {
        plt::subplot(2, 3, j); 
        double q0[3] = {qs[i][0], qs[i][1], qs[i][2]};
        double q1[3] = {qs[i + 1][0], qs[i + 1][1], qs[i + 1][2]};
        ReedShepp rs_path(1/rho); 
        ReedSheppPath* apaths = new ReedSheppPath; 
        rs_path.ShortestRSP(q0, q1, step_size, apaths);
        // std::cout << a << std::endl; 
        plot_path(apaths->x,apaths->y, q0, q1);
        std::cout << apaths->segs_types.size() << std::endl;
        // std::cout << apaths->segs_types[0] << std::endl; 
        j++;
    }
    char ch = 'c';
    std::cout << ch << std::endl;
    // double a[3] = { 1, 1, 1 };
    // double b[3] = {0, 0, 0};
    // ReedShepp rs_path(1/rho); 
    // ReedSheppPath* apaths = new ReedSheppPath; 
    // rs_path.ShortestRSP(a, b, step_size, apaths);
    // plt::subplot(2, 1, 1);
    // plt::plot(xs, ys);
    // plt::subplot(2, 1, 2);
    // plt::plot(ks);
    plt::show();
    return 0;
}