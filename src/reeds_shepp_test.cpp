#include <reeds_shepp.h>
#include <reeds_shepp_wrapper.h>
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

void plot_path(double q0[3], double q1[3], ReedsSheppWrapper& reeds_shepp_path) {
    auto qs = reeds_shepp_path.path_sample(q0, q1, step_size);
    std::vector<double> xs;
    std::vector<double> ys;
    for (int i = 0; i < qs.size(); i++) {
        xs.push_back(qs[i][0]);
        ys.push_back(qs[i][1]);
    }
    plt::plot(xs, ys, "b-");
    plt::plot(xs, ys, "r.");
    plot_car(q0);
    plot_car(q1);
    // plt::axis("equal");
}

int main(int argc, char *argv[])
{

    // double start_x = -1.0;
    // double start_y = -4.0;
    // double start_yaw = -20.0 / 57.3;
    // double q0[3] = {start_x, start_y, start_yaw};

    // double end_x = 5.0;
    // double end_y = 5.0; 
    // double end_yaw = 25.0 / 57.3;
    // double q1[3] = {end_x, end_y, end_yaw};
    

    // double turning_radius = 3.0;       
    

    // ReedsSheppStateSpace* rsptr = new ReedsSheppStateSpace(turning_radius);

    // std::cout << rsptr->distance(q0, q1);

    // ReedsSheppWrapper reeds_shepp_path;

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
    std::cout << rows << std::endl;
    std::cout << cols << std::endl;

    // for (int i = 0; i < rows; i++) {
    //     for (int j = 0; j < cols; j++) {
    //         plt::subplot(rows, cols, i + 1);
    //         std::vector<double> q0{qs[]};
    //         plot_path(qs[i][0], qs[i][1]);
    //     }
    // }
    int j = 1;
    for (int i = 0; i < qs.size() - 1; i+=2) {
        plt::subplot(2, 3, j); 
        double q0[3] = {qs[i][0], qs[i][1], qs[i][2]};
        double q1[3] = {qs[i + 1][0], qs[i + 1][1], qs[i + 1][2]};
        ReedsSheppWrapper reeds_shepp_path(q0, q1, rho); 
        auto qs = reeds_shepp_path.path_sample(q0, q1, step_size);


        reeds_shepp_path.path_type(q0, q1);   
        // std::cout << "size: " << ts.size() << std::endl;
        plot_path(q0, q1, reeds_shepp_path);
        j++;
    }
    // plt::subplot(2, 1, 1);
    // plt::plot(xs, ys);
    // plt::subplot(2, 1, 2);
    // plt::plot(ks);
    plt::show();
    return 0;
}
