/*************************************************************************
	> File Name: trailerlib.cpp
	> Author: Yongyu Chen 
	> Mail: yongyu.chen@tum.de
	> Created Time: 2018
 ************************************************************************/
#include <iostream>
#include <algorithm>
#include <vector>
#include <matplotlibcpp.h>
// #include "kdtree.h"


using namespace std;
namespace plt = matplotlibcpp;

typedef std::vector<std::vector<double>> vector_of_vectors_t;
// typedef KDTreeVectorOfVectorsAdaptor<vector_of_vectors_t, double> kd_tree_t;
 
// For Collision Checking  
struct Point {
	double x;
	double y;
	Point() {}
 	Point(double _x, double _y) {
		x = _x;
		y = _y;
	}
	Point(const Point& _point) {
		x = _point.x;
		y = _point.y;
	}
	Point& operator= (const Point& _point) {
		this->x = _point.x;
		this->y = _point.y;
		return *this; 
	}

};
/**
 * @brief: cross product of |P1P2| X |P1P|
 */  
double CrossProduct(const Point& p1, const Point& p2, const Point& p) {
	return (p2.x - p1.x) * (p2.y - p1.y) * (p.x - p1.x) * (p.y - p1.y);
}

bool IsPointInRec(const Point& p, const Point& a, const Point& b, 
				  const Point& c, const Point& d) {
	return CrossProduct(a, d, p) * CrossProduct(c, b, p) >= 0 && 
		   CrossProduct(d, c, p) * CrossProduct(b, a, p) >= 0;				  
}

// for quick implementation, set all variables as public, todo... 

class TruckTrailer {
 public:
    double WB; //[m] wheel base: rear to front steer
	double LT; // rear to trailer wheel 
    double W; // width of vehicle (head part)    
    double LF; // distance from rear to `vehicle front end
    double LB; // distance from rear to vehicle back end
	double LTF; //  [m] distance from rear steer to vehicle front end of trailer
	double LTB; //  [m] distance from rear steer to vehicle back end of trailer
    double MAX_STEER; // [rad] maximum steering angle 
	double TR; // Tyre radius [m] for plot
	double TW;//  Tyre width [m] for plot

    // bubble collision check parameter for the trailer 
	double DT; 
	double DTR;
	std::vector<double> vrxt; 
	std::vector<double> vryt;  
	// bubble collision check parameter for the truck part 
	double DF;
	double DFR;
	std::vector<double> vrxf;
	std::vector<double> vryf;
 
 public: 
    /**
     * @ default constructor
     */ 
     TruckTrailer() {
        this->WB = 3.7;
		this->LT = 8.0;   
        this->W = 2.6;
        this->LF = 4.5;
        this->LB = 1.0;
		this->LTF = 1.0;
		this->LTB = 9.0;     
        this->MAX_STEER = 0.6;
		this->TR = 0.5;
		this->TW = 1.0;  
		// the following parameters should be calculated from the above parameters
		this->DT = (LTF + LTB) / 2.0 - LTB;
		this->DTR = std::sqrt(std::pow(LTF + LTB / 2.0, 2) + std::pow(W / 2.0, 2));
		this->vrxt.push_back(LTF);
		this->vrxt.push_back(LTF);
		this->vrxt.push_back(-LTB);
		this->vrxt.push_back(-LTB);
		this->vrxt.push_back(LTF);   
		this->vryt.push_back(-W / 2.0);
		this->vryt.push_back(W / 2.0);
		this->vryt.push_back(W / 2.0);
		this->vryt.push_back(-W / 2.0);
		this->vryt.push_back(-W / 2.0);  
		// for collision checking of front part
		// bubble parameter 
		this->DF = (LF  + LB) / 2.0 - LB;
		this->DFR = std::sqrt(std::pow((LF + LB) / 2.0,  2)+std::pow(W / 2.0, 2));   
		this->vrxf.push_back(LF);
		this->vrxf.push_back(LF); 
		this->vrxf.push_back(-LB);
		this->vrxf.push_back(-LB);
		this->vrxf.push_back(LF);     
		this->vryf.push_back(-W / 2.0);
		this->vryf.push_back(W / 2.0); 
		this->vryf.push_back(W / 2.0);
		this->vryf.push_back(-W / 2.0);
		this->vryf.push_back(-W / 2.0);   

     }
     TruckTrailer(double wb, double lt, double w, double lf, double lb,
	 			  double ltf, double ltb, double max_steer, double tr, 
				   double tw) {
         this->WB = wb;
         this->LT = lt;
         this->W= w; 
         this->LF = lf;
         this->LB = lb;
         this->LTF = ltf;
		 this->LTB = ltb; 
		 this->MAX_STEER = max_steer; 
		 this->TR = tr; 
		 this->TW = tw;     
        
		// the following parameters should be calculated from the above parameters
		this->DT = (LTF + LTB) / 2.0 - LTB;
		this->DTR = std::sqrt(std::pow(LTF + LTB / 2.0, 2) + std::pow(W / 2.0, 2));
		this->vrxt.push_back(LTF);
		this->vrxt.push_back(LTF);
		this->vrxt.push_back(-LTB);
		this->vrxt.push_back(-LTB);
		this->vrxt.push_back(LTF);   
		this->vryt.push_back(-W / 2.0);
		this->vryt.push_back(W / 2.0);
		this->vryt.push_back(W / 2.0);
		this->vryt.push_back(-W / 2.0);
		this->vryt.push_back(-W / 2.0);  
		// construct the vertex points for trailer part 
		Point ta(LTF, -W / 2.0), tb(LTF, W / 2.0), tc(-LTB, W / 2.0), td(LTB, W / 2.0);
		
		// for collision checking of front part
		// bubble parameter 
		this->DF = (LF  + LB) / 2.0 - LB;
		this->DFR = std::sqrt(std::pow((LF + LB) / 2.0,  2)+std::pow(W / 2.0, 2));   
		this->vrxf.push_back(LF);
		this->vrxf.push_back(LF); 
		this->vrxf.push_back(-LB);
		this->vrxf.push_back(-LB);
		this->vrxf.push_back(LF);     
		this->vryf.push_back(-W / 2.0);
		this->vryf.push_back(W / 2.0); 
		this->vryf.push_back(W / 2.0);
		this->vryf.push_back(-W / 2.0);
		this->vryf.push_back(-W / 2.0);
		// construct the vertex points for front truck part
		Point fa(LF, -W / 2.0);
     
     }
     void plot_trailer(double x, double y, double yaw, double yaw1, double steer) {
		 // for the truck yaw angle
         double c = std::cos(yaw);
         double s = std::sin(yaw);
		 // for the trailer yaw angle 
		 double c1 = std::cos(yaw1);
		 double s1 = std::sin(yaw1); 
		 // for the steer 
		 double csteer = std::cos(steer);
		 double ssteer = std::sin(steer);
		 // total length of truck main part 
		 double LENGTH = LB + LF;
		 double LENGTHt = LTB + LTF;  

		// total length of trailer main part 
		// The following serves as reference for writing codes for truck trailer  
		/* ref 
        // std::vector<double> car_outline_x;
        // std::vector<double> car_outline_y;
        //  for (int i = 0; i < 5; i++) {
        //      double tx = c * VRX[i] - s * VRY[i] + x; 
        //      double ty = s * VRX[i] + c * VRY[i] + y;
        //      car_outline_x.push_back(tx);
        //      car_outline_y.push_back(ty);
        //  }
        //  plt::plot(car_outline_x, car_outline_y, "-k");
        //      plt::axis("equal");
        //      plt::show();
		*/
		 // 1.1 initialize the outline vectors 
		 std::vector<double> truckOutlineX;
		 std::vector<double> truckOutlineY; 
		 truckOutlineX.push_back(-LB);
		 truckOutlineX.push_back(LENGTH - LB);
		 truckOutlineX.push_back(LENGTH - LB);
		 truckOutlineX.push_back(-LB);
		 truckOutlineX.push_back(-LB);
		 truckOutlineY.push_back(W / 2.0);
		 truckOutlineY.push_back(W / 2.0);
		 truckOutlineY.push_back(-W / 2.0);
		 truckOutlineY.push_back(-W / 2.0);
		 truckOutlineY.push_back(W / 2.0);

	    // 1.2 for plotting the truck outline
		std::vector<double> truck_outline_x;  
		std::vector<double> truck_outline_y;

		for (int i = 0; i < 5; i++) {
			double tx = c * truckOutlineX[i] - s * truckOutlineY[i] + x;
			double ty = s * truckOutlineX[i] + c * truckOutlineY[i] + y;
			truck_outline_x.push_back(tx);
			truck_outline_y.push_back(ty); 
		}
		// 1.3 call the plotting function 
		plt::plot(truck_outline_x, truck_outline_y, "-k");
		// 2.1. intialize the outline for plotting the trailer outline 
		std::vector<double> trailerOutlineX;
		std::vector<double> trailerOutlineY;
		trailerOutlineX.push_back(-LTB);
		trailerOutlineX.push_back(LENGTHt - LTB);
		trailerOutlineX.push_back(LENGTHt - LTB);
		trailerOutlineX.push_back(-LTB);
		trailerOutlineX.push_back(-LTB);
		trailerOutlineY.push_back(W / 2.0);
		trailerOutlineY.push_back(W / 2.0);
		trailerOutlineY.push_back(-W / 2.0);
		trailerOutlineY.push_back(-W / 2.0);
		trailerOutlineY.push_back(W / 2.0);
		std::cout << trailerOutlineX.size() << std::endl;
		std::cout << trailerOutlineY.size() << std::endl; 
		// 2.2 for plotting trailer 
		std::vector<double> trailer_outline_x;
		std::vector<double> trailer_outline_y;

		for (int i = 0; i < 5; i++) {
			double tx = c1 * trailerOutlineX[i] - s1 * trailerOutlineY[i] + x;
			double ty = s1 * trailerOutlineX[i] + c1 * trailerOutlineY[i] + y;
			trailer_outline_x.push_back(tx);
			trailer_outline_y.push_back(ty);
		}
		std::cout << trailer_outline_x.size() << std::endl;
		std::cout << trailer_outline_y.size() << std::endl;
		plt::plot(trailer_outline_x, trailer_outline_y, "-k");


		// 3. for plotting the wheels
		// rr wheel 
		std::vector<double> rrWheelX;
		std::vector<double> rrWheelY;  
		// rl wheel 
		std::vector<double> rlWheelX;
		std::vector<double> rlWheelY;
		// fr wheel
		std::vector<double> frWheelX;
		std::vector<double> frWheelY;   
		// fl wheel
		std::vector<double> flWheelX; 
		std::vector<double> flWheelY; 
		// tr wheel
		std::vector<double> trWheelX;
		std::vector<double> trWheelY;   
		// tl wheel
		std::vector<double> tlWheelX;
		std::vector<double> tlWheelY;    
        // rr wheel vector
		rrWheelX.push_back(TR);
		rrWheelX.push_back(-TR);
		rrWheelX.push_back(-TR);
		rrWheelX.push_back(TR);
		rrWheelX.push_back(TR);

		rrWheelY.push_back(-W / 12.0 + TW);
		rrWheelY.push_back(-W / 12.0 + TW);
		rrWheelY.push_back(W / 12.0 + TW);  
		rrWheelY.push_back(W / 12.0 + TW);
		rrWheelY.push_back(-W / 12.0 + TW);
		// rl wheel vector 
		rlWheelX.push_back(TR);
		rlWheelX.push_back(-TR);
		rlWheelX.push_back(-TR);
		rlWheelX.push_back(TR);
		rlWheelX.push_back(TR);

		rlWheelY.push_back(-W / 12.0 - TW);
		rlWheelY.push_back(-W / 12.0 - TW);
		rlWheelY.push_back(W / 12.0 - TW);  
		rlWheelY.push_back(W / 12.0 - TW);
		rlWheelY.push_back(-W / 12.0 - TW);
		// fr wheel
		frWheelX.push_back(TR);
		frWheelX.push_back(-TR);
		frWheelX.push_back(-TR);
		frWheelX.push_back(TR);
		frWheelX.push_back(TR);

		frWheelY.push_back(-W / 12.0 + TW);
		frWheelY.push_back(-W / 12.0 + TW);
		frWheelY.push_back(W / 12.0 + TW);  
		frWheelY.push_back(W / 12.0 + TW);
		frWheelY.push_back(-W / 12.0 + TW);

		// fl wheel
		flWheelX.push_back(TR);
		flWheelX.push_back(-TR);
		flWheelX.push_back(-TR);
		flWheelX.push_back(TR);
		flWheelX.push_back(TR);

		flWheelY.push_back(-W / 12.0 - TW);
		flWheelY.push_back(-W / 12.0 - TW);
		flWheelY.push_back(W / 12.0 - TW);  
		flWheelY.push_back(W / 12.0 - TW);
		flWheelY.push_back(-W / 12.0 - TW);

		// tr wheel
		trWheelX.push_back(TR);
		trWheelX.push_back(-TR);
		trWheelX.push_back(-TR);
		trWheelX.push_back(TR);
		trWheelX.push_back(TR);

		trWheelY.push_back(-W / 12.0 + TW);
		trWheelY.push_back(-W / 12.0 + TW);
		trWheelY.push_back(W / 12.0 + TW);  
		trWheelY.push_back(W / 12.0 + TW);
		trWheelY.push_back(-W / 12.0 + TW);

		// tl wheel
		tlWheelX.push_back(TR);
		tlWheelX.push_back(-TR);
		tlWheelX.push_back(-TR);
		tlWheelX.push_back(TR);
		tlWheelX.push_back(TR);

		tlWheelY.push_back(-W / 12.0 - TW);
		tlWheelY.push_back(-W / 12.0 - TW);
		tlWheelY.push_back(W / 12.0 - TW);  
		tlWheelY.push_back(W / 12.0 - TW);
		tlWheelY.push_back(-W / 12.0 - TW);
		// plotting the fr wheel 
		std::vector<double> fr_wheel_x, fr_wheel_y, fr_wheel_x_tmp, fr_wheel_y_tmp;
		for (int i = 0; i < 5; i++) {
			double tx = csteer * frWheelX[i] - ssteer * frWheelY[i] + WB;
			double ty = ssteer * frWheelX[i] + csteer * frWheelY[i]; 
			fr_wheel_x_tmp.push_back(tx);
			fr_wheel_y_tmp.push_back(ty);
		}
		for (int i = 0; i < 5; i++) {
			double tx = c * fr_wheel_x_tmp[i] - s * fr_wheel_y_tmp[i] + x;
			double ty = s * fr_wheel_x_tmp[i] + c * fr_wheel_y_tmp[i] + y;
			fr_wheel_x.push_back(tx);
			fr_wheel_y.push_back(ty);
		}
        plt::plot(fr_wheel_x, fr_wheel_y, "-k");

		std::vector<double> fl_wheel_x, fl_wheel_y, fl_wheel_x_tmp, fl_wheel_y_tmp;
		for (int i = 0; i < 5; i++) {
			double tx = csteer * flWheelX[i] - ssteer * flWheelY[i] + WB;
			double ty = ssteer * flWheelX[i] + csteer * flWheelY[i]; 
			fl_wheel_x_tmp.push_back(tx);
			fl_wheel_y_tmp.push_back(ty);
		}
		for (int i = 0; i < 5; i++) {
			double tx = c * fl_wheel_x_tmp[i] - s * fl_wheel_y_tmp[i] + x;
			double ty = s * fl_wheel_x_tmp[i] + c * fl_wheel_y_tmp[i] + y;
			fl_wheel_x.push_back(tx);
			fl_wheel_y.push_back(ty);
		}

        plt::plot(fl_wheel_x, fl_wheel_y, "-k");
        

		std::vector<double> tr_wheel_x, tr_wheel_y;
		for (int i = 0; i < 5; i++) {
			double tx = c1 * (trWheelX[i]-LT) - s1 * trWheelY[i] + x;
			double ty = s1 * (trWheelX[i]-LT) + c1 * trWheelY[i] + y;
			tr_wheel_x.push_back(tx);
			tr_wheel_y.push_back(ty);
		}
        plt::plot(tr_wheel_x, tr_wheel_y, "-k");

		std::vector<double> tl_wheel_x, tl_wheel_y;
		for (int i = 0; i < 5; i++) {
			double tx = c1 * (tlWheelX[i] - LT) - s1 * tlWheelY[i] + x;
			double ty = s1 * (tlWheelX[i] - LT) + c1 * tlWheelY[i] + y; 
			tl_wheel_x.push_back(tx);
			tl_wheel_y.push_back(ty);
		}
        plt::plot(tl_wheel_x, tl_wheel_y, "-k");

		std::vector<double> rr_wheel_x, rr_wheel_y;
		for (int i = 0; i < 5; i++) {
			double tx = c * rrWheelX[i] - s * rrWheelY[i] + x;
			double ty = s * rrWheelX[i] + c * rrWheelY[i] + y;
			rr_wheel_x.push_back(tx);
			rr_wheel_y.push_back(ty);
		}
        plt::plot(rr_wheel_x, rr_wheel_y, "-k");

		std::vector<double> rl_wheel_x, rl_wheel_y;
		for (int i = 0; i < 5; i++) {
			double tx = c * rlWheelX[i] - s * rlWheelY[i] + x;
			double ty = s * rlWheelX[i] + c * rlWheelY[i] + y; 
			rl_wheel_x.push_back(tx);
			rl_wheel_y.push_back(ty);
		}
        plt::plot(rl_wheel_x, rl_wheel_y, "-k");

        plt::axis("equal");
		plt::show();

     }
	 /**
      * @brief: calc trailer yaw list from x, y, yaw lists
      * @return: trailer yaw list
      */ 
	 vector<double> calc_trailer_yaw_from_xyyaw(std::vector<double> x,
									std::vector<double> y, 
									std::vector<double> yaw, 
									double init_tyaw, 
									std::vector<double> steps) {
		vector<double> tyaw(x.size(), 0.0);
		tyaw[0] = init_tyaw;
		for (size_t i = 1; i < x.size(); ++i) {
			tyaw[i] += tyaw[i - 1] + steps[i - 1] / LT * std:: sin(yaw[i - 1] - tyaw[i - 1]);
		}
		return tyaw;  								
	}
	/**
	 * @brief: use motion model to predict the next state
	 * @ref: http://planning.cs.uiuc.edu/node661.html#77556
	 */ 
	// std::vector<double> trailer_motion_model(double x, double y, 
	// 						double yaw0, double yaw1, double delta) {

	// 	return {x + D * std::cos(yaw0), y + D * std::sin(yaw0), 
	// 			yaw0 + D / L * std::tan(delta), yaw1 + D / d * std::sin(yaw0 - yaw1)};						
	// }
    // TDOO: write test for this function
    bool rect_check(double ix, double iy, double iyaw, std::vector<double>& ox, 
				    std::vector<double>& oy, std::vector<double>& vrx, 
					std::vector<double>& vry) {
		double c = cos(-iyaw), s = sin(-iyaw);
		// iterate the obstacles and convert them to vehicle frame 
		for (int i = 0; i < ox.size(); ++i) {
			double tx = ox[i] - ix;
			double ty = oy[i] - iy;
			Point ob(c * tx - s * ty, s * tx + c * ty);
		}
		return true;  

	}

};
