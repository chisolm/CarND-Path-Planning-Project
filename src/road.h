#ifndef ROAD_H
#define ROAD_H

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"

using namespace std;

class Trajectory;
class Vehicle;
class Road {
public:

	int lanes;  // num lanes in the road
	int display_length;
	double speed_limit;
	map<int, Vehicle> cars;
	// ego related values
	double ego_s;
	double ego_d;
	int ego_lane;
	double ego_ref_vel;
	double ego_x;
	double ego_y;
	double ego_yaw;
	vector <double> ego_previous_path_x;
	vector <double> ego_previous_path_y;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

  	Road(double speed_limit, int display_length, string map_file_);

  	virtual ~Road();

  	void test();
  	void print_road();
  	void process_sensors(vector<vector <double>> senssor_fusion);
  	void update_ego(double s, double d);
  	void update_ego(int lane, double ref_vel, double car_s, double car_x, double car_y, double car_yaw,
  		vector <double> previous_path_x, vector <double> previous_path_y);

  	map<int, Trajectory> generate_predictions();


};

#endif  // ROAD_H