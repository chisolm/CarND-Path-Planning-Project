#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include "road.h"

using namespace std;

class Road;
class Vehicle;
class Trajectory {
public:

    vector<double> next_x_vals;
	vector<double> next_y_vals;
	vector<double> next_s_vals;
	vector<double> next_d_vals;
	bool valid;
	int current_lane;
	int desired_lane;

	/**
	* Constructor
	*/
	// Trajectory();
	Trajectory(int lane, Road road);
	Trajectory(int lane, double ref_vel, double car_s, double car_x, double car_y, double car_yaw,
			vector<double> previous_path_x,
			vector<double> previous_path_y,
			Road road);
	Trajectory(Vehicle car, Road road);

	/**
	* Destructor
	*/
	virtual ~Trajectory();
	void print();

};

#endif  // TRAJECTORY_H
