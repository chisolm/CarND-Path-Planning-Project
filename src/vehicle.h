#ifndef VEHICLE_H
#define VEHICLE_H

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
#include "trajectory.h"

using namespace std;

#define N_measurements 10

class Road;
class Trajectory;
class Vehicle {
public:

	int lane;
	vector<double> s;
	vector<double> d;
	vector<double> speed;
	int measurements;
	bool initialized = false;

	/**
	* Constructor
	*/
	Vehicle();
	Vehicle(int lane, double s, double d, double speed);

	/**
	* Destructor
	*/
	virtual ~Vehicle();
	double velocity();
	double current_s();
	double current_d();
	void print();
	void add_sensor_data(double s, double d, double speed);
	Trajectory generate_prediction(Road road);


};

#endif  // VEHICLE_H
