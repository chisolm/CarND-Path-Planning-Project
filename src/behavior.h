#ifndef BEHAVIOR_H
#define BEHAVIOR_H

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
#include "vehicle.h"

using namespace std;

class Road;
class Trajectory;
class Behavior {
public:

	string run_state;
	int lcm_countdown;
	int lcm_desired_lane;

	/**
	* Constructor
	*/
	// Behavior();
	Behavior();

	Trajectory choose_next_state(Road road);
	vector<string> successor_states();

	Trajectory generate_trajectory(string state, map<int , Trajectory> predictions, Road road);
	Trajectory keep_lane_trajectory(string state, map <int, Trajectory> predictions, Road road);
	Trajectory lane_change_trajectory(string state, map <int, Trajectory> predictions, Road road);
	Trajectory lane_change_maneuver_trajectory(string state, map <int, Trajectory> predictions, Road road);

	/**
	* Destructor
	*/
	virtual ~Behavior();
	void print();

};

#endif  // BEHAVIOR_H
