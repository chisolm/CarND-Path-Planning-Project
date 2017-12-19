#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include "trajectory.h"
#include "helper.h"


// Tracking behavior of cars on road

Trajectory::Trajectory(int lane, Road road) {
	Trajectory(lane, road.ego_ref_vel, road.ego_s, road.ego_x, road.ego_y, road.ego_yaw,
		road.ego_previous_path_x, road.ego_previous_path_y, road);
}

Trajectory::Trajectory(int lane, double ref_vel, double car_s, double car_x, double car_y, double car_yaw,
			vector<double> previous_path_x,
			vector<double> previous_path_y,
			Road road) {

    // lane argument is desired lane in this case

    desired_lane = lane;
    current_lane = road.ego_lane;
    int prev_size = previous_path_x.size();

	// Create a list of widely spaced x,y waypoints, evenly spaced at 30m

	vector<double> ptsx;
	vector<double> ptsy;

    // reference x, y, yaw states
	double ref_x = car_x;
	double ref_y = car_y;
	double ref_yaw = deg2rad(car_yaw);

	if (prev_size < 2) {
		double prev_car_x = car_x - cos(car_yaw);
		double prev_car_y = car_y - sin(car_yaw);

		ptsx.push_back(prev_car_x);
		ptsx.push_back(car_x);
		ptsy.push_back(prev_car_y);
		ptsy.push_back(car_y);

	} else {

		ref_x = previous_path_x[prev_size-1];
		ref_y = previous_path_y[prev_size-1];

		double ref_x_prev = previous_path_x[prev_size-2];
		double ref_y_prev = previous_path_y[prev_size-2];
		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);

		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);
	}


	vector<double> next_wp0 = getXY(car_s + 30, (2+4*lane), road.map_waypoints_s, road.map_waypoints_x, road.map_waypoints_y);
	vector<double> next_wp1 = getXY(car_s + 60, (2+4*lane), road.map_waypoints_s, road.map_waypoints_x, road.map_waypoints_y);
	vector<double> next_wp2 = getXY(car_s + 90, (2+4*lane), road.map_waypoints_s, road.map_waypoints_x, road.map_waypoints_y);

	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);

	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);

    // cout << "ptsx 0 x " << ptsx[0] << " y " << ptsy[0] << endl;
    // cout << "ptsx 1 x " << ptsx[1] << " y " << ptsy[1] << endl;
    // cout << "nwp 0 x " << next_wp0[0] << " y " << next_wp0[1] << " s " << car_s + 30 << " d " << (2+4*lane) << endl;
    // cout << "nwp 1 x " << next_wp1[0] << " y " << next_wp1[1] << " s " << car_s + 60 << " d " << (2+4*lane) << endl;
    // cout << "nwp 2 x " << next_wp2[0] << " y " << next_wp2[1] << " s " << car_s + 90 << " d " << (2+4*lane) << endl;

	//  Translate and rotate into local car coordinates, car is 0, 0, 0
	for (int i = 0; i < ptsx.size(); i++) {
		double shift_x = ptsx[i]-ref_x;
		double shift_y = ptsy[i]-ref_y;

		ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
		ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
	}

    // create a spline
	tk::spline s;

    // set (x,y) points to the spline
	s.set_points(ptsx, ptsy);


	// Computing theta for the getFrenet method.  Unknow if I need to compute
	// at each step or if once is sufficient.
	double theta;
	if (previous_path_x.size() > 1) {
		theta = atan2(previous_path_y[1] - previous_path_y[0], previous_path_x[1] - previous_path_x[0]);
	} else {
		theta = 0;  // First loop with < 2 points, this does not matter.
	}

    // Define the actual (x, y) points we will use for the planner

    // Start with all of the previous path points from the last time
    // This is whatever the simulator has not consumed.
	for (int i = 0; i < previous_path_x.size(); i++) {
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
		vector<double> sd = getFrenet(previous_path_x[i], previous_path_y[i], theta, road.map_waypoints_x, road.map_waypoints_y);
		next_s_vals.push_back(sd[0]);
		next_d_vals.push_back(sd[1]);
	}
	cout << "next_x_vals.size() " << next_x_vals.size() << " min 5 " << min(5, (int) previous_path_x.size()) << " " << previous_path_x.size() << endl;

    // Calculate how to break up spline points so that we travel at desired velocity
	double target_x = 30.0;
	double target_y = s(target_x);  // spline function, lookup y for a given x
	double target_dist = sqrt(target_x * target_x + target_y * target_y);
	double x_add_on = 0;

    // TODO re-write this loop.  This is from the walk through.  It is not
    // intuitive at all.

    // Fill up the rest of our path planner after filling it with previous points
	for (int i = 1; i <= 50 - previous_path_x.size(); i++) {

        // TODO, if adjust ref_vel in loop then, we can accelerate at each point
		double N = target_dist / (.02 * ref_vel / 2.24);
		double x_point = x_add_on + target_x/N;
		double y_point = s(x_point);

		x_add_on = x_point;

		double x_ref = x_point;
		double y_ref = y_point;

              // rotate back to normal after rotating it earlier
		x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
		y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

		x_point += ref_x;
		y_point += ref_y;

		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);
		vector<double> sd = getFrenet(x_point, y_point, theta, road.map_waypoints_x, road.map_waypoints_y);
		next_s_vals.push_back(sd[0]);
		next_d_vals.push_back(sd[1]);
	}

	double lx = next_x_vals[0];
	double ly = next_y_vals[0];
	for (int i = 1; i < next_x_vals.size(); i++) {
        // cout << "xdiff " << next_x_vals[i] - lx << " ydiff " << next_y_vals[i] - ly << endl;
		// lx = next_x_vals[i];
		// ly = next_y_vals[i];
	}
	valid = true;
}

Trajectory::Trajectory(Vehicle car, Road road) {

	double s, d;
	double x, y;

	d = 4 * car.lane + 2;
	for (int i = 0; i < 50; i++) {
		s = car.current_s() + car.velocity() * .02;
		next_s_vals.push_back(s);
		next_d_vals.push_back(d);
		vector<double> xy = getXY(s, d, road.map_waypoints_s, road.map_waypoints_x, road.map_waypoints_y);
		next_x_vals.push_back(xy[0]);
		next_y_vals.push_back(xy[1]);
	}
	valid = true;
}



Trajectory::~Trajectory() {}
