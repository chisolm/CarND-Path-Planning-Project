#include "behavior.h"
#include <math.h>
#include <algorithm>
#include <iostream>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"
#include "helper.h"

double calculate_cost(Road road, Trajectory trajectory, map<int, Trajectory> predictions);
int debug_count = 200;

Behavior::Behavior() {
    run_state = "KL";
}

Behavior::~Behavior() {}

// vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions) {
Trajectory Behavior::choose_next_state(Road road) {
    if (debug_count != 0)
        debug_count--;
    // inputs current vehicle state, sensor fusion(maybe as "vehicle objects?")

    // Generate predictions
    map<int , Trajectory> predictions;
    predictions = road.generate_predictions();

    vector<Trajectory> all_trajectories;
    map<string, float> costs;
    for (string& iter_state : successor_states()) {
        // do something with `iter_state`
        Trajectory traj = generate_trajectory(iter_state, predictions, road);
        double cost = 0;
        if (path_debug > 0)
            cout << "state eval: " << iter_state << " " << ((traj.valid == true) ? "true" : "false") << endl;

        if ( traj.valid == true ) {
            cost = calculate_cost(road, traj, predictions);
            if (path_debug > 0)
                cout << "successor_states: " << iter_state << " " << "cost " << cost << endl;
            costs.insert({iter_state, cost});
            all_trajectories.push_back(traj);
        }
    }

    string best_next_state;
    float min_cost = 999999999;
    int idx = 0;
    int best_idx = 0;
    for (auto const& x : costs) {
        string s = x.first;
        float c = x.second;
        cout << "costs: " << x.first << " " << x.second << endl;
        if (c < min_cost) {
            min_cost = c;
            best_next_state = s;
            best_idx = idx;
        }
        idx++;
    }
    if (path_debug > 0)
        cout << "best_next_state: " << best_next_state << endl;

    if (best_next_state.compare("LCL") == 0 || best_next_state.compare("LCR") == 0) {
        this->run_state = "LCM";
        this->lcm_countdown = 30;
        this->lcm_desired_lane = all_trajectories[best_idx].desired_lane;
    } else {
        this->run_state = best_next_state;
    }

    return all_trajectories[best_idx];
}


vector<string> Behavior::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<string> states;
    string state = this->run_state;

    if (lcm_countdown > 0 && state.compare("LCM") == 0) {
        lcm_countdown--;
        states.push_back("LCM");
        return states;
    }
    states.push_back("KL");
    if (state.compare("KL") == 0) {
        states.push_back("LCL");
        states.push_back("LCR");
    }
    // If state is "LCL" or "LCR", then just return "KL"
    return states;
}

Trajectory Behavior::keep_lane_trajectory(string state, map <int, Trajectory> predictions, Road road) {
    /*
    Generate a keep lane trajectory.
    */
    Trajectory trajectory = Trajectory(road.ego_lane, road.ego_ref_vel, road.ego_s, road.ego_x, road.ego_y, road.ego_yaw,
                                      road.ego_previous_path_x, road.ego_previous_path_y, road);
    return trajectory;
}

Trajectory Behavior::lane_change_trajectory(string state, map <int, Trajectory> predictions, Road road) {
    int desired_lane = road.ego_lane;
    bool illegal_lane_change = false;;
    if (state.compare("LCL") == 0)
        desired_lane--;
    else
        desired_lane++;
    if ( desired_lane < 0 ) {
        desired_lane = 0;
        illegal_lane_change = true;
    }
    if (desired_lane > (road.lanes - 1)) {
        desired_lane = road.lanes;
        illegal_lane_change = true;
    }

    Trajectory trajectory = Trajectory(desired_lane, road.ego_ref_vel, road.ego_s, road.ego_x, road.ego_y, road.ego_yaw,
                                      road.ego_previous_path_x, road.ego_previous_path_y, road);

    if ( illegal_lane_change ) {
        trajectory.valid = false;
    }
    return trajectory;
}

Trajectory Behavior::lane_change_maneuver_trajectory(string state, map <int, Trajectory> predictions, Road road) {
    Trajectory trajectory = Trajectory(lcm_desired_lane, road.ego_ref_vel, road.ego_s, road.ego_x, road.ego_y, road.ego_yaw,
                                      road.ego_previous_path_x, road.ego_previous_path_y, road);
    return trajectory;
}

Trajectory Behavior::generate_trajectory(string state, map<int , Trajectory> predictions, Road road) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    cout << "generate_trajectory " << state << endl;
    if (state.compare("KL") == 0) {
        return keep_lane_trajectory(state, predictions, road);
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        return lane_change_trajectory(state, predictions, road);
    } else if (state.compare("LCM") == 0) {
        return lane_change_maneuver_trajectory(state, predictions, road);
    /*
    } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        return prep_lane_change_trajectory(state, predictions);
    */
    } else {
        cout << "state error.  generate_trajectory(): " << state << endl;
        exit(-1);
    }
}


double lane_most_free_ahead_cost(Road road, Trajectory trajectory, map<int, Trajectory> predictions) {
    // Loop through the vehicles, finding closest car that is in the target lane and greater s

    // target lane of trajectory
    int target_lane = trajectory.desired_lane;
    int current_lane = trajectory.current_lane;

    // s at start
    double s = road.ego_s;
    double closest_s = 6600;
    int closest_id = -1;
    map<int, Vehicle>::iterator it = road.cars.begin();
    while (it != road.cars.end()) {
        int v_id = it->first;
        // TODO will have problems at track max length
        double diff = it->second.current_s() - s;
        if ( target_lane == it->second.lane && diff > 0 ) {
            if (diff < closest_s) {
                closest_s = diff;
                closest_id = v_id;
            }
        }
        it++;
    }

    // no cost if free 150 meters out
    double cost;
    if (closest_s > 150) {
        cost = 0;
    } else {
        cost = 1 - (closest_s / 150);
    }
    return cost;
}

double available_slot(Road road, Trajectory trajectory, map<int, Trajectory> predictions) {
    // target lane of trajectory
    int target_lane = trajectory.desired_lane;
    int current_lane = trajectory.current_lane;
    if ( target_lane == current_lane ) {
        return 0;
    }
    // s at start
    double s = road.ego_s;

    double closest_front_s = 6600;
    double closest_back_s = -6600;
    int closest_front_id = -1;
    int closest_back_id = -1;
    map<int, Vehicle>::iterator it = road.cars.begin();
    while (it != road.cars.end()) {
        int v_id = it->first;
        // TODO will have problems at track max length
        // TODO will have problems at track max length
        double diff = it->second.current_s() - s;
        // Find the one ahead
        if ( target_lane == it->second.lane && diff > 0 ) {
            if (diff < closest_front_s) {
                closest_front_s = diff;
                closest_front_id = v_id;
                cout << "closest_front_s " << closest_front_s << " id " << closest_front_id << endl;
            }
        }
        // Find the one behind or equal
        if ( target_lane == it->second.lane && diff <= 0 ) {
            if (diff > closest_back_s) {
                closest_back_s = diff;
                closest_back_id = v_id;
                cout << "closest_back_s " << closest_back_s << " id " << closest_back_id << endl;
            }
        }
        it++;
    }

    double cost;
    // too close, no gap
    if (closest_back_s > -30 || closest_front_s < 30) {
        cost = 1;
        return cost;
    }
    double total_gap = closest_front_s + (-1.0) * closest_back_s;
    if (total_gap > 90) {
        cost = 0;
        return cost;
    }
    cost = 1 - (total_gap / 90);

    // TODO should eval all s values of trajectory to make sure gap stays there.

    return cost;
}

double lane_change_cost(Road road, Trajectory trajectory, map<int, Trajectory> predictions) {
    int target_lane = trajectory.desired_lane;
    int current_lane = trajectory.current_lane;


    if (target_lane != current_lane)
        return 1;
    return 0;
}

#define LANE_FREE_COST 20.0
#define LANE_AVAIL_COST 50.0
#define LANE_CHANGE_COST 5.0

struct cost_functions {
    string name;
    function<double(Road, Trajectory, map<int, Trajectory>)> cf;
    double weight;
};

// Add additional cost functions here.
vector<cost_functions> const ncf_list = {
    {{"lane free"}, lane_most_free_ahead_cost, LANE_FREE_COST},
    {{"lane avail"}, available_slot, LANE_AVAIL_COST},
    {{"lane change"}, lane_change_cost, LANE_CHANGE_COST}
};

double calculate_cost(Road road, Trajectory trajectory, map<int, Trajectory> predictions) {
    /*
    Sum weighted cost functions to get total cost for trajectory.
    */
    double cost = 0.0;

    for (int i = 0; i < ncf_list.size(); i++) {
        float new_cost = ncf_list[i].weight * ncf_list[i].cf(road, trajectory, predictions);
        cout << ncf_list[i].name + " " << new_cost << endl;
        cost += new_cost;
    }

    if (path_debug > 0)
        cout << "total cost " << cost << endl;
    return cost;
}
