#include "road.h"
#include <math.h>
#include <algorithm>
#include <iostream>
#include <map>
#include <string>
#include <iomanip>
#include <iterator>
#include "helper.h"


Road::Road(double speed_limit, int display_length, string map_file_) {
    this->speed_limit = speed_limit;
    this->display_length = display_length;
    lanes = 3;

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }
}

Road::~Road() {}

void Road::test() {
    this->update_ego(50, 6);
    cars[0].add_sensor_data(10, 2, 50);
    cars[0].add_sensor_data(20, 2, 50);
    cars[0].add_sensor_data(30, 2, 50);
    cars[1].add_sensor_data(10, 6, 40);
    cars[1].add_sensor_data(20, 6, 40);
    cars[0].add_sensor_data(40, 2, 60);
    cout << "Car 0" << endl;
    cars[0].print();
    cout << "Car 1" << endl;
    cars[1].print();
    this->print_road();
}

void Road::print_road() {
    double target_s = ego_s;
    double target_d = ego_d;
    double relative_s;
    double view = 300;  // meters around car
    double offset = view / 2 - target_s;
    int length_display_line = view / display_length;
    string laneid_str[3];
    string velocity_str[3];
    string marker(9, '\0');
    const string blank = "        ";

    cout << this->display_length << " " << this->speed_limit << endl;
    for (auto &car : cars) {
        // cout << "id " << car.first << endl;
        // car.second.print();
    }
    for (int i = display_length; i >= 0; i--) {
        if (i % 10) {
            std::stringstream ss;
            ss << std::setw(8) << std::setfill(' ') << (i * length_display_line) + target_s - view / 2;
            marker = ss.str();
        } else {
            marker = blank;
        }
        for (int j = 0; j < 3; j++) {
            laneid_str[j] = blank;
            velocity_str[j] = blank;
        }
        for (auto &car : cars) {
            // what is a line of output?  300m view/display_length(50)
            // relative to start of display
            int start_line = i * length_display_line;
            int end_line = start_line + length_display_line;
            relative_s = car.second.current_s() - target_s + view / 2;
            // cout << "relative_s " << relative_s << endl;
            // cout << "start stop " << i * length_display_line << " " << (i + 1) * length_display_line << endl;
            if (relative_s >= start_line && relative_s < end_line) {
                // cout << "start_line " << start_line << " " << relative_s << " " << car.second.current_s() << " " << target_s << " " << view / 2 << endl;
                int lane = car.second.lane;
                // cout << "current_d " << car.second.current_d() << endl;
                // cout << "current_d " << (int)(car.second.current_d() - 2) << endl;
                // cout << "lane " << lane << endl;
                if (lane < 0 || lane > 2) {
                    cout << "Invalid lane " << lane << endl;
                    lane = 0;
                }
                std::stringstream ss_l;
                ss_l << std::setw(8) << std::setfill(' ') << car.first;
                laneid_str[lane] = ss_l.str();
                std::stringstream ss_v;
                ss_v << std::setw(8) << std::setfill(' ') << setprecision(3) << car.second.velocity();
                velocity_str[lane] = ss_v.str();
            }
        }
        if (target_s + offset >= i * length_display_line && target_s + offset < (i + 1) * length_display_line) {
            int lane = lane_from_d(target_d);
            laneid_str[lane] = "   me   ";
            velocity_str[lane] = blank;
        }
        cout << marker << "|" << laneid_str[0] << "|" << laneid_str[1] << "|" << laneid_str[2] << "|" << endl;
        cout << blank << "|" << velocity_str[0] << "|" << velocity_str[1] << "|" << velocity_str[2] << "|" << endl;
    }
}


void Road::process_sensors(vector<vector <double>> sensor_fusion) {
    // find ref_v to use
    for (int i = 0; i < sensor_fusion.size(); i++) {
        // Car is in my lane
        int id = sensor_fusion[i][0];
        double x = sensor_fusion[i][1];
        double y = sensor_fusion[i][2];
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double s = sensor_fusion[i][5];
        double d = sensor_fusion[i][6];
        double check_speed = sqrt(vx * vx + vy * vy);

        // using d < 0 as a proxy for both invalid data (-280)
        // and perhaps other side of the road.
        if ( d < 0 ) {
            continue;
        }
        cars[id].add_sensor_data(s, d, check_speed);
    }
}

void Road::update_ego(double s, double d) {
    ego_s = s;
    ego_d = d;
}

void Road::update_ego(int lane, double ref_vel, double car_s, double car_x, double car_y, double car_yaw,
        vector <double> previous_path_x, vector <double> previous_path_y) {
    ego_lane = lane;
    ego_ref_vel = ref_vel;
    ego_s = car_s;
    ego_x = car_x;
    ego_y = car_y;
    ego_yaw = car_yaw;
    ego_previous_path_x = previous_path_x;
    ego_previous_path_y = previous_path_y;
}

map<int, Trajectory> Road::generate_predictions() {
    map<int , Trajectory> predictions;

    map<int, Vehicle>::iterator it = this->cars.begin();
    while (it != this->cars.end()) {
        int v_id = it->first;
        Trajectory preds = it->second.generate_prediction(*this);
        predictions.emplace(v_id, preds);
        it++;
    }
    return predictions;
}
