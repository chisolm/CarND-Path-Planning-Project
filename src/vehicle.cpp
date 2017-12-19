#include "vehicle.h"
#include <math.h>
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include <algorithm>
#include "helper.h"


// Tracking behavior of cars on road

Vehicle::Vehicle() {}

Vehicle::Vehicle(int lane, double s, double d, double speed) {
    this->lane = lane;
    this->s[0] = s;
    this->d[0] = d;
    this->speed[0] = speed;
    this->s.resize(N_measurements);
    this->d.resize(N_measurements);
    this->speed.resize(N_measurements);
    measurements++;
    initialized = true;
}

Vehicle::~Vehicle() {}

double Vehicle::current_s() {
    return s[(measurements - 1) % N_measurements];
}
double Vehicle::current_d() {
    return d[(measurements - 1) % N_measurements];
}

double Vehicle::velocity() {
    return speed[(measurements - 1) % N_measurements];
}

void Vehicle::print() {
    for (int i = 0; i < min(measurements, N_measurements); i++) {
        cout << "s " << s[(measurements - 1 - i) % N_measurements] << " d " << d[(measurements - 1 - i) % N_measurements] << endl;
    }
    cout << "velocity " << this->velocity() << endl;
    cout << "lane " << lane << " measurements " << measurements << endl;
    cout << "current_s " << this->current_s() << " current_d " << this->current_d() << endl;
}

void Vehicle::add_sensor_data(double s, double d, double speed) {
    if (initialized == false) {
        measurements = 0;
        this->s.resize(N_measurements);
        this->d.resize(N_measurements);
        this->speed.resize(N_measurements);
        initialized = true;
    }

    this->s[measurements % N_measurements] = s;
    this->d[measurements % N_measurements] = d;
    this->speed[measurements % N_measurements] = speed;
    lane = lane_from_d(d);
    // cout << "add_sensor_data s " << s << " d " << d << " lane " << lane << endl;
    measurements++;
}

Trajectory Vehicle::generate_prediction(Road road) {
    Trajectory prediction = Trajectory(*this, road);
    return prediction;
}

