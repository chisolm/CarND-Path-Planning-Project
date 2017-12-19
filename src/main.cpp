#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "road.h"
#include "trajectory.h"
#include "behavior.h"
#include "helper.h"

using namespace std;

// for convenience
using json = nlohmann::json;

int main() {
  uWS::Hub h;


  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  // Start in lane 1
  int lane = 1;

  // Have a reference velocity to target
  // TODO double ref_vel = 0.0;  // start at 0 mpg, eventual 49.5
  double ref_vel = 0;  // start at 0 mpg, eventual 49.5

  // Test code

  Road test_road = Road(50, 25, map_file_);

  test_road.test();

  Road road = Road(50, 25, map_file_);

  Behavior bh = Behavior();

  h.onMessage([&road, &bh, &ref_vel, &lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            int prev_size = previous_path_x.size();

            lane = lane_from_d(car_d);

            // Update my car information in road object.
            road.update_ego(car_s, car_d);

            // Compute the speed of the closest car in our current lane, if it is too
            // close, begin to reduce speed.

            // TODO Why are we picking off the last item of the list, rather than what is passed to us?
            if (prev_size > 0) {
              car_s = end_path_s;
            }

            for(int i = 0; i < sensor_fusion.size(); i++) {
              cout << "id " << sensor_fusion[i][0];
              for(int j = 1; j < 7; j++) {
                cout << " " << sensor_fusion[i][j];
              }
                cout << endl;
            }
            road.process_sensors(sensor_fusion);
            if (path_debug > 0 )
              road.print_road();

            bool too_close = false;

            // find ref_v to use
            for(int i = 0; i < sensor_fusion.size(); i++) {
              // Car is in my lane
              float d = sensor_fusion[i][6];
              if (d < (4 * lane + 4) && d > (4 * lane)) {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx * vx + vy * vy);
                double check_car_s = sensor_fusion[i][5];

                check_car_s += (double) prev_size * .02 * check_speed;

                if (check_car_s > car_s && ((check_car_s - car_s) < 30)) {
                  too_close = true;
                }
              }
            }

            if (too_close) {
              ref_vel -= .224;
            } else if (ref_vel < 10) {
              ref_vel += .336;
            } else if (ref_vel < 49.5) {
              ref_vel += .224;
            }
          	json msgJson;

            // Update my car information in road object.
            road.update_ego(lane_from_d(car_d), ref_vel, car_s, car_x, car_y, car_yaw, previous_path_x, previous_path_y);

            // Call behavior object to generate trajectory based on it's state machine.
            Trajectory ntj = bh.choose_next_state(road);

          	msgJson["next_x"] = ntj.next_x_vals;
          	msgJson["next_y"] = ntj.next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

            // cout << "json " << msgJson.dump() << endl;

          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
