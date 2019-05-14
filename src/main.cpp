#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // TODO understand this file
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  // TODO what's this?
  // frenet s; then we can see the road as straight line ?
  // at same time it is length of the track; for 50MPH, car will cost almost 5mins to go all the way.

  // transfer data into map_waypoints
  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
  // TODO ifstream?
  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  // TODO variable
  // ego car lane init
  int ego_lane = 1;
  // ego car ref_vel init
  // TODO not suitable variable name
  double ref_vel = 0.0; // MPH
  // speed_diff; plus or minus by this diff while every changing;
  // TDOO the bigger, the larger for jerk?
  // TODO why .224?
  const double speed_diff = .224;
  // max_accel:
  // TODO the limit of the speed?
  const double max_accel = 49.5;

  h.onMessage([&max_accel, &speed_diff, &ref_vel, &ego_lane,
               &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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
          int previous_path_size = previous_path_x.size();
          // Previous path's end s and d values
          // TODO end?target?
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          // TODO how to save and then read this data?

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          // TODO this path will always change?
          // TODO plot the path


          // TODO prediction: predict what actions othe object might take
          // HERE we only care the nearest 3 car ahead (ahead, left ahead, right ahead)
          // type of prediction: trajectory but not all possible trajectories with probablity
          // TODO what???
          if (previous_path_size > 0) {
            car_s = end_path_s;
          }
          // ego car's env state (from other car) init
          bool car_left = false; // is there a car left?
          bool car_right = false; // is there a car right?
          bool car_ahead = false; // is there a car ahead?

          // check all car in sensor_fusion
          for (int i = 0; i < sensor_fusion.size(); i++) {
            // state of the checked car
            double v_x = sensor_fusion[i][3];
            double v_y = sensor_fusion[i][4];
            double sigma_x = sensor_fusion[i][5];
            float sigma_y = sensor_fusion[i][6];

            int check_lane = floor(sigma_y/4); // 0,1,2; 3 lanes with 4m width
            double v = sqrt(v_x * v_x + v_y * v_y);
            // sigma_x in the .02s future
            sigma_x += (double)previous_path_size * v * 0.02;
            // there is a car ahead and distance < 30
            car_ahead |= (check_lane - ego_lane) == 0 && (sigma_x - car_s) > 0 && (sigma_x - car_s) < 30;
            // there is a car left and distance < 30
            car_left |= (check_lane - ego_lane) == -1 && (sigma_x - car_s) > -30 && (sigma_x - car_s) < 30;
            // there is a car right and distance < 30
            car_right |= (check_lane - ego_lane) == 1 && (sigma_x - car_s) > -30 && (sigma_x - car_s) < 30;
          }


          // TODO behavior: stop, changing lane, acceleration,
          // HERE we only care changing lane or reduce speed instead of worry about cost function?
          // "KL" - Keep Lane
          // "LCL" / "LCR"- Lane Change Left / Lane Change Right
          // "PLCL" / "PLCR" - Prepare Lane Change Left / Prepare Lane Change Right
          if(car_ahead) {
            if(!car_left && ego_lane > 0) {
              ego_lane--;
            } else if(!car_right && ego_lane < 2) {
              ego_lane++;
            } else if(!car_left && ego_lane < 2) {
              ego_lane++;
            }else {
              ref_vel -= speed_diff;
            }
          } else if(ref_vel < max_accel){
            ref_vel += speed_diff;
          }

          // TODO trajectory: find the best trajectory
          // TODO create waypoints
          vector<double> ptsx;
          vector<double> ptsy;
          // reference x, y, yaw
          double ref_x;
          double ref_y;
          double ref_yaw;
          double previous_ref_x;
          double previous_ref_y;
          if (previous_path_size >= 2) {
            ref_x = previous_path_x[previous_path_size - 1];
            ref_y = previous_path_y[previous_path_size - 1];
            previous_ref_x = previous_path_x[previous_path_size - 2];
            previous_ref_y = previous_path_y[previous_path_size - 2];
            ref_yaw = atan2(ref_y - previous_ref_y, ref_x - previous_ref_x);
          } else {
            ref_x = car_x;
            ref_y = car_y;
            previous_ref_x = car_x - cos(car_yaw); // TODO WHAT??? use ref_x, ref_y, ref_yaw to compute previous_ref?
            previous_ref_y = car_y - cos(car_yaw);
            ref_yaw = deg2rad(car_yaw);
          }
          ptsx.push_back(previous_ref_x);
          ptsy.push_back(previous_ref_y);
          ptsx.push_back(ref_x);
          ptsy.push_back(ref_y);

          // Setting up target points in the future.
          // TODO 30, 60, 90 ?
          vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*ego_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*ego_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*ego_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // localization
          for (int i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          // TODO create spline from localized map_waypoints
          tk::spline s;
          s.set_points(ptsx, ptsy);


          // TODO previous path for keepping continuous
          for (int i = 0; i < previous_path_size; i++ ) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          // TODO reference points ?Frenet
          // TODO local and global
          // TODO global to local
          // TODO predict by spline and 30m ?
          // future plan by spline
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          double x_add_on = 0; // TODO ?
          for (int i = 1;i < 50 - previous_path_size;i++){
            double N = target_dist/(0.02*ref_vel/2.24);// TODO ? split the goal? 2.24?speed_diff
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            // TODO local to global
            double x_ref = x_point;
            double y_ref = y_point;
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }






          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
