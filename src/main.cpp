#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <list>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;
using std::list;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Gets the average of the last n values
double getLastAverage(list<double> values, unsigned int max_values_qt) {
  if (values.size() > max_values_qt) {
    values.pop_front();
  }
  double sum = 0.0;
  for (list<double>::iterator it = values.begin(); it != values.end(); it++) {
    sum += *it;
  }
  double avg = sum/static_cast<double>(values.size());
  return avg;
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;
  PID pid_pos;
  PID pid_speed;
  /**
   * TODO (DONE): Initialize the pid variable.
   */

  // Twiddle first try (40 mph)
  // pid_pos.InitCoeffs(0.035, 0.000, 0.2, 0.001, 0.0, 0.1);

  // Twiddle second try (50 mph)
  // pid_pos.InitCoeffs(0.123497, 0.0002, 0.061482, 0.0089543, 0.0001, 0.0161819);

  // Twiddle third try (70 mph variable)
  pid_pos.InitCoeffs(0.155743, 0.00206489, 0.0632458,  0.00411405, 4.64091e-05, 0.004977, 0.2);

  // Twiddle speed first try (50 mph)
  //pid_speed.InitCoeffs(0.1, 0.001, 0.01, 0.01, 0.0001, 0.01);

  // Twiddle speed last parameters
  pid_speed.InitCoeffs(0.137334, 0.00137156, 0.0671561, 0.00194872, 0.000177156, 0.0177156, 0.0);
  pid_pos.setLimits(-5.0, 5.0, -1.0, 1.0);
  pid_speed.setLimits(0.0, 100.0, -1.0, 1.0);
  string log_angle_name = "../plots/log_twiddle_final_db.csv";
  string log_speed_name = "../plots/log_twiddle_speed_tunned.csv";

  bool optimize_position = false; // Make true to optimize position loop
  bool optimize_speed = false;    // Make true to optimize speed loop
  bool log_position = false;       // Make true to create a log of the position loop
  bool log_speed = false;         // Make true to create a log of the speed loop
  
  // Removes logs to start new ones on this run of the program
  remove(log_angle_name.c_str());
  remove(log_speed_name.c_str());

  list<double> last_angles;       // List used for filtering the last steering angle values
  h.onMessage([&pid_pos, &pid_speed, &log_angle_name, &log_speed_name, &last_angles, &optimize_position, &optimize_speed, &log_position, &log_speed]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());          

          // Position controller
          pid_pos.setSP(0.0);
          double steer_value = pid_pos.updateController(cte);

          // Speed controller
          last_angles.push_back(fabs(steer_value));
          double angle_avg = getLastAverage(last_angles, 5);
          double speed_setpoint = 70.0;
          double speed_SP_corrected = speed_setpoint*(1.0 - 0.6*std::min(5.0*fabs(angle_avg),1.0));
          // For debugging, the calculated speed setpoint
          std::cout << "Calculated speed setpoint: " << speed_SP_corrected << std::endl; 
          pid_speed.setSP(speed_SP_corrected);
          double throttle = pid_speed.updateController(speed);

          /**
           * TODO (DONE): Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          if (optimize_position) {
              pid_pos.twiddleOpt(cte, 0.0002, 3);
          }
          if (log_position) {
              std::cout << "POSITION CONTROLLER" << std::endl;
              pid_pos.printParameters();
              pid_pos.printToCSV(log_angle_name);
          }
          if (optimize_speed) {
              pid_speed.twiddleOpt(speed, 0.002, 3);
          }
          if (log_speed) {
              std::cout << "SPEED CONTROLLER" << std::endl;
              pid_speed.printParameters();
              pid_speed.printToCSV(log_speed_name);
          }

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;

          if (pid_pos.reset_simulator || pid_speed.reset_simulator) {
            string msg = "42[\"reset\",{}]";
            pid_pos.reset_simulator = false;
            pid_speed.reset_simulator = false;
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
          else {
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }

        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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