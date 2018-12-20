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

using namespace std;

// for convenience
using json = nlohmann::json;

#define MIN_V           (0.2)  // minimum speed in mph
#define MAX_V           (49.5)  // maximum speed in mph
#define DELTA_T         (0.02)  // 20 ms
#define LANE_WIDTH      (4)     // 4 meters per lane
#define NUM_LANES       (3)
#define MID_LANE        (1)
#define BYPASS_DIST     (15)    // The distance between the cars to bypass
#define TRAILING_DIST   (30)


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
// MPH to Meter per second conversion
double mph2mps(double x) { return (x / 2.24); }
double mps2mph(double x) { return (x * 2.24); }
int    d2lane(double d) { return int(d / LANE_WIDTH); }
double lane2d(int lane) { return LANE_WIDTH / 2.0 + lane * LANE_WIDTH; }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while((prev_wp < (int)(maps_s.size() - 1)) && (s > maps_s[prev_wp+1]))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

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
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

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

  int     cur_lane = 1;
  double  ref_v = MIN_V;
  int     stay_on_lane = 0;

#ifdef UWS_VCPKG
  h.onMessage([&stay_on_lane, &ref_v, &cur_lane, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length,
    uWS::OpCode opCode) {
#else
  h.onMessage([&stay_on_lane, &ref_v, &cur_lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
#endif
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

          json msgJson;

          int prev_size = previous_path_x.size();

          // adjust the car's s to the previous path's end s, because the
          // collision is calculated based on prev_size steps
          if (prev_size > 0) {
            car_s = end_path_s;
          }

          /* Go through every car, and find out the distance from cars ahead it and 
           * behind it on each lane 
           */
          double car_ahead_s_delta[NUM_LANES] = { 1000, 1000, 1000 };
          double car_behind_s_delta[NUM_LANES] = { 1000, 1000, 1000 };
          double car_ahead_v[NUM_LANES] = { MAX_V, MAX_V, MAX_V };
          double car_behind_v[NUM_LANES] = { MIN_V, MIN_V, MIN_V };

          for (int i = 0; i < sensor_fusion.size(); i++) {
            // get the car's lane
            double d = sensor_fusion[i][6];
            int    lane = d2lane(d);

            double vx = sensor_fusion[i][3]; // velocity on x
            double vy = sensor_fusion[i][4]; // velocity on y
            double v = sqrt(vx * vx + vy * vy); // total velocity
            double s = sensor_fusion[i][5]; // s in Frenet

            // predict where this car will be after prev_size steps
            s += v * DELTA_T * prev_size;

            // Detect the car right in front of this car
            if ((s > car_s) && ((s - car_s) < car_ahead_s_delta[lane])) {
              car_ahead_s_delta[lane] = s - car_s;
              car_ahead_v[lane] = v;
            }

            // Detect the car right after this car
            if ((s < car_s) && ((car_s - s) < car_behind_s_delta[lane])) {
              car_behind_s_delta[lane] = car_s - s;
              car_behind_v[lane] = v;
            }
          }

          /* Find out the target lane to change, target lane maybe two lanes away */
          int target_lane = cur_lane;
          /* Start change lane decision when the car in front is 30x3 meters away */
          if (car_ahead_s_delta[cur_lane] < (TRAILING_DIST * 3)) {
            int max_v = car_ahead_v[cur_lane];
            //int start_lane = rand() % NUM_LANES;
            int start_lane = 0;
            for (int i = 0; i < NUM_LANES; i++) {
              int check_lane = (start_lane + i) % NUM_LANES;
              /* If the car's velocity is the maximum or the car is pretty far  
               * way ahead(30x2 meters) comparing with the car ahead on the current lane,
               * we will set it as the target lane
               */
              if ( (car_ahead_s_delta[check_lane] > (car_ahead_s_delta[cur_lane] + TRAILING_DIST * 2)) ||
                   (car_ahead_v[check_lane] > max_v) )
              {
                max_v = car_ahead_v[check_lane];
                target_lane = check_lane;
              }
            }
          }
  
          if (target_lane > cur_lane) {
            target_lane = cur_lane + 1;
          }
          else if (target_lane < cur_lane) {
            target_lane = cur_lane - 1;
          }

          double px;
          double py;
          double psi;
          double px_prev;
          double py_prev;
          vector<double> ptsx;
          vector<double> ptsy;


          if (prev_size < 2)
          {
            px = car_x;
            py = car_y;
            psi = deg2rad(car_yaw);

            px_prev = px - cos(psi);
            py_prev = py - sin(psi);
          }
          else {
            px = previous_path_x[prev_size - 1];
            py = previous_path_y[prev_size - 1];

            px_prev = previous_path_x[prev_size - 2];
            py_prev = previous_path_y[prev_size - 2];
            psi = atan2(py - py_prev, px - px_prev);
          }

          ptsx.push_back(px_prev);
          ptsy.push_back(py_prev);

          ptsx.push_back(px);
          ptsy.push_back(py);

          // generate 3 positions on Frenet coordinate for smooth 
          // fitting of the planned path
          for (int i = 1; i <= 3; i++) {

            double next_s = car_s + 30 * i;
            double next_d = lane2d(cur_lane);

            vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(xy[0]);
            ptsy.push_back(xy[1]);

            /* change lane */
            if (i == 1) {
              stay_on_lane++;

              if ( (stay_on_lane > 100) &&
                   (target_lane != cur_lane) )
              {
                if ((car_ahead_s_delta[target_lane] >= TRAILING_DIST) &&
                    (car_behind_s_delta[target_lane] >= BYPASS_DIST))
                {
                  cur_lane = target_lane;
                  stay_on_lane = 0;
                }
              }
            }
          }

          // Now convert the points from map coordinate to the car's coordinate
          // because it's easier to generate middle points on the car's coordinate
          // with controlled speed
          for (int i = 0; i < ptsx.size(); i++) {
            double x_diff = ptsx[i] - px;
            double y_diff = ptsy[i] - py;

            ptsx[i] = x_diff * cos(-psi) - y_diff * sin(-psi);
            ptsy[i] = y_diff * cos(-psi) + x_diff * sin(-psi);
          }

          tk::spline s;
          s.set_points(ptsx, ptsy);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // use the left over points in the previous path 
          for (int i = 0; i < prev_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);
          double acc = 0.15; // acceleration
          double dec = 0.18; // deceleration

          double diff_x = 0;

          // Now generate the points on the planned path
          for (int i = 0; i < 50 - prev_size; i++)
          {
            double N = target_dist / (mph2mps(ref_v) * DELTA_T);
            
            diff_x += target_x / N;
            double diff_y = s(diff_x);
            // convert the x, y from car's coordinate back to the map's cooridnate
            double next_x = px + diff_x * cos(psi) - diff_y * sin(psi);
            double next_y = py + diff_x * sin(psi) + diff_y * cos(psi);

            next_x_vals.push_back(next_x);
            next_y_vals.push_back(next_y);

            /* update the car_ahead_s_delta for each delta_t */
            car_ahead_s_delta[target_lane] += mph2mps(car_ahead_v[target_lane] - ref_v) * DELTA_T;
            car_ahead_s_delta[cur_lane] += mph2mps(car_ahead_v[cur_lane] - ref_v) * DELTA_T;

            /* match the speed to the target lane */
            if ( ((car_ahead_s_delta[target_lane] > TRAILING_DIST) ||
                   (car_ahead_s_delta[target_lane] < BYPASS_DIST)) &&
                 (car_ahead_s_delta[cur_lane] > TRAILING_DIST) )
            {
              ref_v += acc;
            }
            else {
              if (ref_v - dec > car_behind_v[target_lane]) {
                ref_v -= dec;
              }
              else {
                ref_v = car_behind_v[target_lane];
              }

              /* Has to at least faster than the car behind use */
              if (car_behind_s_delta[cur_lane] < BYPASS_DIST) {
                if (ref_v < car_behind_v[cur_lane]) {
                  ref_v = car_behind_v[cur_lane];
                }
              }
            }

            if (ref_v > MAX_V) {
              ref_v = MAX_V;
            }

            if (ref_v < MIN_V) {
              ref_v = MIN_V;
            }
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
#ifdef UWS_VCPKG
          ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
#ifdef UWS_VCPKG
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
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

#ifdef UWS_VCPKG
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
#else
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
#endif
    std::cout << "Connected!!!" << std::endl;
  });

#ifdef UWS_VCPKG
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code,
    char *message, size_t length) {
    ws->close();
#else
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
#endif
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen("127.0.0.1", port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
