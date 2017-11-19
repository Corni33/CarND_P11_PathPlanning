#include <fstream>
#include <math.h>
#include <algorithm>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "spline.h"
//#include "datatable.h"
//#include "bspline.h"
//#include "tinysplinecpp.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
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

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
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

vector<double> global2vehicle(double x_global, double y_global, double car_x, double car_y, double car_yaw)
{
	double x_veh_tr = x_global - car_x;
	double y_veh_tr = y_global - car_y;

	double x_veh = cos(car_yaw)*x_veh_tr - sin(car_yaw)*y_veh_tr;
	double y_veh = sin(car_yaw)*x_veh_tr + cos(car_yaw)*y_veh_tr;

	return {x_veh, y_veh};
}

vector<double> vehicle2global(double x_veh, double y_veh, double car_x, double car_y, double car_yaw)
{
	double x_global_tr = cos(car_yaw)*x_veh + sin(car_yaw)*y_veh;
	double y_global_tr =-sin(car_yaw)*x_veh + cos(car_yaw)*y_veh;

	double x_global = x_global_tr + car_x;
	double y_global = y_global_tr + car_y;

	return {x_global, y_global};
}

int get_lane_index(double d)
{
	const double lane_width = 4.0;
	return (int)(d/lane_width) + 1;
}

double IDM_acc(double dist, double v_ego, double v_front)
{
	const double a = 2.0;
	const double b = 5.0;
	const double T = 1.2;
	const double s0 = 5.0;
	const double v0 = 50*0.44704; // 50 mph target velocity

	const double delta_v = v_ego - v_front;

	const double delta = 4.0;

	double s_star = s0 + v_ego*T + v_ego*delta_v/(2.0*sqrt(a*b));
	double acc = a*(1.0 - pow(v_ego/v0, delta) - pow(s_star/dist, 2));

	return acc;
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

	// close waypoint loop -> first point = last point	
	//int n_waypoints = map_waypoints_x.size();
	//double dist = distance(map_waypoints_x[n_waypoints-1], map_waypoints_y[n_waypoints-1], map_waypoints_x[0], map_waypoints_y[0]);

	//map_waypoints_x.push_back(map_waypoints_x[0]);
	//map_waypoints_y.push_back(map_waypoints_y[0]);	
	//map_waypoints_s.push_back(map_waypoints_s[n_waypoints-1] + dist);
	//map_waypoints_dx.push_back(d_x); // TODO (necessary?)
	//map_waypoints_dy.push_back(d_y);
	//++n_waypoints;

  //&map_interp_s, &map_interp_x, &map_interp_y
	h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
						
						car_yaw = deg2rad(car_yaw); 
						car_speed = 0.44704*car_speed; // convert from mph to m/s

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
						auto sensor_fusion = j[1]["sensor_fusion"];
						

						const int n_path_points = 30; // number of path points
						const double dt = 0.02; // simulator step time
						const double lane_width = 4.0;

						vector<double> next_x_vals;
          	vector<double> next_y_vals;	

						double ref_s = car_s;
						double ref_d = car_d;
						double ref_x = car_x;
						double ref_y = car_y;
						double ref_yaw = car_yaw;
						double ref_speed = car_speed;

						int prev_size = previous_path_x.size();	

						vector<double> lp_x, lp_y;					
						
						// if previous path is long enough, start from its end with new path planning
						if (prev_size > 3)
						{
							ref_s = end_path_s;
							ref_d = end_path_d;
							ref_x = previous_path_x[prev_size-1];
							ref_y = previous_path_y[prev_size-1];							

							double dx = (double)previous_path_x[prev_size-1] - (double)previous_path_x[prev_size-2];
							double dy = (double)previous_path_y[prev_size-1] - (double)previous_path_y[prev_size-2];
							
							ref_yaw = atan2(dy,dx);	
							ref_speed = sqrt(dx*dx + dy*dy)/dt;

							auto p0 = global2vehicle(ref_x - dx, ref_y - dy, ref_x, ref_y, ref_yaw);
							auto p1 = global2vehicle(ref_x, ref_y, ref_x, ref_y, ref_yaw);

							lp_x.push_back(p0[0]);
							lp_x.push_back(p1[0]);

							lp_y.push_back(p0[1]);
							lp_y.push_back(p1[1]);

							for (int k=0; k < prev_size; ++k)
							{
								next_x_vals.push_back(previous_path_x[k]);
								next_y_vals.push_back(previous_path_y[k]);
							}					
						}	
						else 
						{
							auto p0 = global2vehicle(ref_x - cos(ref_yaw), ref_y - sin(ref_yaw), ref_x, ref_y, ref_yaw);
							auto p1 = global2vehicle(ref_x, ref_y, ref_x, ref_y, ref_yaw);

							lp_x.push_back(p0[0]);
							lp_x.push_back(p1[0]);

							lp_y.push_back(p0[1]);
							lp_y.push_back(p1[1]);
						}

						cout << "ref_yaw = " << ref_yaw << endl;
						cout << "ref_x = " << ref_x << endl;
						cout << "ref_s = " << ref_s << ", car_s = " << car_s <<  endl;
						cout << "ref_speed = " << ref_speed << endl;			


						//find out ego-vehicle lane
						const int ego_lane_index = get_lane_index(ref_d);

						double acc_ego = IDM_acc(9999, ref_speed, 0.0); 						

						// handle detected traffic vehicles
						/*for (auto traffic_obj : sensor_fusion)
						{
							int id = traffic_obj[0];
							double px = traffic_obj[1];
							double py = traffic_obj[2];
							double vx = traffic_obj[3]; // in m/s
							double vy = traffic_obj[4];
							double s = traffic_obj[5];
							double d = traffic_obj[6];							

							if (get_lane_index(d) == ego_lane_index)
							{
								if (s > car_s) //TODO: what about loop closing of the track?
								{
									double v_traffic = sqrt(vx*vx + vy*vy); 
									acc_ego = IDM_acc(s - car_s, car_speed, v_traffic);  
								}
							}
							
						}*/ 

						
						//const int n_wp_behind = 2; 
						const int n_wp_forward = 3;
						const double spacing = 30; // m

						for (int k = 1; k <= n_wp_forward; ++k)
						{
							auto xy_global = getXY(ref_s + k*spacing, (ego_lane_index-1 + 0.5)*lane_width, map_waypoints_s, map_waypoints_x, map_waypoints_y);										
							auto lane_point = global2vehicle(xy_global[0], xy_global[1], ref_x, ref_y, ref_yaw);
							lp_x.push_back(lane_point[0]);
							lp_y.push_back(lane_point[1]);
						}					
						
						//for(p : lp_x) cout << p << ", " ;
						//cout << endl;

						tk::spline lane_spline;
						lane_spline.set_points(lp_x, lp_y);		


						double v_ego = ref_speed;					
						//acc_ego = 2.0;	

						cout << "car_speed = " << car_speed << endl;
						
						double x_path = 0.0;
						double y_last = 0.0;

						for(int i = next_x_vals.size(); i < n_path_points; i++)
						{		
							// ds is the distance the vehicle should cover in one timestep	
							double ds = v_ego*dt + 0.5*acc_ego*dt*dt;
							v_ego += acc_ego*dt;

							// find a point on the spline that has distance ds from the last path point							
							// initial approximation 
							double y_path = lane_spline(x_path + ds);
							
							double dy = y_path - y_last;
							double dst = sqrt(dy*dy + ds*ds);
							double ratio = ds/dst; 

							// improve approximation
							x_path += ds*ratio;			
							y_path = lane_spline(x_path);																												

							auto xy_global = vehicle2global(x_path, y_path, ref_x, ref_y, ref_yaw);

							next_x_vals.push_back(xy_global[0]);
							next_y_vals.push_back(xy_global[1]);	

							y_last = y_path;								
						}

						//weight with previous path
						/*if (prev_size > 20)
						{
							int n_weight_points = 20;
							for (int k=0; k < prev_size; ++k)
							{
								double w = max(0.0, 1.0 - (double)k/n_weight_points); // linear weighting

								next_x_vals[k] = (1.0-w)*next_x_vals[k] + w*(double)previous_path_x[k];
								next_y_vals[k] = (1.0-w)*next_y_vals[k] + w*(double)previous_path_y[k]; 
							}
						}*/

						cout << "acc_ego = " << acc_ego << endl;

						/*cout << "previous_path_x: ";
						for(double p : previous_path_x) cout << p << ", " ;
						cout << endl;

						cout << "next_x_vals: ";
						for(p : next_x_vals) cout << p << ", " ;
						cout << endl;	*/					


						json msgJson;

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

						//this_thread::sleep_for(chrono::milliseconds(100));
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