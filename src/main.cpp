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
#include "vehicle.h"

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

  map<int, vector<double>> prev_sensor_fusion_data ;

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
  vc::Vehicle my_vehicle= vc::Vehicle();
  int frame_counter=0;
  bool is_chaging_lane = false;
  int target_lane = -1;

  h.onMessage([&prev_sensor_fusion_data,&my_vehicle, &is_chaging_lane, &target_lane, &frame_counter, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          double _time = 0.02;
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

            my_vehicle = vc::Vehicle(77, car_s, car_d, car_speed, 0, is_chaging_lane, target_lane);



          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];





            int prev_size = previous_path_x.size();
            // if(prev_size>0){
            //   car_s = end_path_s;
            // }
            cout<<"Frame: "<<frame_counter<<endl;
            cout<<"MY VEHICLE: "<<endl;
            cout<<"speed: "<<car_speed<<endl;
            cout<<"s: "<<car_s<<endl;
            cout<<"car d : "<<car_d<<", end_path_d: "<<end_path_d<<endl;
            cout<<"LANE: "<<my_vehicle.lane<<endl;
            cout<<"is_lane_changing: "<<is_chaging_lane<<endl;

            cout<<"-----------------------------------------------------"<<endl;
            cout<<"Sensor Fusion:"<<endl;

            for(int i = 0 ; i< sensor_fusion.size(); i++)
            {
               //x, y, vx, vy, s, d
               int v_id = sensor_fusion[i][0];
               double v_s = sensor_fusion[i][5];
               double v_d = sensor_fusion[i][6];
               double v_velocity = sqrt(sensor_fusion[i][3] * sensor_fusion[i][3]+ sensor_fusion[i][4] * sensor_fusion[i][4]);

               double v_acceleration = 0;
               double v_x = sensor_fusion[i][1];
               double v_y = sensor_fusion[i][2];

               vc::Vehicle temp_vehicle = vc::Vehicle(v_id, v_s, v_d, v_velocity, v_acceleration, false, -1);
               cout<<"id: "<<temp_vehicle.id<<", s:"<<temp_vehicle.s<<", d:"<<temp_vehicle.d;
               cout<<", v: "<<temp_vehicle.velocity<<", lane:"<<temp_vehicle.lane<<endl;
               //valid lane
               if(temp_vehicle.lane >= 0)
               {
                 double temp_next_s_val = temp_vehicle.s + ((double)prev_size*0.02*temp_vehicle.velocity);
                 //is in front of my car
                 if(temp_next_s_val > my_vehicle.s )
                  {
                    // if ()
                    // {
                    //    my_vehicle.velocity = 29.5;
                    // }
                    if(temp_vehicle.lane == my_vehicle.lane)
                    {
                      if(temp_next_s_val-my_vehicle.s < 50){
                        cout<<"COLLISION with:"<< temp_vehicle.id<<" s:"<<temp_vehicle.s<<" next_s"<<temp_next_s_val<<endl;
                        my_vehicle.collision_flag = true;
                      }
                    }

                    //find the front car closest to my car
                   if(my_vehicle.has_value_front[temp_vehicle.lane]){
                     if( temp_next_s_val < my_vehicle.closest_vehicles_front[temp_vehicle.lane].s)
                      {
                        my_vehicle.closest_vehicles_front[temp_vehicle.lane] = temp_vehicle;
                      }
                   }
                   else{
                     my_vehicle.closest_vehicles_front[temp_vehicle.lane] = temp_vehicle;
                     my_vehicle.has_value_front[temp_vehicle.lane] = true;
                   }
                }
                //is at the back of my car
                else{

                  if(my_vehicle.has_value_back[temp_vehicle.lane]){
                    if( temp_next_s_val > my_vehicle.closest_vehicles_back[temp_vehicle.lane].s)
                     {
                       my_vehicle.closest_vehicles_back[temp_vehicle.lane] = temp_vehicle;
                     }
                  }
                  else{
                    my_vehicle.closest_vehicles_back[temp_vehicle.lane] = temp_vehicle;
                    my_vehicle.has_value_back[temp_vehicle.lane] = true;
                  }

                }

              }
            }

            /* states */
            vector<string> available_states={"KL", "LCL", "LCR"};
            map<string, double> costs = {{"KL",INFINITY}, {"LCL", INFINITY}, {"LCR", INFINITY}};
            double best_speed = my_vehicle.velocity;
            double best_lane = my_vehicle.lane;
            double best_cost = INFINITY;

            if(my_vehicle.lane == 0){
                available_states={"KL", "LCR"};
            }
            else if(my_vehicle.lane == 2){
              available_states={"KL", "LCL"};
            }


            if((frame_counter > 50 && !my_vehicle.is_chaging_lane )|| my_vehicle.collision_flag){
              for(int i=0; i<available_states.size(); i++)
              {
                map<string, double>result = my_vehicle.realize_state( available_states[i], prev_size);
                costs[available_states[i]] = my_vehicle.calculate_cost(result["available_space_front"], result["available_space_back"],result["lane"]);

                //TODO temporary keep line
                if(costs[available_states[i]]< best_cost)
                {
                  best_lane = result["lane"];
                  best_cost = costs[available_states[i]];
                }
              }

              for(map<string, double>::iterator cost_it = costs.begin(); cost_it != costs.end(); cost_it++)
              {
                cout<<"cost : "<<cost_it->first<<" , "<<cost_it->second<<endl;
              }
              if (my_vehicle.lane != best_lane)
              {
                cout<<"LANE CHANGE TO *** "<< best_lane<<endl;
                my_vehicle.lane = best_lane;
                my_vehicle.is_chaging_lane = true;
                is_chaging_lane = true;
                target_lane = best_lane;
                my_vehicle.target_lane = best_lane;
              }
              else{
                cout<<"KEEP LANE *** "<< my_vehicle.lane<<endl;
                frame_counter=0;
                my_vehicle.is_chaging_lane = false;
                is_chaging_lane = false;
              }
            }

            frame_counter++;

            my_vehicle.adjust_speed(prev_size);

            if(frame_counter == 200)
            {
              frame_counter=0;
              my_vehicle.is_chaging_lane = false;
              is_chaging_lane= false;
            }
              cout<<"-----------------------------------------------------"<<endl;
            /*
            Smoothing Trajectory
              GOAL: interpolating more waypoints with spline to fill trajectory with more points
            */

            double ref_v = 49.5; //mph
            if(my_vehicle.is_initialized)
              {
                if(my_vehicle.velocity < 0.5){
                  my_vehicle.velocity += 0.5;
                }
                ref_v = my_vehicle.velocity;

              }
            //-----------------------------------------------------------------
            //-1 Keep current car information as ref values (x,y,yaw)
            vector<double> pts_x, pts_y;
            double ref_x = car_x,
                   ref_y = car_y,
                   ref_yaw= deg2rad(car_yaw);
            //-----------------------------------------------------------------
            //-2 Use 2 previous waypoints to make path tanget to car
            if(previous_path_x.size()<2){
              //go backward 1 step
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);
              pts_x.push_back(prev_car_x);
              pts_x.push_back(car_x);
              pts_y.push_back(prev_car_y);
              pts_y.push_back(car_y);
            }
            else{
              ref_x = previous_path_x[previous_path_x.size()-1];
              ref_y = previous_path_y[previous_path_x.size()-1];
              double ref_x_prev = previous_path_x[previous_path_x.size()-2];
              double ref_y_prev = previous_path_y[previous_path_x.size()-2];
              ref_yaw = atan2( ref_y - ref_y_prev, ref_x - ref_x_prev);
              pts_x.push_back(ref_x_prev);
              pts_x.push_back(ref_x);
              pts_y.push_back(ref_y_prev);
              pts_y.push_back(ref_y);
            }
            //-----------------------------------------------------------------
            //-3 Add a couple of points within 30m ahead of car
            double AHEAD = 30;
            double a = 0, b = 0;
            double ref_d0 =2+ 4*my_vehicle.lane;
            double ref_d1 = ref_d0;
            double ref_d2 = ref_d1;
            if(my_vehicle.is_chaging_lane){
              AHEAD = 50;
              if(my_vehicle.target_lane > my_vehicle.lane)
              {

                ref_d0 += 0.2;
                ref_d1= ref_d0 + 0.2;
                ref_d2 = ref_d1+ 0.2;
              }
              else{
                  ref_d0 -= 0.2;
                  ref_d1= ref_d0 - 0.2;
                  ref_d2 = ref_d1- 0.2;
              }
            }
            double next_s0 = car_s+AHEAD;
            double next_d0 = ref_d0;
            vector<double> next_wp0 = getXY(next_s0, next_d0, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            double next_s1 = car_s+ 2*AHEAD;
            double next_d1 = ref_d1;
            vector<double> next_wp1 = getXY(next_s1, next_d1, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            double next_s2 = car_s+ 3*AHEAD;
            double next_d2 = ref_d2;
            vector<double> next_wp2 = getXY(next_s2, next_d2, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            pts_x.push_back(next_wp0[0]);
            pts_y.push_back(next_wp0[1]);
            pts_x.push_back(next_wp1[0]);
            pts_y.push_back(next_wp1[1]);
            pts_x.push_back(next_wp2[0]);
            pts_y.push_back(next_wp2[1]);

            //-----------------------------------------------------------------
            // -4 transforming points to local car's coordinates
            for(int i =0; i<pts_x.size(); i++){
              double shift_x = pts_x[i] - ref_x;
              double shift_y = pts_y[i] - ref_y;
              pts_x[i]=(shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
              pts_y[i]=(shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
            }
            //-----------------------------------------------------------------
            // -5 fit the points
            tk::spline s;
            s.set_points(pts_x, pts_y);


            //-----------------------------------------------------------------
            // -6 keep the previous points
            vector<double> next_x_vals;
          	vector<double> next_y_vals;
            for(int i =0 ; i< previous_path_x.size(); i++){
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
            //-----------------------------------------------------------------
            // -7 add more points and convert to global coordinate
            double target_x = (double)AHEAD;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);

            double add_on=0;

            for(int i =0 ; i<= 50-previous_path_x.size(); i++ ){
              double N = target_dist/(_time*ref_v/2.24);
              double x_point = add_on + target_x/N;
              double y_point = s(x_point);
              add_on = x_point;

              //global conversion
              double x_ref = x_point;
              double y_ref = y_point;
              x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
              y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));
              x_point+=ref_x;
              y_point+=ref_y;
              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }



          	json msgJson;


            double dist_inc = 0.5;
            // for(int i = 0; i < 50; i++)
            // {
            //      double next_s_val = car_s + (i+1)*dist_inc;
            //      double next_d_val = 6; // distance from middle doubled-yellow lane
            //      vector<double> next_xy = getXY(next_s_val, next_d_val, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            //      next_x_vals.push_back(next_xy[0]);
            //      next_y_vals.push_back(next_xy[1]);
            //     //  next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
            //     //  next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
            // }

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
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
