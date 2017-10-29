
#include <iostream>
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <typeinfo>
#include "Eigen-3.3/Eigen/Dense"
#include "constants.h"
#ifndef VEHICLE_H
#define VEHICLE_H


// unnamed namespace only because the implementation is in this
// header file and we don't want to export symbols to the obj files
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace vc
{

// spline interpolation
class Vehicle
{
  public:
      int id;
      double lane;

      double s=0.0;
      double s_dot=0.0;
      double s_dot_dot=0.0;
      double d=0.0;
      double d_dot=0.0;
      double d_dot_dot=0.0;

      double map_index;
      double dx;
      double dy;

      bool is_chaging_lane = false;
      double target_lane;

  public:
      // set default boundary condition to be zero curvature at both ends
      Vehicle(int id ,double s, double d, double velocity,bool is_chaging_lane, int target_lane) {
          this->is_chaging_lane = is_chaging_lane;
          this->d = d;
          this->s = s;
          this->s_dot = velocity;
          this->id = id;
          this->lane = getLine(d);
          if(is_chaging_lane){
            this->lane = target_lane;
          }
      }

      Vehicle() {
      }
      ~Vehicle(){}

      double getLine(double d);
      void initialize(vector<double> previous_path_x, vector<double>previous_path_y);
      vector<vector<double>> getHorizon();
      vector<string> getAvailableStates();
      map<string, double> realize_state(string state, int prev_size, map<int, vector<vector<double>>> predictions, bool ignite);

      vector<double> check_for_car_front(int lane, map<int, vector<vector<double>>> predictions);
      vector<double> check_for_car_back(int lane, map<int, vector<vector<double>>> predictions);

      map<string, vector<double>> get_trajectory(map<string, double> target_data);
      vector<double> get_JMT_coeffs(vector<double> start, vector<double> target);

      double calculate_cost(map<string, vector<double>> trajectory_data , map<int, vector<vector<double>>> predictions, double target_lane  );
      double cost_collision(map<string, vector<double>> trajectory_data , map<int, vector<vector<double>>> predictions,double target_lane  );
      double cost_speed(vector<double> s_trajectory);
      double cost_lane_change(vector<double> s_trajectory,map<int, vector<vector<double>>> predictions, double target_lane ,vector<double> car_back);

};

double Vehicle::getLine(double _d){
  double _lane = -1;
  if(_d >=0 && _d< 4 ){
    _lane = 0;
  }
  else if (_d>= 4.0 && _d<8.0 ){
    _lane = 1;
  }
  else if( _d>= 8.0){
    _lane = 2;
  }
  return _lane;
}

void Vehicle::initialize(vector<double> previous_path_x, vector<double>previous_path_y){
  //starting point in trajectory
  double pos1_x = previous_path_x[previous_path_x.size()-1];
  double pos1_y = previous_path_y[previous_path_y.size()-1];
  double pos2_x = previous_path_x[previous_path_x.size()-2];
  double pos2_y = previous_path_y[previous_path_y.size()-2];
  double pos3_x = previous_path_x[previous_path_x.size()-3];
  double pos3_y = previous_path_y[previous_path_y.size()-3];

  double v1_x = (pos1_x - pos2_x)/0.02;
  double v1_y = (pos1_y - pos2_y)/0.02;
  double v2_x = (pos2_x - pos3_x)/0.02;
  double v2_y = (pos2_y - pos3_y)/0.02;

  double a_x = (v1_x - v2_x)/0.02;
  double a_y = (v1_y - v2_y)/0.02;

  double s_x = -1 * this->dy;
  double s_y = -1 * this->dx;

  this->d_dot = dx * v1_x + dy * v1_y;
  this->s_dot_dot = sqrt( pow(a_x,2) + pow(a_y,2));//s_x * a_x + s_y * a_y;
  this->d_dot_dot = dx * a_x + dy * a_y;

}

vector<vector<double>> Vehicle::getHorizon(){
    //constant velocity
    vector<vector<double>> predictions;
    double t = TIME;
    for( int i = 0; i < HORIZON; i++)
    {
      double s = this->s + this->s_dot * t;
      vector<double> pred = {s, this->d};
      predictions.push_back(pred);
      t+=TIME;
  	}
    return predictions;

  }

vector<string> Vehicle::getAvailableStates(){
  vector<string> available_states={"KL", "LCL", "LCR"};
  if(this->lane == 0){
      available_states={"KL", "LCR"};
  }
  else if(this->lane == 2){
    available_states={"KL", "LCL"};
  }
  return available_states;
}

map<string, double> Vehicle::realize_state(string state, int prev_size,map<int, vector<vector<double>>> predictions,  bool ignite){
  // find next car in this lane
  // find available room and proper velocity
    double buffer = 40;
    map<string, double> result;

    //s_dot m/s
    //minimizing jerk
    result["target_s_dot"] = this->s_dot + TIME*HORIZON*9;


    result["target_s_dot_dot"]= 0.0; //const velocity
    result["target_d_dot"]= 0.0; // no lateral velocity
    result["target_d_dot_dot"]= 0.0; //const velocity

    // approx target_s by avg speed of currect s_dot to target
    double avg_speed =(result["target_s_dot"] + this->s_dot)/2;
    result["target_s"] = this->s + avg_speed *TIME*HORIZON; // + 0.5*pow(0.02*5,2)*this->s_dot_dot;

    int lane = this->lane;

    if(state.compare("KL") == 0)
    {
      result["target_d"] = this->d;
    }
    else if(state.compare("LCL") == 0)
    {
      lane  =this->lane-1;
      result["target_d"] = (double)lane * 4.0 +2.0;
    }
    else if(state.compare("LCR") == 0)
    {
      lane  =this->lane+1;
      result["target_d"] = (double)lane* 4.0 +2.0;
    }

    vector<double> car_front = check_for_car_front(lane, predictions);
    if(car_front[2] != -1){
      if(result["target_s"] <= car_front[0]){
        double diff = car_front[0] - buffer;
        if(diff <= this->s){
          // my car is already too close to the front car
          result["target_s"] = this->s+ (car_front[3] - this->s)/5 ;
          result["target_s_dot"]= this->s_dot-1;
        }
        else if(car_front[0] - result["target_s"] < buffer){
          result["target_s"] =  this->s+(car_front[0] - this->s)/2;
          result["target_s_dot"] = this->s_dot-1;
        }
        //else nothing to do
      }
      else{
        result["target_s"] = car_front[0] - buffer;
        result["target_s_dot"] -=2;
      }

    }
    // cout<<"      Realized Target"<<endl;
    // cout<<"            s: "<<result["target_s"]<<endl;
    // cout<<"            s_dot: "<<result["target_s_dot"]<<endl;
    // cout<<"            s_dot_dot: "<<result["target_s_dot_dot"]<<endl;
    // cout<<"            d: "<<result["target_d"]<<endl;
    // cout<<"            d_dot: "<<result["target_d_dot"]<<endl;
    // cout<<"            d_dot_dot: "<<result["target_d_dot_dot"]<<endl;

    result["target_lane"] = lane;

    if(result["target_s_dot"] > SPEED_MAX * MPH_TO_MS){
      result["target_s_dot"] = SPEED_MAX * MPH_TO_MS;
    }
    return result;
}

vector<double> Vehicle::check_for_car_front(int lane, map<int, vector<vector<double>>> predictions){
  //find the nearest car in the back
  double nearest_s = INFINITY;
  double speed = 0.0;
  double id = -1;
  double s_start_ , s_end_;
  double s_start , s_end;

  for(map<int, vector<vector<double>>>::iterator it = predictions.begin(); it!= predictions.end(); it++){
    vector<vector<double>> s_and_d = it->second;
    double _lane = getLine(s_and_d[0][1]);
    if( _lane == lane ){
      s_start = s_and_d[0][0];
      s_end = s_and_d[s_and_d.size()-1][0];
      double s_before_end = s_and_d[s_and_d.size()-2][0];
      if(s_end < nearest_s && s_start > this->s){
        nearest_s = s_end;
        //speed m/s
        speed = (s_end - s_before_end)/(0.02);
        id = it->first;
        s_end_ = s_end;
        s_start_= s_start;
      }
    }
  }
  return {nearest_s, speed, id, s_start_, s_end_};
}

vector<double> Vehicle::check_for_car_back(int lane, map<int, vector<vector<double>>> predictions){
  //find the nearest car at the back
  double nearest_s = 0;
  double speed = 0.0;
  double id = -1;
  double s_start_ , s_end_;
  double s_start , s_end;
  for(map<int, vector<vector<double>>>::iterator it = predictions.begin(); it!= predictions.end(); it++){
    vector<vector<double>> s_and_d = it->second;
    double _lane = getLine(s_and_d[0][1]);
    if( _lane == lane ){
      s_start = s_and_d[0][0];
      s_end = s_and_d[s_and_d.size()-1][0];
      double s_before_end = s_and_d[s_and_d.size()-2][0];
      if(s_end > nearest_s && s_start < this->s){
        nearest_s = s_end;
        //speed m/s
        speed = (s_end - s_before_end)/(0.02);
        id = it->first;
        s_end_ = s_end;
        s_start_= s_start;
      }
    }
  }
  return {nearest_s, speed, id, s_start_, s_end_};
}


map<string, vector<double>> Vehicle::get_trajectory(map<string, double> target_data){
  map<string, vector<double>> result;
  vector<double> target_s = {target_data["target_s"],target_data["target_sd"] ,target_data["target_sdd"] };
  vector<double> target_d = {target_data["target_d"],target_data["target_dd"],target_data["target_ddd"]};

  vector<double> start_s ={ this->s, this->s_dot, this->s_dot_dot};
  vector<double> start_d ={ this->d, this->d_dot, this->d_dot_dot};

  vector<double> coeffs_s = get_JMT_coeffs(start_s, target_s);
  vector<double> coeffs_d = get_JMT_coeffs(start_d, target_d);

  // now I have all coeefs to generate s and d values for trajectory
  // s = a0 + a1*T + a2*T^2 + a3*T^3 + a4*T^4 + a5 * T^5
  // d = a0 + a1*T + a2*T^2 + a3*T^3 + a4*T^4 + a5 * T^5
  //generate 20 s_points and d_points for trajectory

  double t  = TIME;

  for(int i = 0 ; i< TRAJECTORY_SAMPLES; i++){
    double s_t = coeffs_s[0] + coeffs_s[1] * t + coeffs_s[2] * pow(t,2) +
                 coeffs_s[3] * pow(t,3)  + coeffs_s[4] * pow(t,4)  + coeffs_s[5] * pow(t,5) ;
    double d_t = coeffs_d[0] + coeffs_d[1] * t + coeffs_d[2] * pow(t,2) +
                 coeffs_d[3] * pow(t,3)  + coeffs_d[4] * pow(t,4) + coeffs_d[5] * pow(t,5) ;

    result["s_trajectory"].push_back(s_t);
    result["d_trajectory"].push_back(d_t);
    t+=TIME;
  }
  // cout<<"      Trajectory_data: "<<endl;
  // cout<<"            From s:"<<result["s_trajectory"][0]<<" ,d:"<<result["d_trajectory"][0]<<endl;
  // cout<<"            To s:"<<result["s_trajectory"][result["s_trajectory"].size()-1]<<" ,d:"<<result["d_trajectory"][result["s_trajectory"].size()-1]<<endl;
  return result;
}

vector<double> Vehicle::get_JMT_coeffs(vector<double> start, vector<double> end){
  double a_0= start[0], a_1 = start[1], a_2= 0.5*start[2];
  double a_3, a_4, a_5;
  double T = HORIZON;
  MatrixXd A =MatrixXd(3,3);
  A<< pow(T,3),pow(T,4),pow(T,5),
     3*pow(T,2),4*pow(T,3),5*pow(T,4),
     6*T,12*pow(T,2),20*pow(T,3);
  MatrixXd B= MatrixXd(3,1);
  B<< end[0] - (start[0]+start[1]*T+0.5*start[2]*T*T),
     end[1]-(start[1]+ start[2]*T),
     end[2]-start[2];
  MatrixXd Ai= A.inverse();
  MatrixXd result= Ai*B;
  a_3 = result(0);
  a_4 = result(1);
  a_5 = result(2);
  return {a_0,a_1,a_2,a_3,a_4,a_5};
}

double Vehicle::calculate_cost( map<string, vector<double>> trajectory_data , map<int, vector<vector<double>>> predictions, double target_lane  ){
  vector<double> car_back = check_for_car_back(target_lane, predictions);
  double total_cost =0;
  double _collision = cost_collision(trajectory_data, predictions, target_lane);
  double _speed = cost_speed(trajectory_data["s_trajectory"]);
  double _lane_change = cost_lane_change(trajectory_data["s_trajectory"],predictions,target_lane, car_back  );

  total_cost = _collision+ _speed + _lane_change;
  return total_cost;
}

double Vehicle::cost_collision(map<string, vector<double>> trajectory_data , map<int, vector<vector<double>>> predictions, double target_lane){
  // 20point s,d in trajectory data & other vehiles predictions for horizon 5
  double minimum_space = INFINITY;
  double distance= INFINITY;
  int car_id;
  vector<double> s_trajectory = trajectory_data["s_trajectory"];
  vector<double> d_trajectory = trajectory_data["d_trajectory"];
  if(!predictions.empty())
  {
    for(map<int, vector<vector<double>>>::iterator it= predictions.begin(); it!= predictions.end(); it++){
      double vehicle_s = it->second[0][0];
      double vehicle_d = it->second[0][1];
      double vehicle_lane = getLine(vehicle_d);
      //if at any point in trajectory there's a collision add to cost
      if(vehicle_lane == this->lane || vehicle_lane == target_lane){
        // we have 20 points in trajectory_data
        for(int i=0; i<s_trajectory.size(); i++){
          double _s = s_trajectory[i] - vehicle_s;
          double _d = d_trajectory[i] - vehicle_d;
          distance = sqrt(pow(_s,2) + pow(_d,2));
          if(distance < minimum_space){
            minimum_space = distance;
            car_id = it->first;
          }
        }
      }

    }
  }
  // Restricted AREA
  if(minimum_space < 10){
    return COLLISON_COST*10;
  }
  else if(minimum_space< 100){
    return COLLISON_COST/minimum_space;
  }
  else{
    return 0;
  }
}

double Vehicle::cost_speed(vector<double> s_trajectory){
  double _speed = this->s_dot;
  for(int i = 0 ; i< s_trajectory.size()-1; i++){
    //if at any point in trajectory speed is exceeding max LIMIT add to cost
    _speed = (s_trajectory[i+1] - s_trajectory[i])/0.02;
    if(_speed > 50 * MPH_TO_MS){
      return SPEED_COST;
    }
  }
  if(abs(_speed) < 30* MPH_TO_MS){
    return SPEED_COST/abs(_speed);
  }

  return 0;
}

double Vehicle::cost_lane_change(vector<double> s_trajectory,map<int, vector<vector<double>>> predictions, double target_lane ,vector<double> car_back){
    double total = LANE_CHANGE_COST;
    double slower = 500;
    if(target_lane != this->lane){
      vector<double> car_front_current_lane = check_for_car_front(this->lane, predictions);
      vector<double> car_front_target_lane = check_for_car_front(target_lane, predictions);
      if(this->s_dot < 25 * MPH_TO_MS){
        //not a suitable speed for lane change
        total*=10;
      }

      //  If there is a car in back
      if(car_back[2] != -1){
         if( s_trajectory[ s_trajectory.size()-1] - car_back[0] < 30 || this->s -car_back[3] < 20){
           //dont change lane if car is going to slow down
            total+=500;
         }
      }

      //  If there is a car in front
      if(car_front_target_lane[2] != -1){

        //COMPARING 2 CASES FOR FRONT CAR IN TARGET_LANE & CURRENT_LANE
        // 1- if the car in target_lane is slower than the car in current_lane add to cost
        if(car_front_target_lane[1] < car_front_current_lane[1] - 5)
        {
          total+=slower;
        }
        // 2- if the car in target_lane is closer than the car in current_lane add to cost
        if(car_front_target_lane[0] - car_front_current_lane[0] < 30)
        {
          total+=slower;
        }

        // if the car in target_lane is blocking my trajectory add to cost
        if(car_front_target_lane[0]<s_trajectory[ s_trajectory.size()-1])
        {
          total += 1000/abs(car_front_target_lane[0] - s_trajectory[ s_trajectory.size()-1]);
        }
        else{
          //reward changing lane becuase it's not blocking my trajectory
          total -= slower;
        }

      }
      else{
        // NO CAR IN FRONT REWARD
        total-=500;
      }
      return total;
    }
    else{
      return 0;
    }
}


} // namespace vc
#endif /* VEHICLE_H */
