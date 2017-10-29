
#include <iostream>
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <typeinfo>
#include "Eigen-3.3/Eigen/Dense"
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

      bool is_initialized;
      double _time = 0.02;
      vector<bool> has_value_front={false,false,false};
      vector<bool> has_value_back={false,false,false};
      map<int,vc::Vehicle> closest_vehicles_front;
      map<int,vc::Vehicle> closest_vehicles_back;
      bool is_chaging_lane = false;
      double target_lane;
      bool collision_flag = false;
      double prev_available_space;
      double reducing_speed = false;
      double collision_space =0;

  public:
      // set default boundary condition to be zero curvature at both ends
      Vehicle(int id ,double s, double d, double velocity,bool is_chaging_lane, int target_lane) {
          this->is_chaging_lane = is_chaging_lane;
          this->d = d;
          this->s = s;
          this->s_dot = velocity;
          this->id = id;
          this->is_initialized = true;
          this->lane = getLine(d);
          if(is_chaging_lane){
            this->lane = target_lane;
          }


      }
      Vehicle() {
        this->is_initialized= false;
      }
      ~Vehicle(){}

      double getLine(double d);
      void initialize(vector<double> previous_path_x, vector<double>previous_path_y);
      vector<vector<double>> getHorizon(double horizon);
      vector<string> getAvailableStates();
      map<string, double> realize_state(string state, int prev_size, map<int, vector<vector<double>>> predictions, double horizon, bool ignite);
      double calculate_cost(map<string, vector<double>> trajectory_data , map<int, vector<vector<double>>> predictions, double target_lane  );

      vector<double> check_for_car_front(int lane, map<int, vector<vector<double>>> predictions, double horizon);
      vector<double> check_for_car_back(int lane, map<int, vector<vector<double>>> predictions, double horizon);
      vector<vector<double>>  check_for_car_side(double lane, map<int, vector<vector<double>>> predictions, double horizon);

      map<string, vector<double>> get_trajectory(map<string, double> target_data, double horizon);
      vector<double> get_JMT_coeffs(vector<double> start, vector<double> target, double horizon);

      double cost_collision(map<string, vector<double>> trajectory_data , map<int, vector<vector<double>>> predictions,double target_lane  );
      double cost_speed(vector<double> s_trajectory);
      double cost_lane_change(vector<double> s_trajectory,map<int, vector<vector<double>>> predictions, double target_lane ,vector<double> car_back);

};

double Vehicle::getLine(double _d){
  double _lane = -1;
  if(_d >=0 && _d< 4 ){
    _lane = 0;
  }
  // else if(_d >3.8 && _d<= 4.0 ){
  //   _lane = 0.5;
  // }
  else if (_d>= 4.0 && _d<8.0 ){
    _lane = 1;
  }
  // else if( _d>7.8 && _d<8.2){
  //   _lane = 1.5;
  // }
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

  // this->s_dot = s_x * v1_x + s_y * v1_y; current speed
  this->d_dot = dx * v1_x + dy * v1_y;


  this->s_dot_dot = sqrt( pow(a_x,2) + pow(a_y,2));//s_x * a_x + s_y * a_y;
  this->d_dot_dot = dx * a_x + dy * a_y;

}

vector<vector<double>> Vehicle::getHorizon(double horizon){
    vector<vector<double>> predictions;
    double t = 0.02;
    for( int i = 0; i < horizon; i++)
    {
      double s = this->s + this->s_dot * t;
      vector<double> pred = {s, this->d};
      predictions.push_back(pred);
      t+=0.02;
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

map<string, double> Vehicle::realize_state(string state, int prev_size,map<int, vector<vector<double>>> predictions, double horizon, bool ignite){
  // find next car in this lane
  // find available room and proper velocity
    double buffer = 40;
    map<string, double> result;

    //s_dot m/s
    //minimizing jerk
    result["target_s_dot"] = this->s_dot + 0.02*5*9;


    result["target_s_dot_dot"]= 0.0; //const velocity
    result["target_d_dot"]= 0.0; // no lateral velocity
    result["target_d_dot_dot"]= 0.0; //const velocity

    // result["target_s_dot"] = 49.0 * 0.44704;
    double avg_speed =(result["target_s_dot"] + this->s_dot)/2; //(this->s_dot + result["target_s_dot"]) /2;
    result["target_s"] = this->s + avg_speed *0.02*horizon; // + 0.5*pow(0.02*5,2)*this->s_dot_dot;

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

    vector<double> car_front = check_for_car_front(lane, predictions, horizon);
    if(car_front[2] != 900){

      cout<<"      Car FRONT: "<<car_front[2]<<endl;
      cout<<"            horizon from s:"<<car_front[3]<<" to s:"<<car_front[0]<<endl;
      cout<<"            lane:"<<lane<<endl;
      cout<<"            speed:"<<car_front[1]<<endl;

      cout<<"            target_s"<<result["target_s"]<<endl;
      if(result["target_s"] <= car_front[0]){
        double diff = car_front[0] - buffer;
        cout<<"            diff: "<<diff<<endl;
        if(diff <= this->s){
          // my car is already too close to the front car
          result["target_s"] = this->s+ (car_front[3] - this->s)/5 ;
          result["target_s_dot"]= this->s_dot-1;
          cout<<"            adjusting s to *: "<<result["target_s"] <<endl;
          cout<<"            adjusting sd to *: "<<result["target_s_dot"] <<endl;
        }
        else if(car_front[0] - result["target_s"] < buffer){
          result["target_s"] =  this->s+(car_front[0] - this->s)/2;
          result["target_s_dot"] = this->s_dot-1;
          cout<<"            adjusting s to **: "<<result["target_s"] <<endl;
          cout<<"            adjusting sd to *: "<<result["target_s_dot"] <<endl;
        }
        //else nothing to do
      }
      else{
        //target_s > car_front[0]
        result["target_s"] = car_front[0] - buffer;
        result["target_s_dot"] -=2;
        cout<<"***"<<endl;
      }

    }
    else
    {
      cout<<"      No Car in Front"<<endl;
    }

    vector<vector<double>> car_sides = check_for_car_side(lane, predictions, horizon);
    if(car_sides.size() > 0)
    {
      for(int i =0 ;i <car_sides.size(); i++){
        cout<<"      Car Side"<<endl;
        cout<<"            id: "<<car_sides[i][0]<<endl;
        cout<<"            speed: "<<car_sides[i][1]<<endl;
        cout<<"            s_start: "<<car_sides[i][2]<<endl;
        cout<<"            s_end: "<<car_sides[i][3]<<endl;
      }
    }
    else{
      cout<<"      NO Car Side"<<endl;
    }


    cout<<"      Realized Target"<<endl;
    cout<<"            s: "<<result["target_s"]<<endl;
    cout<<"            s_dot: "<<result["target_s_dot"]<<endl;
    cout<<"            s_dot_dot: "<<result["target_s_dot_dot"]<<endl;
    cout<<"            d: "<<result["target_d"]<<endl;
    cout<<"            d_dot: "<<result["target_d_dot"]<<endl;
    cout<<"            d_dot_dot: "<<result["target_d_dot_dot"]<<endl;

    result["target_lane"] = lane;

    if(result["target_s_dot"] > 46.0 * 0.44704){
      result["target_s_dot"] = 46.0 * 0.44704;
    }
    return result;
}

vector<vector<double>>  Vehicle::check_for_car_side(double lane, map<int, vector<vector<double>>> predictions, double horizon){
  double nearest_s = INFINITY;
  double speed = 0.0;
  double id = 900;
  double s_start_ , s_end_;
  double s_start , s_end;
  double zone_start = this->s - 20;
  double zone_end = this->s + 20;
  vector<vector<double>> cars_side;
  for(map<int, vector<vector<double>>>::iterator it = predictions.begin(); it!= predictions.end(); it++){
    vector<vector<double>> s_and_d = it->second;
    double _lane = getLine(s_and_d[0][1]);
    if( _lane == lane + 0.5 || _lane == lane - 0.5  ){
      // cout<<"- car id:"<<it->first;
      s_start = s_and_d[0][0];
      s_end = s_and_d[s_and_d.size()-1][0];
      // cout<<" s_start: "<<s_start<<", s_end: "<<s_end<<endl;
      double s_before_end = s_and_d[s_and_d.size()-2][0];
      if(s_start >= zone_start || s_start< zone_end ||
         s_end >= zone_start || s_end< zone_end){
           cars_side.push_back({ id,speed, s_start, s_end});
         }
    }
  }
  return cars_side;
}


vector<double> Vehicle::check_for_car_front(int lane, map<int, vector<vector<double>>> predictions, double horizon){
  double nearest_s = INFINITY;
  double speed = 0.0;
  double id = 900;
  double s_start_ , s_end_;
  double s_start , s_end;
  for(map<int, vector<vector<double>>>::iterator it = predictions.begin(); it!= predictions.end(); it++){
    vector<vector<double>> s_and_d = it->second;
    double _lane = getLine(s_and_d[0][1]);
    if( _lane == lane ){
      // cout<<"- car id:"<<it->first;
      s_start = s_and_d[0][0];
      s_end = s_and_d[s_and_d.size()-1][0];
      // cout<<" s_start: "<<s_start<<", s_end: "<<s_end<<endl;
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

vector<double> Vehicle::check_for_car_back(int lane, map<int, vector<vector<double>>> predictions, double horizon){
  double nearest_s = 0;
  double speed = 0.0;
  double id = 900;
  double s_start_ , s_end_;
  double s_start , s_end;
  for(map<int, vector<vector<double>>>::iterator it = predictions.begin(); it!= predictions.end(); it++){
    vector<vector<double>> s_and_d = it->second;
    double _lane = getLine(s_and_d[0][1]);
    if( _lane == lane ){
      // cout<<"- car id:"<<it->first;
      s_start = s_and_d[0][0];
      s_end = s_and_d[s_and_d.size()-1][0];
      // cout<<" s_start: "<<s_start<<", s_end: "<<s_end<<endl;
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



map<string, vector<double>> Vehicle::get_trajectory(map<string, double> target_data, double horizon){
  map<string, vector<double>> result;
  vector<double> target_s = {target_data["target_s"],target_data["target_sd"] ,target_data["target_sdd"] };
  vector<double> target_d = {target_data["target_d"],target_data["target_dd"],target_data["target_ddd"]};

  vector<double> start_s ={ this->s, this->s_dot, this->s_dot_dot};
  vector<double> start_d ={ this->d, this->d_dot, this->d_dot_dot};

  vector<double> coeffs_s = get_JMT_coeffs(start_s, target_s, horizon);
  vector<double> coeffs_d = get_JMT_coeffs(start_d, target_d, horizon);

  // now I have all coeefs to generate s and d values for trajectory
  // s = a0 + a1*T + a2*T^2 + a3*T^3 + a4*T^4 + a5 * T^5
  // d = a0 + a1*T + a2*T^2 + a3*T^3 + a4*T^4 + a5 * T^5
  //generate 20 s_points and d_points for trajectory

  double t  = 0.02;
  cout<<"      COEEFS s:"<<coeffs_s[0]<<", "<<coeffs_s[1]<<", "<<coeffs_s[2]<<" ,"<<coeffs_s[3]<<", "<<coeffs_s[4]<<", "<<coeffs_s[5]<<endl;
  cout<<"      COEEFS d:"<<coeffs_d[0]<<", "<<coeffs_d[1]<<", "<<coeffs_d[2]<<" ,"<<coeffs_d[3]<<", "<<coeffs_d[4]<<", "<<coeffs_d[5]<<endl;

  int TRAJECTORY_SAMPLES = 20 ;
  for(int i = 0 ; i< TRAJECTORY_SAMPLES; i++){
    double s_t = coeffs_s[0] + coeffs_s[1] * t + coeffs_s[2] * pow(t,2) +
                 coeffs_s[3] * pow(t,3)  + coeffs_s[4] * pow(t,4)  + coeffs_s[5] * pow(t,5) ;
    double d_t = coeffs_d[0] + coeffs_d[1] * t + coeffs_d[2] * pow(t,2) +
                 coeffs_d[3] * pow(t,3)  + coeffs_d[4] * pow(t,4) + coeffs_d[5] * pow(t,5) ;

    result["s_trajectory"].push_back(s_t);
    result["d_trajectory"].push_back(d_t);
    t+=0.02;
  }


  cout<<"      Trajectory_data: "<<endl;
  cout<<"            From s:"<<result["s_trajectory"][0]<<" ,d:"<<result["d_trajectory"][0]<<endl;
  cout<<"            To s:"<<result["s_trajectory"][result["s_trajectory"].size()-1]<<" ,d:"<<result["d_trajectory"][result["s_trajectory"].size()-1]<<endl;

  return result;
}

vector<double> Vehicle::get_JMT_coeffs(vector<double> start, vector<double> end, double T){
  double a_0= start[0], a_1 = start[1], a_2= 0.5*start[2];
  double a_3, a_4, a_5;

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
  vector<double> car_back = check_for_car_back(target_lane, predictions, 5);


  cout<<"      COST DETAIL:"<<endl;
  double total_cost =0;
  cout<<"            collision: "<<endl;
  double _collision = cost_collision(trajectory_data, predictions, target_lane);
  cout<<"                  "<<_collision<<endl;
  cout<<"            speed:"<<endl;
  double _speed = cost_speed(trajectory_data["s_trajectory"]);
  cout<<"                  "<<_speed<<endl;
  double _lane_change = cost_lane_change(trajectory_data["s_trajectory"],predictions,target_lane, car_back  );
  cout<<"            lane_change:"<<_lane_change<<endl;
  // total_cost += cost_lane_change(trajectory_data);

  // total_cost += cost_distance_from_goal(trajectory_data);
  total_cost = _collision+ _speed + _lane_change;

  cout<<"            total:"<<total_cost<<endl;
  return total_cost;
}

double Vehicle::cost_collision(map<string, vector<double>> trajectory_data , map<int, vector<vector<double>>> predictions, double target_lane){
  // 20point s,d in trajectory data & other vehiles predictions for horizon 5
  double minimum_space = INFINITY;
  double COLLISON_COST = 10000;
  double distance= INFINITY;
  int car_id;
  vector<double> s_trajectory = trajectory_data["s_trajectory"];
  vector<double> d_trajectory = trajectory_data["d_trajectory"];
  cout<<"                  prediction size:"<<predictions.size()<<endl;
  if(!predictions.empty())
  {
    for(map<int, vector<vector<double>>>::iterator it= predictions.begin(); it!= predictions.end(); it++){
      double vehicle_s = it->second[0][0];
      double vehicle_d = it->second[0][1];
      double vehicle_lane = getLine(vehicle_d);

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
  cout<<"                  distance to next car:"<<minimum_space<<" car_id:"<<car_id<<endl;
  // Restricted AREA is a circle around the car
  // assuming car radius is 1m
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
  double SPEED_COST = 1000;
  double _speed = this->s_dot;
  for(int i = 0 ; i< s_trajectory.size()-1; i++){
    _speed = (s_trajectory[i+1] - s_trajectory[i])/0.02;
    if(_speed > 50 * 0.44704){
      return SPEED_COST;
    }
    // if vehicle is gonna be slower in the end of trajectory penalize the decision
    // if (i == s_trajectory.size()-2 && _speed < this->s_dot){
    //   return SPEED_COST/5;
    // }
  }
  if(abs(_speed) < 40* 0.44704){
    return SPEED_COST/abs(_speed);
  }

  return 0;
}

double Vehicle::cost_lane_change(vector<double> s_trajectory,map<int, vector<vector<double>>> predictions, double target_lane ,vector<double> car_back){
    double LANE_CHANGE_COST = 400;
    double slower = 500;

    if(target_lane != this->lane){
      vector<double> car_front_current_lane = check_for_car_front(this->lane, predictions, 5);
      vector<double> car_front_target_lane = check_for_car_front(target_lane, predictions, 5);
      if(this->s_dot < 15 * 0.44704){
        //not a suitable speed for lane change
        LANE_CHANGE_COST*=10;
      }

      if(car_back[2] != 900){

        cout<<"      Car BACK: "<<car_back[2]<<endl;
        cout<<"            horizon from s:"<<car_back[3]<<" to s:"<<car_back[0]<<endl;
        cout<<"            lane:"<<target_lane<<endl;
        cout<<"            speed:"<<car_back[1]<<endl;
        //HANDLING BACK CAR
         if( s_trajectory[ s_trajectory.size()-1] - car_back[0] < 30 || this->s -car_back[3] < 20){
           //dont change lane if car is going to slow down
           cout<<"CAR BACK:500"<<endl;
            LANE_CHANGE_COST+=500;
         }
      }

      //  CAR IN FRONT?
      if(car_front_target_lane[2] != 900){
        //speed is slowing down add to cost
        if(car_front_target_lane[1] < car_front_current_lane[1] - 5)
        {
          cout<<"            slower*:"<<endl;
          LANE_CHANGE_COST+=slower;
        }
        if(car_front_target_lane[0] - car_front_current_lane[0] < 30)
        {
          LANE_CHANGE_COST+=slower;
          cout<<"            slower**:"<<endl;
        }

        if(car_front_target_lane[0]<s_trajectory[ s_trajectory.size()-1])
        {
            cout<<"            slower***:"<<endl;
          LANE_CHANGE_COST += 1000/abs(car_front_target_lane[0] - s_trajectory[ s_trajectory.size()-1]);
        }
        else{
          //reward changing lane
          LANE_CHANGE_COST -= slower;
        }

      }
      else{
        // NO CAR IN FRONT GIVE BONUS
        cout<<"BONOUS:500"<<endl;
        LANE_CHANGE_COST-=500;
      }




      return LANE_CHANGE_COST ;


    }
    else{
      return 0;
    }
}


} // namespace vc
#endif /* VEHICLE_H */
