
#include <iostream>
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <typeinfo>

#ifndef VEHICLE_H
#define VEHICLE_H


// unnamed namespace only because the implementation is in this
// header file and we don't want to export symbols to the obj files
using namespace std;
namespace vc
{

// spline interpolation
class Vehicle
{
  public:
      int id;
      int lane;
      double d;
      double s;
      double velocity;
      double acceleration;
      bool is_initialized;
      double _time = 0.02;
      vector<bool> has_value_front={false,false,false};
      vector<bool> has_value_back={false,false,false};
      map<int,vc::Vehicle> closest_vehicles_front;
      map<int,vc::Vehicle> closest_vehicles_back;
      bool is_chaging_lane = false;
      int target_lane;
      bool collision_flag = false;
      double prev_available_space;
      double reducing_speed = false;

  public:
      // set default boundary condition to be zero curvature at both ends
      Vehicle(int id ,double s, double d, double velocity,double acceleration, bool is_chaging_lane, int target_lane) {
          this->is_chaging_lane = is_chaging_lane;
          this->d = d;
          this->s = s;
          this->velocity = velocity;
          this->acceleration = acceleration;
          this->id = id;
          this->is_initialized = true;

        //  double temp_lane = (this->d - 2)/4;
        if(this->is_chaging_lane){
          this->lane = target_lane;
        }
        else
        {
          if(this->d >=0 && this->d <=3.8){
            this->lane=0;
          }
          else if(this->d >3.8 && this->d <=7.8){
            this->lane=1;
          }
          else if(this->d >7.8){
            this->lane=2;
          }
          else{
            this->lane = -1;
          }
        }

        //set targte lane to current lane if none is given
        if(target_lane == -1)
        {
          this->target_lane = this->lane;
        }
        else{
          this->target_lane = target_lane;
        }

      }
      Vehicle() {
        this->is_initialized= false;
      }
      ~Vehicle(){}

      vector<vector<double>> getHorizon(int horizon);
      map<string, double> realize_state(string state, int prev_size);
      void adjust_speed(int prev_size);
      double calculate_cost( double available_space_front, double available_space_back, int lane);
      double check_for_car_back(int lane, int prev_size);
      double check_for_car_front(int lane, int prev_size);
};

vector<vector<double>> Vehicle::getHorizon(int horizon){
    double t = _time;
    vector<vector<double>> predictions;
    for( int i = 0; i < horizon; i++)
    {
      double s = this->s + this->velocity * t + this->acceleration * t * t / 2;
      double v = this-> velocity + this->acceleration * t;
      double a = this->acceleration;
      vector<double> temp_pred = {s, this->d, v ,a};
      predictions.push_back(temp_pred);
      t+=_time;
  	}
    return predictions;

  }

map<string, double> Vehicle::realize_state(string state, int prev_size){
  // find next car in this lane
  // find available room and proper velocity
    //s, d, v ,a
    map<string, double> result;

    if(state.compare("KL") == 0)
    {
      cout<<"Keep Lane:"<<endl;
      result["available_space_front"] = check_for_car_front(this->lane, prev_size);
      result["available_space_back"]= INFINITY;
      cout<<"available_space_front: "<<result["available_space_front"]<<endl;
      result["lane"] = this->lane;
      cout<<"--"<<endl;
    }
    else if(state.compare("LCL") == 0)
    {
      cout<<"Left Lane:"<<endl;
      result["available_space_front"] = check_for_car_front(this->lane-1, prev_size);
      cout<<"available_space_front: "<<result["available_space_front"]<<endl;
      result["available_space_back"] = check_for_car_back(this->lane-1, prev_size);
      result["lane"] = this->lane-1;
      cout<<"available_space_back: "<<result["available_space_back"]<<endl;
      cout<<"--"<<endl;
      //check for car in left
    }
    else if(state.compare("LCR") == 0)
    {
      cout<<"Right Lane:"<<endl;
      result["available_space_front"] = check_for_car_front(this->lane+1, prev_size);
      cout<<"available_space_front: "<<result["available_space_front"]<<endl;
      result["available_space_back"] = check_for_car_back(this->lane+1, prev_size);
      result["lane"] = this->lane+1;
      cout<<"available_space_back: "<<result["available_space_back"]<<endl;
      cout<<"--"<<endl;
      //check for car in right
    }
    return result;
}


void Vehicle::adjust_speed(int prev_size){

  // find all car in this lane
  //s, d, v ,a
  bool found = false;
  double buffer = 50;
  double critical_space = 10;
  double stop_space = 2;
  double velocity_decrese = 2;
  double velocity_increase = 2;
  double t = _time;

  if(this->collision_flag)
  {
    this->velocity  -=  (velocity_decrese + 0.5);
    cout<<"COLLISION SPEED DECREASE "<<velocity_decrese<<endl;
    reducing_speed= true;
  }
  else
  {
    double available_space_front = check_for_car_front(this->lane, prev_size);
    cout<<"Adjusting Speed available_space: "<<available_space_front<<endl;
    cout<<"reducing_speed: "<< reducing_speed<<endl;
    if(available_space_front - prev_available_space >10){
      if(available_space_front >30)
        velocity_decrese =0.5;
      else{
        velocity_decrese =1.5;
      }
    }
    else{
      if(reducing_speed)
        {
          velocity_increase =0.5;
        }
    }
    // if(this->is_chaging_lane){
    //   this->velocity  -= velocity_decrese;
    //   cout<<"SPEED DECREASE "<<velocity_decrese<<endl;
    // }
    if(!this->is_chaging_lane)
    {
      if(available_space_front < buffer){
          //if(this->velocity- velocity_decrese >= 0){
            this->velocity  -= velocity_decrese;
            cout<<"* SPEED DECREASE "<<velocity_decrese<<endl;
            reducing_speed= true;
          //}
      }
      else{
        if (this->velocity + velocity_increase < 49)
        {
          this->velocity  += velocity_increase;
          cout<<"** SPEED INCREASE "<<velocity_increase<<endl;
          reducing_speed = false;
        }
        else{
          cout<<"*** SPEED REMAIN "<<velocity_decrese<<endl;
          reducing_speed = false;
        }
      }
    }
    prev_available_space = available_space_front;
  }



  //reduce speed if changing lane
  // if(this->is_chaging_lane){
  //   this->velocity -= 0.5;
  // }


  // to avoid division by zero
  if(this->velocity  < 0){
    this->velocity  = 0.224;
  }

}

double Vehicle::check_for_car_back(int lane, int prev_size ){
  double t = _time;
  double available_space = INFINITY;
  if(this->has_value_back[lane]){
    Vehicle closest_vehicle = this->closest_vehicles_back[lane];
    cout<<"Vehicle BACK **DETECTED** id:"<<closest_vehicle.id<<" d: "<<closest_vehicle.d<<" s:"<<closest_vehicle.s<<" , v:"<<closest_vehicle.velocity<<endl;
    // double my_vehicle_next_s = this->s + prev_size*this->velocity*t;
    // available_space = my_vehicle_next_s - closest_vehicle.s;
    double other_vehicle_next_s = closest_vehicle.s + prev_size*closest_vehicle.velocity*t;
    available_space = this->s - other_vehicle_next_s;
  }
  return available_space;
}

double Vehicle::check_for_car_front(int lane, int prev_size ){
  double t = _time;
  double available_space = INFINITY;
  if(this->has_value_front[lane]){
    Vehicle closest_vehicle = this->closest_vehicles_front[lane];
    cout<<"Vehicle FRONT **DETECTED** id:"<<closest_vehicle.id<<" d: "<<closest_vehicle.d<<" s:"<<closest_vehicle.s<<" , v:"<<closest_vehicle.velocity<<endl;
    // double my_vehicle_next_s = this->s + prev_size*this->velocity*t;
    // available_space = closest_vehicle.s - my_vehicle_next_s;
    double other_vehicle_next_s = closest_vehicle.s + prev_size*closest_vehicle.velocity*t;
    available_space = other_vehicle_next_s - this->s;
  }
  return available_space;
}


double Vehicle::calculate_cost(double available_space_front, double available_space_back, int new_lane){

  double cost_speed=0;
  double cost_collision= 10000;
  double cost_jerk=0;
  double multiply_lane_change = 1000;
  double speed_limit = 50;
  double lane_change = 2000;

  double total_cost =0;
  if(this->lane != new_lane){
      total_cost+=50;
      if(available_space_back < 20){
        //huge penalty
          total_cost += 1000 * cost_collision;
          cout<<"COLLISION COST ADDED"<<endl;
      }
      total_cost+= cost_collision/available_space_back; //* exp(-available_space_back);

  }
  total_cost += cost_collision/available_space_front ;


  total_cost += sqrt(speed_limit - this->velocity);


  return total_cost;
}

} // namespace vc
#endif /* VEHICLE_H */
