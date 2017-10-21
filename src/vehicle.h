
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
      double d;
      double s;
      double velocity;
      double acceleration;
      bool is_initialized;
      double _time = 0.5;
  public:
      // set default boundary condition to be zero curvature at both ends
      Vehicle(int id ,double s, double d, double velocity,double acceleration) {
          this->d = d;
          this->s = s;
          this->velocity = velocity;
          this->acceleration = acceleration;
          this->id = id;
          this->is_initialized = true;
          // state = "CS";
          // max_acceleration = -1;
      }
      Vehicle() {
        this->is_initialized= false;
      }
      ~Vehicle(){}

      vector<vector<double>> getHorizon(int horizon);
      void realize_state(map<int, vector<vector<double>>> predictions, string state);
      void adjust_speed_for_lane(map<int, vector<vector<double>>> predictions, int lane);

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

void Vehicle::realize_state(map<int, vector<vector<double>>> predictions, string state){
  // find next car in this lane
  // find available room and proper velocity
    //s, d, v ,a

    int lane = int((this->d-2)/4);
    cout<<"Current Lane: "<< lane<<" d:"<<this->d<<" s:"<<this->s<<endl;
    if(state.compare("KL") == 0)
    {
      cout<<"*****"<<endl;
    	adjust_speed_for_lane(predictions, lane);
    }
    else if(state.compare("LCL") == 0)
    {

    }
    else if(state.compare("LCR") == 0)
    {

    }
    else if(state.compare("PLCL") == 0)
    {

    }
    else if(state.compare("PLCR") == 0)
    {

    }
}

void Vehicle::adjust_speed_for_lane(map<int, vector<vector<double>>> predictions, int lane){

  // find all car in this lane
  //s, d, v ,a
  map<string,double> closest_vehicle={{"v",INFINITY}, {"s",INFINITY}};
  bool found = false;
  double buffer = 40;
  double velocity_change = 1.5;

  for(map<int, vector<vector<double>>>::iterator it=predictions.begin(); it != predictions.end(); it++){
    vector<vector<double>>::iterator it2= it->second.begin();
    vector<double> other_vehicle =  it2[0];
    int other_vehicle_id = it->first;
    int other_vehicle_lane = int((other_vehicle[1]-2)/4);
    cout<<"Vehicle **DETECTED** id:"<<other_vehicle_id<<" lane: "<<other_vehicle_lane<<" s:"<<other_vehicle[0]<<" , v:"<<other_vehicle[2]<<endl;

    if(other_vehicle_lane == lane && other_vehicle[0]> this->s){
        // find the closes vehicle in front
        if(other_vehicle[0]  <= closest_vehicle["s"]){
          closest_vehicle["s"] = other_vehicle[0];
          closest_vehicle["v"] = other_vehicle[2];
          cout<<"closest_vehicle:"<<other_vehicle_id<<endl;
          found = true;
        }
    }
  }

  double t = _time;
  double vehicle_next_s = this->s + this->velocity*t;
  double available_space = buffer;
  if(found){
    available_space = closest_vehicle["s"] - vehicle_next_s;
  }
  cout<<"current s:"<<this->s <<" ,next s:"<<vehicle_next_s<<endl;
  cout<<"available_space: "<<available_space<<endl;
  cout<<"Speed Adjusted from:"<<this->velocity;
  if(available_space < buffer){
    if(this->velocity - velocity_change >= 0){
      this->velocity -= velocity_change;
    }
    else{
      this->velocity = 0;
    }
  }
  else if (this->velocity + velocity_change < 49){
    this->velocity += velocity_change;
  }


  if(this->velocity < 0.5){
    this->velocity += 0.5;
  }

  cout<<"Speed Adjusted to:"<<this->velocity<<endl;
}


} // namespace vc
#endif /* VEHICLE_H */
