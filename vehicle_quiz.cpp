#include <iostream>
#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <typeinfo>
/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int lane, int s, int v, int a) {

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    state = "CS";
    max_acceleration = -1;

}
Vehicle::Vehicle(int _lane, int _s, int _v, int _a, string _state, int _max_a){
  this->lane= _lane;
  this->s=_s ;
  this->v = _v;
  this->a = _a;
  state = _state;
  max_acceleration = _max_a;
}

Vehicle::~Vehicle() {}

// TODO - Implement this method.
void Vehicle::update_state(map<int,vector < vector<int> > > predictions) {
	/*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is
       traffic in front of it, in which case it will slow down.


    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
     - The vehicle will find the nearest vehicle in the adjacent lane which is
       BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
      3 : [
        {"s" : 4, "lane": 0},
        {"s" : 6, "lane": 0},
        {"s" : 8, "lane": 0},
        {"s" : 10, "lane": 0},
      ]
    }

    */
    //   map<int, vector<vector<int> > >::iterator it = predictions.begin();
    // vector<vector<vector<int> > > in_front;
    // while(it != predictions.end())
    // {
          //   [
    //     {"s" : 4, "lane": 0},
    //     {"s" : 6, "lane": 0},
    //     {"s" : 8, "lane": 0},
    //     {"s" : 10, "lane": 0},
    //   ]
    //   cout<<temp<<endl;

    //   for(temp_car_it = temp_car.begin(); temp_car_it != temp_car.end(); temp_car_it++){
    //      cout<<"s:"<<temp_car_it[0][1]<<", lane:"<<temp_car_it[0][0]<<endl;

    //   }
    // 	int v_id = it->first;
    // 	cout<<(it->second).size()<<endl;

    //     it++;
    // }
    state = "KL"; // this is an example of how you change state.
    double cost_KL = 0, cost_LCL=0, cost_LCR=0;


    // int left_lane = this->lane -1;
    // int right_lane = this->lane +1;
    // cout<<"Left Lane: "<<left_lane<<endl;
    // //valid left lane
    // if(left_lane ==1 || left_lane ==0)
    // {
    //     int current_lane = this->lane;
    //     // this->lane = left_lane;
    //     map<int,vector < vector<int> > > left_lane_predictions = get_predictions_by_lane(predictions,this->lane-1);
    //     cout<<this->calculate_cost_left(left_lane_predictions)<<endl;
    //     // this->lane = current_lane;
    // }
    string next_state = get_next_state(predictions);
    this->state = next_state;
}

map<int,vector < vector<int> > > Vehicle::deep_copy_predictions(map<int,vector < vector<int> > > predictions){
  map<int,vector < vector<int> > > predictions_copy;
  map<int, vector<vector<int> > >::iterator it = predictions.begin();
  while(it != predictions.end())
  {
      auto temp_car = it->second;
      int car_id = it->first;
      vector<vector<int>>::iterator temp_car_it;
      int prediction_counter = 0;
      vector<vector<int>> car_predictions ;
      for(temp_car_it = temp_car.begin(); temp_car_it != temp_car.end(); temp_car_it++){

         vector<int> lane_s = {temp_car_it[0][0], temp_car_it[0][1]};
         car_predictions.push_back(lane_s);
         prediction_counter++;
         if (prediction_counter == 10){
          predictions_copy[car_id] = car_predictions;
          car_predictions.clear();
          prediction_counter=0;
         }
      }
      ++it;
  }
  return predictions_copy;
}

string  Vehicle::get_next_state(map<int,vector<vector<int>>> predictions){
    string best_state = state;
    vector<string> available_states;
    map<string,int> costs;
    if(this->lane == 0){
      available_states = {"KL", "LCL", "PLCL"};
    }
    else if (this->lane == 3){
      available_states= {"KL", "LCR",  "PLCR"};
    }
    else{
        available_states= {"KL", "LCL", "LCR", "PLCL", "PLCR"};
    }

   double lowest_cost = 1000000;
   for(vector<string>::iterator state_it = available_states.begin(); state_it != available_states.end(); state_it++){

        map<int,vector<vector<int>>> predictions_copy = deep_copy_predictions(predictions);
        vector<Vehicle*> trajectory = get_trajectory(state_it[0], predictions_copy);
        double temp_cost = calculate_cost(trajectory,predictions );
        if (temp_cost < lowest_cost){
            lowest_cost = temp_cost;
            best_state = state_it[0];

        }
    }
    cout<<"best score:"<<lowest_cost<<", "<<best_state<<endl;
    return best_state;
}


vector<Vehicle*> Vehicle::get_trajectory(string _state, map<int,vector<vector<int>>> predictions){
  int horizon = 5;
  vector<Vehicle*> trajectory;
  trajectory.push_back(new Vehicle(this->lane, this->s, this->v, this->a, this->state, this->max_acceleration));
  for(int i=0; i<horizon ; i++){
    Vehicle* current_state= trajectory[0];
    this->state = _state;
    this->realize_state(predictions);
    if(this->lane >=0 && this->lane <=3){
      this->increment(i);
      trajectory.push_back(new Vehicle(this->lane, this->s, this->v, this->a, this->state, this->max_acceleration));
    //   cout<<"realized:"<<this->s<<","<<this->lane<<","<<this->state<<","<<this->v<<endl;

      map<int, vector<vector<int> > >::iterator it = predictions.begin();
      while(it != predictions.end())
      {
        // cout<<"item to pop"<<it->second.begin()[0][0]<<","<< it->second.begin()[0][1]<<endl;
        it->second.erase( it->second.begin(), it->second.begin()+2 );
        // cout<<"item after pop"<<it->second.begin()[0][0]<<","<< it->second.begin()[0][1]<<endl;

        it++;
      }
    }
    this->restore_state(current_state);
  }
  return trajectory;
}
void Vehicle::restore_state(Vehicle* prev_state){
  this->lane= prev_state->lane;
  this->s=prev_state->s ;
  this->v = prev_state->v;
  this->a = prev_state->a;
  this->state = prev_state->state;
  this->max_acceleration = prev_state->max_acceleration;
}

map<string,int> Vehicle::get_helper(vector<Vehicle*> trajectory, map<int,vector<vector<int>>> predictions){
  map<string,int> helper_data;
  vector<Vehicle*>::iterator it = trajectory.begin();
  Vehicle* current_state = it[0];
  Vehicle* first = it[1];
  Vehicle* last =it[4];
  helper_data["collision_result"] = 0;
  helper_data["collision_time"] = -1;
  helper_data["proposed_lane"] = first->lane;
  helper_data["avg_speed"] = (last->s - current_state->s )/ 5;
  helper_data["s_from_goal"]= abs(last->s - this->goal_s);
  helper_data["lane_from_goal"]= abs(last->lane - this->goal_lane);
  map<int,vector<vector<int>>> lane_predictions = get_predictions_by_lane(predictions, helper_data["proposed_lane"]);
  map<int, vector<vector<int> > >::iterator it_lane = lane_predictions.begin();
  while(it_lane != lane_predictions.end())
  {
    // cout<<"item to pop"<<it->second.begin()[0][0]<<","<< it->second.begin()[0][1]<<endl;
    //int lane, int s, int v, int a
    int v_id = it_lane->first;
    int v_1 = it_lane->second[1][1] - it_lane->second[0][1];
    int v_2 = it_lane->second[1][1] - it_lane->second[2][1];
    int a = v_2 - v_1;
    Vehicle other_v = Vehicle(helper_data["proposed_lane"], it_lane->second[0][1], v_1, a);
    Vehicle::collider collider_temp = will_collide_with(other_v,2);
    if (collider_temp.collision == true){
        helper_data["collision_result"] = 1;
        helper_data["collision_time"] = collider_temp.time;
        helper_data["collision_with_car"] = v_id;
    }
    it_lane++;
  }

  return helper_data;
}

double Vehicle::calculate_cost(vector<Vehicle*> trajectory,map<int,vector < vector<int> > > predictions){
    double total_cost =0;
    map<string,int> trajectory_data = get_helper(trajectory, predictions);
    cout<<"Cost for Lane:"<<trajectory_data["proposed_lane"]<<endl;
    total_cost += cost_collision(trajectory_data);
    total_cost += cost_lane_change(trajectory_data);
    total_cost += cost_speed(trajectory_data);
    total_cost += cost_distance_from_goal(trajectory_data);
    cout<<"total:"<<total_cost<<endl;
    return total_cost;
}

double Vehicle::cost_collision( map<string,int> trajectory_data){
  double cost = pow(10,6);
  if(trajectory_data["collision_result"] == 1){
      double result =  cost * exp(-trajectory_data["collision_time"]);
      cout<<result<<", collision with:"<<trajectory_data["collision_with_car"]<<endl;
      return result;
  }
  return 0;
}

double Vehicle::cost_lane_change( map<string,int> trajectory_data){
  double cost = pow(10,4);
  if(trajectory_data["proposed_lane"] < this->lane){
      return -cost;
  }
  return cost;
}

double Vehicle::cost_speed( map<string,int> trajectory_data){
  double cost = pow(10,2);
  int avg_speed = trajectory_data["avg_speed"];
  int diff = target_speed - avg_speed;
  if(diff < -5 or diff > 5){
      // high speed should slow down, low speed should increase
      return cost*abs(diff);
  }
  return 0;
}

double Vehicle::cost_distance_from_goal( map<string,int> trajectory_data){
  double cost = pow(10,5);
  double time_to_goal = trajectory_data["s_from_goal"] / trajectory_data["avg_speed"] ;
  double result = cost*trajectory_data["lanes_from_goal"] / time_to_goal;
  return result;
}

map<int,vector < vector<int> > > Vehicle::get_predictions_by_lane(map<int,vector < vector<int> > > predictions, int lane_id) {
	map<int, vector<vector<int> > >::iterator it;
	map<int,vector < vector<int> > > lane_predictions;
	for(it =  predictions.begin(); it != predictions.end(); it++){
	    auto temp_car = it->second;
	    int car_id = it->first;
	    vector<vector<int>>::iterator temp_car_it;
	    temp_car_it = temp_car.begin();
        if( temp_car_it[0][0] == lane_id && car_id != -1){
        {
            int i =0;
            while(i<10)
            {
                lane_predictions[it->first].push_back( temp_car_it[i]);
                i++;
            }
        }

       }
	}
    return lane_predictions;
}

void Vehicle::configure(vector<int> road_data) {
	/*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle.
    */
    target_speed = road_data[0];
    lanes_available = road_data[1];
    goal_s = road_data[2];
    goal_lane = road_data[3];
    max_acceleration = road_data[4];
}

string Vehicle::display() {

	ostringstream oss;

	oss << "s:    " << this->s << "\n";
    oss << "lane: " << this->lane << "\n";
    oss << "v:    " << this->v << "\n";
    oss << "a:    " << this->a << "\n";

    return oss.str();
}

void Vehicle::increment(int dt = 1) {

	this->s += this->v * dt;
    this->v += this->a * dt;
}

vector<int> Vehicle::state_at(int t) {

	/*
    Predicts state of vehicle in t seconds (assuming constant acceleration)
    */
    int s = this->s + this->v * t + this->a * t * t / 2;
    int v = this->v + this->a * t;
    return {this->lane, s, v, this->a};
}

bool Vehicle::collides_with(Vehicle other, int at_time) {

	/*
    Simple collision detection.
    */
    vector<int> check1 = state_at(at_time);
    vector<int> check2 = other.state_at(at_time);
    return (check1[0] == check2[0]) && (abs(check1[1]-check2[1]) <= L);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {

	Vehicle::collider collider_temp;
	collider_temp.collision = false;
	collider_temp.time = -1;

	for (int t = 0; t < timesteps+1; t++)
	{
      	if( collides_with(other, t) )
      	{
			collider_temp.collision = true;
			collider_temp.time = t;
        	return collider_temp;
    	}
	}

	return collider_temp;
}

void Vehicle::realize_state(map<int,vector < vector<int> > > predictions) {

	/*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
    string state = this->state;
    if(state.compare("CS") == 0)
    {
    	realize_constant_speed();
    }
    else if(state.compare("KL") == 0)
    {
    	realize_keep_lane(predictions);
    }
    else if(state.compare("LCL") == 0)
    {
    	realize_lane_change(predictions, "L");
    }
    else if(state.compare("LCR") == 0)
    {
    	realize_lane_change(predictions, "R");
    }
    else if(state.compare("PLCL") == 0)
    {
    	realize_prep_lane_change(predictions, "L");
    }
    else if(state.compare("PLCR") == 0)
    {
    	realize_prep_lane_change(predictions, "R");
    }

}

void Vehicle::realize_constant_speed() {
	a = 0;
}

int Vehicle::_max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s) {

	int delta_v_til_target = target_speed - v;
    int max_acc = min(max_acceleration, delta_v_til_target);

    map<int, vector<vector<int> > >::iterator it = predictions.begin();
    vector<vector<vector<int> > > in_front;
    while(it != predictions.end())
    {

    	int v_id = it->first;

        vector<vector<int> > v = it->second;

        if((v[0][0] == lane) && (v[0][1] > s))
        {
        	in_front.push_back(v);

        }
        it++;
    }

    if(in_front.size() > 0)
    {
    	int min_s = 1000;
    	vector<vector<int>> leading = {};
    	for(int i = 0; i < in_front.size(); i++)
    	{
    		if((in_front[i][0][1]-s) < min_s)
    		{
    			min_s = (in_front[i][0][1]-s);
    			leading = in_front[i];
    		}
    	}

    	int next_pos = leading[1][1];
    	int my_next = s + this->v;
    	int separation_next = next_pos - my_next;
    	int available_room = separation_next - preferred_buffer;
    	max_acc = min(max_acc, available_room);
    }

    return max_acc;

}

void Vehicle::realize_keep_lane(map<int,vector< vector<int> > > predictions) {
	this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int,vector< vector<int> > > predictions, string direction) {
	int delta = -1;
    if (direction.compare("L") == 0)
    {
    	delta = 1;
    }
    this->lane += delta;
    int lane = this->lane;
    int s = this->s;
    this->a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_prep_lane_change(map<int,vector<vector<int> > > predictions, string direction) {
	int delta = -1;
    if (direction.compare("L") == 0)
    {
    	delta = 1;
    }
    int lane = this->lane + delta;

    map<int, vector<vector<int> > >::iterator it = predictions.begin();
    vector<vector<vector<int> > > at_behind;
    while(it != predictions.end())
    {
    	int v_id = it->first;
        vector<vector<int> > v = it->second;

        if((v[0][0] == lane) && (v[0][1] <= this->s))
        {
        	at_behind.push_back(v);

        }
        it++;
    }
    if(at_behind.size() > 0)
    {

    	int max_s = -1000;
    	vector<vector<int> > nearest_behind = {};
    	for(int i = 0; i < at_behind.size(); i++)
    	{
    		if((at_behind[i][0][1]) > max_s)
    		{
    			max_s = at_behind[i][0][1];
    			nearest_behind = at_behind[i];
    		}
    	}
    	int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
    	int delta_v = this->v - target_vel;
    	int delta_s = this->s - nearest_behind[0][1];
    	if(delta_v != 0)
    	{

    		int time = -2 * delta_s/delta_v;
    		int a;
    		if (time == 0)
    		{
    			a = this->a;
    		}
    		else
    		{
    			a = delta_v/time;
    		}
    		if(a > this->max_acceleration)
    		{
    			a = this->max_acceleration;
    		}
    		if(a < -this->max_acceleration)
    		{
    			a = -this->max_acceleration;
    		}
    		this->a = a;
    	}
    	else
    	{
    		int my_min_acc = max(-this->max_acceleration,-delta_s);
    		this->a = my_min_acc;
    	}

    }

}

vector<vector<int> > Vehicle::generate_predictions(int horizon = 10) {

	vector<vector<int> > predictions;
    for( int i = 0; i < horizon; i++)
    {
      vector<int> check1 = state_at(i);
      vector<int> lane_s = {check1[0], check1[1]};
      predictions.push_back(lane_s);
  	}
    return predictions;

}
