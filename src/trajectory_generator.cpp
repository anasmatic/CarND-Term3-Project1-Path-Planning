//create fsm
//keeplane
//	can be changeLaneR
//	can be changeLaneL
//then reset to keep lane after change lane action is done

//create cost functions
//pass params to cost function, return numeric value

//compare costs in the fsm 
//change lane value if cost of keeplane is more than changeLane R/L
#include <iostream>
#include "trajectory_generator.h"
#include "spline.h"
#include "json.hpp"
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <math.h>
using namespace std;

int MAX_COST = 10;
int AHEAD_SAFE_ZONE = 25;
int COLLISION_ZONE = 15;
//constructor
TrajectoryGenerator::TrajectoryGenerator() {}
//_________________________________________________________________________

//this function will decide the "state" of the car, and later manipulate "lane" param,
//two params only will be not remain the same within this function :
//1. lane : decides which lane the car should go move to.
//2. slow_down_to : forces car speed to not exceed this limit
//slow_down_to will be manipulated in helper COST functions
void TrajectoryGenerator::plan_path(nlohmann::basic_json<>& sensor_fusion, double &car_s, double &car_d, int & previous_path_x_size, int & lane, double &slow_down_to)
{
		
	if (TrajectoryGenerator::state == LANE_KEEP) {//current state is to keep lane
		//evaluate keep lane cost, (0 or 1)
		if (cost_of_LANEKEEP(sensor_fusion, car_s, previous_path_x_size, lane, slow_down_to) > 0)
		{//there is a cost for keeping lane, try to change it
			//save cost values for changing lane to left and right
			int cost_of_left = cost_of_LANE_CHANGE_LEFT(sensor_fusion, car_s, previous_path_x_size, lane, slow_down_to);
			int cost_of_right = cost_of_LANE_CHANGE_RIGHT(sensor_fusion, car_s, previous_path_x_size, lane, slow_down_to);
			//if cost exceeds max cost , do nothing, keep lane
			//otherwise
			if (cost_of_left < MAX_COST || cost_of_right < MAX_COST) {
				//chose the lower cost between right or left.
				//set the state to prepare lane change, don't change lane now, wait for next update
				if (cost_of_left > cost_of_right) {
					TrajectoryGenerator::state = PREPARE_LANE_CHANE_RIGHT;
				}
				else{
					TrajectoryGenerator::state = PREPARE_LANE_CHANE_LEFT;
				}
			}//end of (cost_of_left < 10 || cost_of_right < 10)
		}//else cost_of_LANEKEEP<=0 keep lane !
	//if current state is prepare change lane, do collision cost check, then decide to change lane or keep lane.
	}else if (TrajectoryGenerator::state == PREPARE_LANE_CHANE_RIGHT) {//if current state is prepare right lane
		lane += 1;//change lane to right
		if (cost_of_COLLISION(sensor_fusion, car_s, previous_path_x_size, lane) == 0) {//if no collision cost
			TrajectoryGenerator::state = LANE_CHANE_RIGHT;//change state, and keep lane new value
		}
		else {//but if there is collision cost
			lane -= 1;//revert lane change
			TrajectoryGenerator::state = LANE_KEEP;//set state to keep
		}
	}
	else if (TrajectoryGenerator::state == PREPARE_LANE_CHANE_LEFT) {//if current state is prepare left lane
		lane -= 1;//change lane to left
		if (cost_of_COLLISION(sensor_fusion, car_s, previous_path_x_size, lane) == 0) {//if no collision cost
			TrajectoryGenerator::state = LANE_CHANE_LEFT;//change state, and keep lane new value
		}
		else {//but if there is collision cost
			lane += 1;//revert lane change
			TrajectoryGenerator::state = LANE_KEEP;//set state to keep
		}
	}else {//if state is LANE_CHANE_LEFT or LANE_CHANE_RIGHT , check if change is done, then reset to LANE_KEEP
		if (car_d < (2 + 4 * lane + 2) && car_d >(2 + 4 * lane - 2)) {//if car is within new lane center
			TrajectoryGenerator::state = LANE_KEEP;//change state to keep
		}
	}
}

//------------------ cost functions -------------------
#pragma region CostFunctions
//this function checks cost of keeping the same lane, it will return 0 or 1, 
//it will also effect slow_down_to param, which controls car max speed
int TrajectoryGenerator::cost_of_LANEKEEP(nlohmann::basic_json<>& sensor_fusion, double & car_s, int & previous_path_x_size, int & lane, double &slow_down_to)
{
	//["sensor_fusion"] array of [0=ID, 1=x , 2=y , 3=xVelocity~m/s, 4=yVelocity~m/s, 5=s , 6=d ]
	for (int i = 0; i < sensor_fusion.size(); i++)//loop through cars around us
	{
		float d = sensor_fusion[i][6];//get d of a check_car in order to find its lane
		if (d < (2 + 4 * lane + 2) && d >(2 + 4 * lane - 2)) {//if check_car is in my lane
			double vx = sensor_fusion[i][3];
			double vy = sensor_fusion[i][4];
			double check_speed = sqrt(vx*vx + vy*vy);
			double check_car_s = sensor_fusion[i][5];
			check_car_s += ((double)previous_path_x_size*0.02*check_speed);
			//if check_car is ahead , and can approach safe zone
			if ((check_car_s > car_s) && (check_car_s - car_s) <= AHEAD_SAFE_ZONE) {
				slow_down_to = check_speed*2.24;//set max speed allowed to same as the ahead car
				return 1;//there is a cost for keeping lane
			}
		}
	}
	//other wise , allow max speed to be road max speed !
	slow_down_to = -1.0;//no slow down, speed to the legal limit !
	return 0;//there is no cost, keeps lane man !
}

//param lane is NOT going to change here
int TrajectoryGenerator::cost_of_LANE_CHANGE_LEFT(	nlohmann::basic_json<> &sensor_fusion, double &car_s, 
													int &previous_path_x_size, int &lane, double &slow_down_to) {
	//["sensor_fusion"] array of [0=ID, 1=x , 2=y , 3=xVelocity~m/s, 4=yVelocity~m/s, 5=s , 6=d ]
	
	int cost = 0;
	if (lane > 0)//can switch to left, because I'm not in the far left
	{
		int newlane = lane - 1;//create new value for lane, because we want to keep lane until we evaluate cost
		lane_change_cost(sensor_fusion, car_s, previous_path_x_size, newlane, cost, slow_down_to);//function will change cost value
	}
	else cost = MAX_COST;//can NOT change lane, because I'm at far left 
	return cost;
}

//param lane is NOT going to change here
int TrajectoryGenerator::cost_of_LANE_CHANGE_RIGHT(	nlohmann::basic_json<> &sensor_fusion, double &car_s, 
													int &previous_path_x_size, int &lane, double &my_car_vel) {
	
	int cost = 1;//always start with one, to prefer going left 
	if (lane < 2)//can switch to right, because I'm not in the far right
	{
		int newlane = lane + 1;//create new value for lane, because we want to keep lane until we evaluate cost
		lane_change_cost(sensor_fusion, car_s, previous_path_x_size, newlane, cost, my_car_vel);//function will change cost value
	}
	else cost = MAX_COST;//can NOT change lane, because I'm at far right
	return cost;
}

//common function used by both cost_of_LANE_CHANGE_RIGHT & cost_of_LANE_CHANGE_LEFT
//param: cost , if it was set by LeftLane function it will be 0, but if from rightLane function it will be initialized as 1
void TrajectoryGenerator::lane_change_cost(	nlohmann::basic_json<> &sensor_fusion, double &car_s, 
											int &previous_path_x_size, int &newlane, int &cost, double &my_car_vel) {

	for (int i = 0; i < sensor_fusion.size(); i++) {//loop through cars around us
		float d2 = sensor_fusion[i][6];//get d of a check_car in order to find its lane
		if (d2 < (2 + 4 * newlane + 2) && d2 >(2 + 4 * newlane - 2)) {//if check_car is in new lane
			double vx2 = sensor_fusion[i][3];
			double vy2 = sensor_fusion[i][4];
			double check_speed = sqrt(vx2*vx2 + vy2*vy2);
			double check_car_s = sensor_fusion[i][5];
			check_car_s += ((double)previous_path_x_size*0.02*check_speed);
			double delta_s = check_car_s - car_s;
			double abs_delta_s = abs(delta_s);//save also absolute delta value
			if (abs_delta_s < 60) {//if check_car is 60 units far away (check back and front)
				if (delta_s > 0) {//check_car in front
					if (abs_delta_s < 60) {//check_car is this range ?
						if (check_speed*2.24 < my_car_vel) {//if it's slower than me, cost++
							cost++;//increase cost
						} 

						for (int i = 55; i >= 10; i-=5)//loop for every 5 units of distance
						{
							if (abs_delta_s < i)//the closer the check_car, the more is the cost
								cost++;
							
							if (i<20)//double it if less than 20, so it will NEVER pass if car is closer than 20
								cost++;
						}
					}
				}
				else {//car coming from behind
					if (abs_delta_s < 10) {//TOO CLOSE, never allow lane change
						cost += MAX_COST;
					}
					else if (abs_delta_s < 35) {//not so close ?
						//predict check_car s after 2 seconds
						double check_car_new_s = check_car_s+(((double)previous_path_x_size*0.04*check_speed));
						//predict my s after 1 seconds
						double my_car_new_s = car_s + (((double)previous_path_x_size*0.02*my_car_vel/2.24));
						if (check_car_new_s >= my_car_new_s) { //if check_car may collide with me in the future
							cost += MAX_COST;//NEVER ALLOW LANE CHANGE
						}
					}
				}//end of else
			}//end of if(abs_delta_s < 30)
			//if cost returned is less than 10, then it is some how safe to change lane
		}
	}
}

//Although Lane Change Cost functions calculate in an indirect way the cost of collision , but it is better to test it separately 
//returns 0 or 1 , if returned 1 then it means cost is hight for the move
int TrajectoryGenerator::cost_of_COLLISION(nlohmann::basic_json<>& sensor_fusion, double & car_s, int & previous_path_x_size, int lane)
{
	//["sensor_fusion"] array of [0=ID, 1=x , 2=y , 3=xVelocity~m/s, 4=yVelocity~m/s, 5=s , 6=d ]
	for (int i = 0; i < sensor_fusion.size(); i++)//loop through cars around us
	{
		float d = sensor_fusion[i][6];//get d of a check_car in order to find its lane
		if (d < (2 + 4 * lane + 2) && d >(2 + 4 * lane - 2)) {//if check_car is in new lane
			double vx = sensor_fusion[i][3];
			double vy = sensor_fusion[i][4];
			double check_speed = sqrt(vx*vx + vy*vy);
			double check_car_s = sensor_fusion[i][5];
			check_car_s += ((double)previous_path_x_size*0.02*check_speed);
			double abs_delta_s = abs(check_car_s - car_s);//save also absolute delta value
			if (abs_delta_s <= COLLISION_ZONE)//check_car within collision zone :
				return 1;//Cost is hight !

		}
	}
	return 0;//no cost for next move
}

#pragma endregion


//_________________________________________________________________________
//destructor
TrajectoryGenerator::~TrajectoryGenerator() {}