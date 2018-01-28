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

//constructor
TrajectoryGenerator::TrajectoryGenerator() {}
//_________________________________________________________________________
int TrajectoryGenerator::plan_path(nlohmann::basic_json<>& sensor_fusion, double &car_s, double &car_d, int & previous_path_x_size, int & lane, bool & should_slowdown)
{

	if (TrajectoryGenerator::state == LANE_KEEP) {
		//TODO: if cost of keep lane < cost of lane change , then keep lane 
		if (cost_of_LANEKEEP(sensor_fusion, car_s, previous_path_x_size, lane, should_slowdown) > 0)
		{//there is a cost for keeping lane
			int cost_of_left = cost_of_LANE_CHANGE_LEFT(sensor_fusion, car_s, previous_path_x_size, lane);
			int cost_of_right = cost_of_LANE_CHANGE_RIGHT(sensor_fusion, car_s, previous_path_x_size, lane);
			if (cost_of_left < 10 && cost_of_right < 10) {
				if (cost_of_left > cost_of_right) {
					TrajectoryGenerator::state = LANE_CHANE_RIGHT;
					lane += 1;
					cout << endl;
					cout << "RRRRRRRRRRRRRRrrr" << endl;
				}
				else {
					TrajectoryGenerator::state = LANE_CHANE_LEFT;
					lane-=1;
					cout << endl;
					cout << "LLLLLLLLLEEEEEFFFFFTTTTTTTT" << endl;
				}
			}
		}//else keep lane !
		cout<<"k "<< should_slowdown<<" ? ,";
		 //TODO: if cost of lane change is better, then change lane
		 //TOOD: decide which lane to go to, 
	}
	else {
		cout << "~" ;
		if (car_d < (2 + 4 * lane + 2) && car_d >(2 + 4 * lane - 2)) {
			TrajectoryGenerator::state = LANE_KEEP;
			cout << "~~~~~~RESET 2 KEEP~~~~~~" << endl;
		}
	}
}

//------------------ cost functions -------------------
#pragma region CostFunctions
int TrajectoryGenerator::cost_of_LANEKEEP(nlohmann::basic_json<>& sensor_fusion, double & car_s, int & previous_path_x_size, int & lane, bool & should_slowdown)
{
	//["sensor_fusion"] [0=ID, 1=x , 2=y , 3=xVelocity~m/s, 4=yVelocity~m/s, 5=s , 6=d ]
	for (int i = 0; i < sensor_fusion.size(); i++)
	{
		float d = sensor_fusion[i][6];
		//if another car in my lane
		if (d < (2 + 4 * lane + 2) && d >(2 + 4 * lane - 2)) {
			double vx = sensor_fusion[i][3];
			double vy = sensor_fusion[i][4];
			double check_speed = sqrt(vx*vx + vy*vy);
			double check_car_s = sensor_fusion[i][5];

			check_car_s += ((double)previous_path_x_size*0.02*check_speed);

			if ((check_car_s > car_s) && (check_car_s - car_s) < 30) {//TODO: && check_speed < mycar speed
				should_slowdown = true;
				return 1;
				
				double check_xV = sensor_fusion[i][5];
				cout << "how slow?" << check_speed << " , vx "<< vx << endl;

				//switch lane
				//todo: fsm & cost function
				if (lane > 0)
				{
					int newlane = lane - 1;
					bool canswitch = true;
					for (int i = 0; i < sensor_fusion.size(); i++) {
						float d2 = sensor_fusion[i][6];
						if (d2 < (2 + 4 * newlane + 2) && d2 >(2 + 4 * newlane - 2)) {
							double vx2 = sensor_fusion[i][3];
							double vy2 = sensor_fusion[i][4];
							double check_speed2 = sqrt(vx2*vx2 + vy2*vy2);
							double check_car_s2 = sensor_fusion[i][5];

							check_car_s2 += ((double)previous_path_x_size*0.02*check_speed2);
							//cout << "<30? " << (abs(check_car_s2 - car_s) < 30) << endl;
							if (abs(check_car_s2 - car_s) < 30) {
								canswitch = false;
								break;
							}

						}
					}
					if (canswitch)
						lane = newlane;
				}
			}
		}
	}
	return 0;
}

int TrajectoryGenerator::cost_of_LANE_CHANGE_LEFT(nlohmann::basic_json<> &sensor_fusion, double &car_s, int &previous_path_x_size, int &lane) {
	//["sensor_fusion"] [0=ID, 1=x , 2=y , 3=xVelocity~m/s, 4=yVelocity~m/s, 5=s , 6=d ]
		//switch lane
		//todo: fsm & cost function
	int cost = 0;
	if (lane > 0)//can switch to left, because I'm not in the far left
	{
		int newlane = lane - 1;
		bool canswitch = true;
		for (int i = 0; i < sensor_fusion.size(); i++) {
			float d2 = sensor_fusion[i][6];//get near car lane
			if (d2 < (2 + 4 * newlane + 2) && d2 >(2 + 4 * newlane - 2)) {//if near car is in left lane to me
				double vx2 = sensor_fusion[i][3];
				double vy2 = sensor_fusion[i][4];
				double check_speed2 = sqrt(vx2*vx2 + vy2*vy2);
				double check_car_s2 = sensor_fusion[i][5];

				check_car_s2 += ((double)previous_path_x_size*0.02*check_speed2);
				//cout << "<30? " << (abs(check_car_s2 - car_s) < 30) << endl;
				if (abs(check_car_s2 - car_s) < 20) {
					canswitch = false;
					cost = 10;
					cout <<" Left car speed: " << check_speed2*2.24 << endl;
					break;
				}

			}
		}
		if (canswitch)
			lane = newlane;
	}
	else cost = 10;
	return cost;
}

int TrajectoryGenerator::cost_of_LANE_CHANGE_RIGHT(nlohmann::basic_json<> &sensor_fusion, double &car_s, int &previous_path_x_size, int &lane) {
	
	int cost = 1;//always start with one, to prefare going left 
	if (lane < 2)//can switch to left, because I'm not in the far left
	{
		int newlane = lane + 1;
		bool canswitch = true;
		for (int i = 0; i < sensor_fusion.size(); i++) {
			float d2 = sensor_fusion[i][6];//get near car lane
			if (d2 < (2 + 4 * newlane + 2) && d2 >(2 + 4 * newlane - 2)) {//if near car is in left lane to me
				double vx2 = sensor_fusion[i][3];
				double vy2 = sensor_fusion[i][4];
				double check_speed2 = sqrt(vx2*vx2 + vy2*vy2);
				double check_car_s2 = sensor_fusion[i][5];

				check_car_s2 += ((double)previous_path_x_size*0.02*check_speed2);
				//cout << "<30? " << (abs(check_car_s2 - car_s) < 30) << endl;
				if (abs(check_car_s2 - car_s) < 20) {
					canswitch = false;
					cost = 10;
					cout << " RIGHT car speed: " << check_speed2*2.24 << endl;
					break;
				}

			}
		}
		if (canswitch)
			lane = newlane;
	}
	else cost = 10;
	return cost;
}
#pragma endregion


//_________________________________________________________________________
//destructor
TrajectoryGenerator::~TrajectoryGenerator() {}