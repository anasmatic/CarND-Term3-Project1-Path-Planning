#pragma once
//#ifndef TRAJECTORYGENERATOR_H
//#define TRAJECTORYGENERATOR_H

#include <iostream>
#include "json.hpp"
#include <vector>
#include <math.h>
enum State
{
	LANE_KEEP,
	LANE_CHANE_RIGHT,
	LANE_CHANE_LEFT
};
class TrajectoryGenerator{

public:
	State state = LANE_KEEP;
	TrajectoryGenerator();

	int plan_path(nlohmann::basic_json<> &sensor_fusion, double &car_s, double &car_d, int &previous_path_x_size, int &lane, bool &should_slowdown);
	int cost_of_LANEKEEP(nlohmann::basic_json<> &sensor_fusion, double &car_s, int &previous_path_x_size, int &lane, bool &should_slowdown);
	int cost_of_LANE_CHANGE_LEFT(nlohmann::basic_json<> &sensor_fusion, double &car_s, int &previous_path_x_size, int &lane);
	int cost_of_LANE_CHANGE_RIGHT(nlohmann::basic_json<> &sensor_fusion, double &car_s, int &previous_path_x_size, int &lane);
	virtual ~TrajectoryGenerator();
};


//#endif
