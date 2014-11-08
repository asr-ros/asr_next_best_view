/*
 * MILDRobotState.cpp
 *
 *  Created on: Nov 2, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/robot_model/impl/MILDRobotState.hpp"

namespace next_best_view {
	MILDRobotState::MILDRobotState() : RobotState(), pan(0), tilt(0), rotation(0), x(0), y(0) { }
	MILDRobotState::~MILDRobotState()  { }
}
