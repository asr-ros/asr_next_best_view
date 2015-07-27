/*
 * MILDRobotState.cpp
 *
 *  Created on: Nov 2, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/robot_model/impl/MILDRobotState.hpp"

namespace next_best_view {
    MILDRobotState::MILDRobotState(float pan, float tilt, float rotation, float x, float y): RobotState(), pan(pan), tilt(tilt), rotation(rotation), x(x), y(y) { }
	MILDRobotState::MILDRobotState() : RobotState(), pan(0), tilt(0), rotation(0), x(0), y(0) { }
	MILDRobotState::~MILDRobotState()  { }
}
