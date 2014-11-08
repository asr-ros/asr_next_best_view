/*
 * RoboterState.hpp
 *
 *  Created on: Sep 23, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/robot_model/RobotModel.hpp"

namespace next_best_view {
	RobotModel::RobotModel() : CommonClass() {}
	RobotModel::~RobotModel() {}

	RobotStatePtr RobotModel::calculateRobotState(const SimpleVector3 &position, const SimpleQuaternion &orientation) {
		return this->calculateRobotState(mCurrentRobotState, position, orientation);
	}

	float RobotModel::getMovementCosts(const RobotStatePtr &targetRobotState) {
		return this->getMovementCosts(mCurrentRobotState, targetRobotState);
	}

	void RobotModel::setCurrentRobotState(const RobotStatePtr &currentRobotState) {
		mCurrentRobotState = currentRobotState;
	}

	RobotStatePtr RobotModel::getCurrentRobotState() {
		return mCurrentRobotState;
	}
}
