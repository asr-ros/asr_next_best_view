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

    float RobotModel::getBase_TranslationalMovementCosts(const RobotStatePtr &targetRobotState) {
        return this->getBase_TranslationalMovementCosts(mCurrentRobotState, targetRobotState);
	}

    float RobotModel::getPTU_PanMovementCosts(const RobotStatePtr &targetRobotState) {
        return this->getPTU_PanMovementCosts(mCurrentRobotState, targetRobotState);
    }

    float RobotModel::getPTU_TiltMovementCosts(const RobotStatePtr &targetRobotState) {
        return this->getPTU_TiltMovementCosts(mCurrentRobotState, targetRobotState);
    }

    float RobotModel::getBase_RotationalMovementCosts(const RobotStatePtr &targetRobotState) {
        return this->getBase_RotationalMovementCosts(mCurrentRobotState, targetRobotState);
    }

	void RobotModel::setCurrentRobotState(const RobotStatePtr &currentRobotState) {
		mCurrentRobotState = currentRobotState;
	}

	RobotStatePtr RobotModel::getCurrentRobotState() {
		return mCurrentRobotState;
	}
}
