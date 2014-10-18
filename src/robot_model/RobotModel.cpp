/*
 * RoboterState.hpp
 *
 *  Created on: Sep 23, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/robot_model/RobotModel.hpp"

namespace next_best_view {
	RobotModel::RobotModel() {}
	RobotModel::~RobotModel() {}

	void RobotModel::filterReachableOrientations(SimpleQuaternionCollectionPtr &orientationsPtr) {
		// Test every quaternion if its orientation is according to the robot's possibilities.
		// do a in-place selection of the right quaternions
		SimpleQuaternionCollection::iterator currentPosition = orientationsPtr->begin();
		SimpleQuaternionCollection::iterator currentEndPosition = orientationsPtr->end();

		// test each quaternion if it fits the possibilities.
		while (currentPosition != currentEndPosition) {
			SimpleQuaternion orientation = *currentPosition;
			bool reachable = this->isOrientationReachable(orientation);
			if (!reachable) {
				currentEndPosition--;
				std::swap(*currentPosition, *currentEndPosition);
				continue;
			}
			currentPosition++;
		}

		orientationsPtr->resize(std::distance(orientationsPtr->begin(), currentPosition));
	}
}
