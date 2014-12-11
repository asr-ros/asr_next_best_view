/*
 * TypeHelper.cpp
 *
 *  Created on: Oct 18, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/helper/TypeHelper.hpp"

namespace next_best_view {
	geometry_msgs::Point TypeHelper::getPointMSG(const SimpleVector3 &vector) {
		geometry_msgs::Point point;
		point.x = vector[0];
		point.y = vector[1];
		point.z = vector[2];
		return point;
	}

	SimpleVector3 TypeHelper::getSimpleVector3(const geometry_msgs::Pose &pose) {
		return TypeHelper::getSimpleVector3(pose.position);
	}

	SimpleVector3 TypeHelper::getSimpleVector3(const geometry_msgs::Point &point) {
		return SimpleVector3(point.x, point.y, point.z);
	}


	SimpleQuaternion TypeHelper::getSimpleQuaternion(const geometry_msgs::Pose &pose) {
		return TypeHelper::getSimpleQuaternion(pose.orientation);
	}

	SimpleQuaternion TypeHelper::getSimpleQuaternion(const geometry_msgs::Quaternion &quaternion) {
		return SimpleQuaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
	}
}
