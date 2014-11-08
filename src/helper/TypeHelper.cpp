/*
 * TypeHelper.cpp
 *
 *  Created on: Oct 18, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/helper/TypeHelper.hpp"

namespace next_best_view {
	geometry_msgs::Point TypeHelper::getGeometryMsgsPoint(const SimpleVector3 &vector) {
		geometry_msgs::Point point;
		point.x = vector[0];
		point.y = vector[1];
		point.z = vector[2];
		return point;
	}

	SimpleVector3 TypeHelper::getSimpleVector3(const geometry_msgs::Point &point) {
		return SimpleVector3(point.x, point.y, point.z);
	}
}
