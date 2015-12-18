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

    geometry_msgs::Quaternion TypeHelper::getQuaternionMSG(const SimpleQuaternion &quaternion) {
        geometry_msgs::Quaternion result;

        result.w = quaternion.w();
        result.x = quaternion.x();
        result.y = quaternion.y();
        result.z = quaternion.z();

        return result;
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

    geometry_msgs::Vector3 TypeHelper::getVector3(const vector<double> &vector) {
        geometry_msgs::Vector3 result;

        result.x = vector[0];
        result.y = vector[1];
        result.z = vector[2];

        return result;
    }

    std_msgs::ColorRGBA TypeHelper::getColor(const vector<double> &vector) {
        std_msgs::ColorRGBA result;

        result.a = vector[3];
        result.r = vector[0];
        result.g = vector[1];
        result.b = vector[2];

        return result;
    }
}
