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

    SimpleVector3 TypeHelper::getSimpleVector3(const std::vector<double> &vector) {
        return SimpleVector3(vector.at(0), vector.at(1), vector.at(2));
    }


    SimpleVector3 TypeHelper::getSimpleVector3(const aiVector3D &vector)
    {
        return SimpleVector3(vector.x, vector.y, vector.z);
    }

    SimpleVector4 TypeHelper::getSimpleVector4(const std::vector<double> &vector) {
        return SimpleVector4(vector[0], vector[1], vector[2], vector[3]);
    }

    SimpleVector4 TypeHelper::getSimpleVector4(const std_msgs::ColorRGBA &color) {
        return SimpleVector4(color.r, color.g, color.b, color.a);
    }

	SimpleQuaternion TypeHelper::getSimpleQuaternion(const geometry_msgs::Pose &pose) {
		return TypeHelper::getSimpleQuaternion(pose.orientation);
	}

	SimpleQuaternion TypeHelper::getSimpleQuaternion(const geometry_msgs::Quaternion &quaternion) {
        return SimpleQuaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    }

    SimpleQuaternion TypeHelper::getSimpleQuaternion(const std::vector<double> &vector)
    {
        return SimpleQuaternion(vector.at(0), vector.at(1), vector.at(2), vector.at(3));
    }

    geometry_msgs::Vector3 TypeHelper::getVector3(const SimpleVector3 &vector) {
        geometry_msgs::Vector3 result;

        result.x = vector[0];
        result.y = vector[1];
        result.z = vector[2];

        return result;
    }

    aiVector3D TypeHelper::getAiVector3D(const SimpleVector3 &vector)
    {
        return aiVector3D(vector[0], vector[1], vector[2]);
    }

    std_msgs::ColorRGBA TypeHelper::getColor(const SimpleVector4 &vector) {
        std_msgs::ColorRGBA result;

        result.r = vector[0];
        result.g = vector[1];
        result.b = vector[2];
        result.a = vector[3];

        return result;
    }

}
