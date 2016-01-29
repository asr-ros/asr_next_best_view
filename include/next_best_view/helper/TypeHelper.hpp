/*
 * TypeHelper.hpp
 *
 *  Created on: Oct 18, 2014
 *      Author: ralfschleicher
 */

#ifndef TYPEHELPER_HPP_
#define TYPEHELPER_HPP_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <vector>
#include "typedef.hpp"

namespace next_best_view {
	class TypeHelper {
	public:
		static geometry_msgs::Point getPointMSG(const SimpleVector3 &vector);
        static geometry_msgs::Quaternion getQuaternionMSG(const SimpleQuaternion &quaternion);
		static SimpleVector3 getSimpleVector3(const geometry_msgs::Pose &pose);
		static SimpleVector3 getSimpleVector3(const geometry_msgs::Point &point);
        static SimpleVector3 getSimpleVector3(const std::vector<double> &vector);
        static SimpleVector4 getSimpleVector4(const std::vector<double> &vector);
        static SimpleVector4 getSimpleVector4(const std_msgs::ColorRGBA &color);
		static SimpleQuaternion getSimpleQuaternion(const geometry_msgs::Pose &pose);
		static SimpleQuaternion getSimpleQuaternion(const geometry_msgs::Quaternion &quaternion);
        static geometry_msgs::Vector3 getVector3(const SimpleVector3 &vector);
        static std_msgs::ColorRGBA getColor(const SimpleVector4 &vector);
	};
}


#endif /* TYPEHELPER_HPP_ */
