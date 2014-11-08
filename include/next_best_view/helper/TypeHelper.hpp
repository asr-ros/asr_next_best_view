/*
 * TypeHelper.hpp
 *
 *  Created on: Oct 18, 2014
 *      Author: ralfschleicher
 */

#ifndef TYPEHELPER_HPP_
#define TYPEHELPER_HPP_

#include <geometry_msgs/Point.h>
#include "typedef.hpp"

namespace next_best_view {
	class TypeHelper {
	public:
		static geometry_msgs::Point getGeometryMsgsPoint(const SimpleVector3 &vector);
		static SimpleVector3 getSimpleVector3(const geometry_msgs::Point &point);
	};
}


#endif /* TYPEHELPER_HPP_ */
