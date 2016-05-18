/*
 * PTURoboterState.hpp
 *
 *  Created on: Sep 23, 2014
 *      Author: ralfschleicher
 */

#pragma once

#include <boost/tuple/tuple.hpp>
#include "next_best_view/robot_model/impl/MILDRobotModel.hpp"
#include "typedef.hpp"
#include "geometry_msgs/Pose.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "next_best_view/helper/DebugHelper.hpp"

namespace next_best_view {
	/*!
     * \brief MILDRobotModelWithApproximatedIK implements a model of pan tilt unit robot where the inverse kinematic is defined by an approximation function.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
    class MILDRobotModelWithApproximatedIK : public MILDRobotModel {
	public:
		/*!
         * \brief constructor of the MILDRobotModelWithApproximatedIK
		 */
        MILDRobotModelWithApproximatedIK();

		/*!
         * \brief destructor of the MILDRobotModelWithApproximatedIK
		 */
        virtual ~MILDRobotModelWithApproximatedIK();


		RobotStatePtr calculateRobotState(const RobotStatePtr &sourceRobotState, const SimpleVector3 &position, const SimpleQuaternion &orientation);
		
	};

    typedef boost::shared_ptr<MILDRobotModelWithApproximatedIK> MILDRobotModelWithApproximatedIKPtr;
}
