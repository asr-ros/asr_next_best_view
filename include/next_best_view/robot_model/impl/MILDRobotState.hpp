/*
 * MILDRobotState.hpp
 *
 *  Created on: Nov 2, 2014
 *      Author: ralfschleicher
 */

#ifndef MILDROBOTSTATE_HPP_
#define MILDROBOTSTATE_HPP_

#include "next_best_view/robot_model/RobotState.hpp"

namespace next_best_view {
	struct MILDRobotState : public RobotState {
	public:
		float pan;
		float tilt;
		float rotation;
		float x;
		float y;
	public:
		MILDRobotState(float pan, float tilt, float rotation, float x, float y);
		MILDRobotState();
		virtual ~MILDRobotState();
	};

	typedef boost::shared_ptr<MILDRobotState> MILDRobotStatePtr;
}


#endif /* MILDROBOTSTATE_HPP_ */
