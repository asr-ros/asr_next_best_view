/*
 * RobotState.hpp
 *
 *  Created on: Nov 2, 2014
 *      Author: ralfschleicher
 */

#ifndef ROBOTSTATE_HPP_
#define ROBOTSTATE_HPP_

#include <boost/shared_ptr.hpp>

namespace next_best_view {
	struct RobotState {
	public:
		RobotState();
		virtual ~RobotState();
	};

	typedef boost::shared_ptr<RobotState> RobotStatePtr;
}


#endif /* ROBOTSTATE_HPP_ */
