/*
 * RoboterState.hpp
 *
 *  Created on: Sep 23, 2014
 *      Author: ralfschleicher
 */

#ifndef ROBOTMODEL_HPP_
#define ROBOTMODEL_HPP_

#include "next_best_view/common/CommonClass.hpp"
#include "next_best_view/robot_model/RobotState.hpp"
#include "typedef.hpp"

namespace next_best_view {
	/*!
	 * \brief RobotModel generalizes a robot model.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
	class RobotModel : public CommonClass {
	private:
		RobotStatePtr mCurrentRobotState;
	public:
		/*!
		 * \brief the constructor of the RobotModel object
		 */
		RobotModel();

		/*!
		 * \brief the destructor of the RobotModel object
		 */
		virtual ~RobotModel();

		/*!
		 * \param orientation the orientation to reach
		 * \return if the orienation is reachable or not
		 */
		virtual bool isPoseReachable(const SimpleVector3 &position, const SimpleQuaternion &orientation) = 0;

		/*!
		 * \brief calculates the target robot state by assuming the saved current state of the roboter as source state
		 * \param position the position
		 * \param orientation the orientation
		 * \return the new robot state
		 */
		RobotStatePtr calculateRobotState(const SimpleVector3 &position, const SimpleQuaternion &orientation);

		/*!
		 * \brief calculates the target robot state
		 * \param currentRobotState the current robot state
		 * \param position the position
		 * \param orientation the orientation
		 * \return the new robot state
		 */
		virtual RobotStatePtr calculateRobotState(const RobotStatePtr &sourceRobotState, const SimpleVector3 &position, const SimpleQuaternion &orientation) = 0;

		/*!
		 * \param targetRobotState the target state
		 * \return the rating
		 */
		float getMovementCosts(const RobotStatePtr &targetRobotState);

		/*!
		 * \param sourceRobotState the source state
		 * \param targetRobotState the target state
		 * \return the rating to move to the pose.
		 */
		virtual float getMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState) = 0;

		/*!
		 * \param currentRobotState sets the current robot state ptr
		 */
		void setCurrentRobotState(const RobotStatePtr &currentRobotState);

		/*!
		 * \return the current robot state ptr
		 */
		RobotStatePtr getCurrentRobotState();
	};

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<RobotModel> RobotModelPtr;
}

#endif /* ROBOTMODEL_HPP_ */
