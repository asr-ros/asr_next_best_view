/*
 * RoboterState.hpp
 *
 *  Created on: Sep 23, 2014
 *      Author: ralfschleicher
 */

#ifndef ROBOTMODEL_HPP_
#define ROBOTMODEL_HPP_

#include "typedef.hpp"

namespace next_best_view {
	/*!
	 * \brief RobotModel generalizes a robot model.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
	class RobotModel {
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
		virtual bool isOrientationReachable(SimpleQuaternion orientation) = 0;

		/*!
		 * \param orientation the orientation to move to.
		 * \return the rating to move to the orientation.
		 */
		virtual float getMovementRating(SimpleQuaternion orientation) = 0;

		/*!
		 * \brief completely filters the reachable orientations
		 * \param orientationsPtr [in / out] a shared pointer to a collection of orientations.
		 */
		void filterReachableOrientations(SimpleQuaternionCollectionPtr &orientationsPtr);
	};

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<RobotModel> RobotModelPtr;
}

#endif /* ROBOTMODEL_HPP_ */
