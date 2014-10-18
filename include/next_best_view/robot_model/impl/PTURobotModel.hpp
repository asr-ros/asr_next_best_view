/*
 * PTURoboterState.hpp
 *
 *  Created on: Sep 23, 2014
 *      Author: ralfschleicher
 */

#ifndef PTUROBOTMODEL_HPP_
#define PTUROBOTMODEL_HPP_

#include "next_best_view/robot_model/RobotModel.hpp"
#include "typedef.hpp"

namespace next_best_view {
	/*!
	 * \brief PTURobotModel implements a model of pan tilt unit robot.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
	class PTURobotModel : public RobotModel {
	private:
		/*!
		 * the current angle of pan
		 */
		float mCurrentPanAngle;

		/*!
		 * the current angle of tilt
		 */
		float mCurrentTiltAngle;

		/*!
		 * the current angle of rotations
		 */
		float mCurrentRotationAngle;

		/*!
		 * the minimum angle of pan
		 */
		float mMinPanAngle;

		/*!
		 * the maximum angle of pan
		 */
		float mMaxPanAngle;

		/*!
		 * the minimum angle of tilt
		 */
		float mMinTiltAngle;

		/*!
		 * the maximum angle of tilt
		 */
		float mMaxTiltAngle;
	public:
		/*!
		 * \brief constructor of the PTURobotModel
		 */
		PTURobotModel();

		/*!
		 * \brief destructor of the PTURobotModel
		 */
		virtual ~PTURobotModel();

		/*!
		 * \brief sets the angle limits of the pan angle.
		 * \param minAngleDegrees the minimum angle in degrees
		 * \param maxAngleDegrees the maximum angle in degrees
		 */
		void setPanAngleLimits(float minAngleDegrees, float maxAngleDegrees);

		/*!
		 * \brief sets the angle limits of the tilt angle.
		 * \param minAngleDegrees the minimum angle in degrees
		 * \param maxAngleDegrees the maximum angle in degrees
		 */
		void setTiltAngleLimits(float minAngleDegrees, float maxAngleDegrees);

		/*!
		 * \return the span between lower and upper angle.
		 */
		float getPanSpan();

		/*!
		 * \brief sets the current state of the robot model.
		 */
		void setCurrentState(float panAngleDegrees, float tiltAngleDegrees, float rotationAngleDegrees);

		bool isOrientationReachable(SimpleQuaternion orientation);

		float getMovementRating(SimpleQuaternion orientation);
	};

	typedef boost::shared_ptr<PTURobotModel> PTURobotModelPtr;
}


#endif /* PTUROBOTMODEL_HPP_ */
