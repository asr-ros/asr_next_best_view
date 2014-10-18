/*
 * PTURoboterState.hpp
 *
 *  Created on: Sep 23, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/robot_model/impl/PTURobotModel.hpp"
#include "next_best_view/helper/MathHelper.hpp"

namespace next_best_view {
	PTURobotModel::PTURobotModel() : RobotModel(), mCurrentPanAngle(0.0), mCurrentTiltAngle(0.0), mCurrentRotationAngle(0.0), mMinPanAngle(0.0), mMaxPanAngle(0.0), mMinTiltAngle(0.0), mMaxTiltAngle(0.0) {
	}

	PTURobotModel::~PTURobotModel() {}

	void PTURobotModel::setPanAngleLimits(float minAngleDegrees, float maxAngleDegrees) {
		mMinPanAngle = MathHelper::degToRad(minAngleDegrees);
		mMaxPanAngle = MathHelper::degToRad(maxAngleDegrees);
	}

	/**
	 * @return the span between lower and upper angle.
	 */
	float PTURobotModel::getPanSpan() {
		return mMaxPanAngle - mMinPanAngle;
	}

	void PTURobotModel::setTiltAngleLimits(float minAngleDegrees, float maxAngleDegrees) {
		mMinTiltAngle = MathHelper::degToRad(minAngleDegrees);
		mMaxTiltAngle = MathHelper::degToRad(maxAngleDegrees);
	}

	void PTURobotModel::setCurrentState(float panAngleDegrees, float tiltAngleDegrees, float rotationAngleDegrees) {
		mCurrentPanAngle = MathHelper::degToRad(panAngleDegrees);
		mCurrentTiltAngle = MathHelper::degToRad(tiltAngleDegrees);
		mCurrentRotationAngle = MathHelper::degToRad(rotationAngleDegrees);
	}

	bool PTURobotModel::isOrientationReachable(SimpleQuaternion orientation) {
		SimpleVector3 cartesianCoords = orientation.toRotationMatrix() * SimpleVector3::UnitX();
		SimpleVector3 polarCoords = MathHelper::getSphericalCoords(cartesianCoords);

		float thetaAngle = polarCoords[2];
		return (mMinTiltAngle <= thetaAngle && thetaAngle <= mMaxTiltAngle);
	}

	float PTURobotModel::getMovementRating(SimpleQuaternion orientation) {
		SimpleVector3 cartesianCoords = orientation.toRotationMatrix() * SimpleVector3::UnitX();
		SimpleVector3 polarCoords = MathHelper::getSphericalCoords(cartesianCoords);

		float phiAngle = polarCoords[1];

//		float panLowerLimit = fmodf(mMinPanAngle + mCurrentRotationAngle + 2 * M_PI, 2 * M_PI);
//		float panUpperLimit = fmodf(mMaxPanAngle + mCurrentRotationAngle + 2 * M_PI, 2 * M_PI);

		//bool reachable = (panLowerLimit <= panUpperLimit ? (panLowerLimit <= phiAngle && phiAngle <= panUpperLimit) : (0 <= phiAngle && phiAngle <= panUpperLimit) || (panLowerLimit <= phiAngle && phiAngle <= 2 * M_PI));
		float angleDifference = min(fabsf(mCurrentPanAngle - phiAngle), (float) (2 * M_PI - fabsf(mCurrentPanAngle - phiAngle)));
		float panRating = MathHelper::getRatingFunction(angleDifference, getPanSpan() * .5);
		if (panRating != 0.0) {
			panRating += 1;
		}

		return panRating;
	}
}

