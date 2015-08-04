/*
 * CameraModel.hpp
 *
 *  Created on: Sep 25, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/camera_model_filter/CameraModelFilter.hpp"

namespace next_best_view {
	CameraModelFilter::CameraModelFilter() : GeneralFilter(), mFOVX(0.0), mFOVY(0.0), mNcp(0.0), mFcp(0.0) {
	}

	CameraModelFilter::~CameraModelFilter() { }

	void CameraModelFilter::setPivotPointPose(const SimpleVector3 &position, const SimpleQuaternion &orientation) {
		this->setPivotPointPosition(position);
		this->setOrientation(orientation);
	}

	void CameraModelFilter::setPivotPointPosition(const SimpleVector3 &position) {
		mPivotPointPosition = position;
	}

	SimpleVector3 CameraModelFilter::getPivotPointPosition() {
		return mPivotPointPosition;
	}

	void CameraModelFilter::setOrientation(const SimpleQuaternion &orientation) {
		mPivotPointOrientation = orientation;
	}

	SimpleQuaternion CameraModelFilter::getOrientation() {
		return mPivotPointOrientation;
	}

	SimpleMatrix4 CameraModelFilter::getCameraPoseMatrix(const SimpleVector3 &position, const SimpleQuaternion &orientation) {
		SimpleMatrix4 cameraPoseMatrix = SimpleMatrix4::Identity();
		cameraPoseMatrix.block<3, 3>(0, 0) = orientation.toRotationMatrix();
		cameraPoseMatrix.block<3, 1>(0, 3) = position;
		return cameraPoseMatrix;
	}

	void CameraModelFilter::setHorizontalFOV(double fovDegrees) {
		mFOVX = fovDegrees;
	}

	double CameraModelFilter::getHorizontalFOV() {
		return mFOVX;
	}

	void CameraModelFilter::setVerticalFOV(double fovDegrees) {
		mFOVY = fovDegrees;
	}

	double CameraModelFilter::getVerticalFOV() {
		return mFOVY;
	}

	void CameraModelFilter::setNearClippingPlane(double ncp) {
		mNcp = ncp;
	}

	double CameraModelFilter::getNearClippingPlane() {
		return mNcp;
	}

	void CameraModelFilter::setFarClippingPlane(double fcp) {
		mFcp = fcp;
	}

	double CameraModelFilter::getFarClippingPlane() {
		return mFcp;
	}
}

