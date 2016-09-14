/**

Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Heller Florian, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Stroh Daniel, Trautmann Jeremias, Walter Milena, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

4. The use is explicitly not permitted to any application which deliberately try to kill or do harm to any living creature.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "next_best_view/camera_model_filter/CameraModelFilter.hpp"

namespace next_best_view {
	CameraModelFilter::CameraModelFilter() : GeneralFilter(), mParametersChanged(true), mFOVX(0.0), mFOVY(0.0), mNcp(0.0), mFcp(0.0) {
	}

	CameraModelFilter::~CameraModelFilter() { }

    void CameraModelFilter::setRecognizerCosts(float recognizerCosts, std::string objectType)
    {
        this->recognizerCosts = recognizerCosts;
    }


    float CameraModelFilter::getRecognizerCosts(std::string objectType) {
        return recognizerCosts;
	}

	void CameraModelFilter::setPivotPointPose(const SimpleVector3 &position, const SimpleQuaternion &orientation) {
		this->setPivotPointPosition(position);
		this->setOrientation(orientation);
	}

	void CameraModelFilter::setPivotPointPosition(const SimpleVector3 &position) {
		mPivotPointPosition = position;
		this->setParametersChanged(true);
	}

	SimpleVector3 CameraModelFilter::getPivotPointPosition() {
		return mPivotPointPosition;
	}

	void CameraModelFilter::setOrientation(const SimpleQuaternion &orientation) {
		mPivotPointOrientation = orientation;
		this->setParametersChanged(true);
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
		this->setParametersChanged(true);
	}

	double CameraModelFilter::getHorizontalFOV() {
		return mFOVX;
	}

	void CameraModelFilter::setVerticalFOV(double fovDegrees) {
		mFOVY = fovDegrees;
		this->setParametersChanged(true);
	}

	double CameraModelFilter::getVerticalFOV() {
		return mFOVY;
	}

	void CameraModelFilter::setNearClippingPlane(double ncp) {
		mNcp = ncp;
		this->setParametersChanged(true);
	}

	double CameraModelFilter::getNearClippingPlane() {
		return mNcp;
	}

	void CameraModelFilter::setFarClippingPlane(double fcp) {
		mFcp = fcp;
		this->setParametersChanged(true);
	}

	double CameraModelFilter::getFarClippingPlane() {
		return mFcp;
	}
}

