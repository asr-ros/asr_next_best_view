/**

Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Braun Kai, Heller Florian, Hutmacher Robin, Karrenbauer Oliver, Marek Felix, Mayr Matthias, Mehlhaus Jonas, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Stroh Daniel, Trautmann Jeremias, Walter Milena
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

4. The use is explicitly not permitted to any application which deliberately try to kill or do harm to any living creature.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "next_best_view/camera_model_filter/impl/StereoCameraModelFilter.hpp"

namespace next_best_view {
	StereoCameraModelFilter::StereoCameraModelFilter(const SimpleVector3 &leftCameraPivotPointOffset, const SimpleVector3 &rightCameraPivotPointOffset) :
		CameraModelFilter(), mFilteringType(INTERSECTION) {
		mLeftCameraModelFilter = SingleCameraModelFilter(leftCameraPivotPointOffset);
		mRightCameraModelFilter = SingleCameraModelFilter(rightCameraPivotPointOffset);
	}

    void StereoCameraModelFilter::copySettings() {
		if (!this->haveParametersChanged()) {
			return;
		}

		// set up left camera
		mLeftCameraModelFilter.setHorizontalFOV(this->getHorizontalFOV());
		mLeftCameraModelFilter.setVerticalFOV(this->getVerticalFOV());
		mLeftCameraModelFilter.setNearClippingPlane(this->getNearClippingPlane());
		mLeftCameraModelFilter.setFarClippingPlane(this->getFarClippingPlane());
		mLeftCameraModelFilter.setPivotPointPosition(this->getPivotPointPosition());
		mLeftCameraModelFilter.setOrientation(this->getOrientation());
		mLeftCameraModelFilter.setInputCloud(this->getInputCloud());
		mLeftCameraModelFilter.setIndices(this->getIndices());

		// set up right camera
		mRightCameraModelFilter.setHorizontalFOV(this->getHorizontalFOV());
		mRightCameraModelFilter.setVerticalFOV(this->getVerticalFOV());
		mRightCameraModelFilter.setNearClippingPlane(this->getNearClippingPlane());
		mRightCameraModelFilter.setFarClippingPlane(this->getFarClippingPlane());
		mRightCameraModelFilter.setPivotPointPosition(this->getPivotPointPosition());
		mRightCameraModelFilter.setOrientation(this->getOrientation());
		mRightCameraModelFilter.setInputCloud(this->getInputCloud());
		mRightCameraModelFilter.setIndices(this->getIndices());

		this->setParametersChanged(false);
	}

	void StereoCameraModelFilter::doFiltering(IndicesPtr &indicesPtr) {
		this->copySettings();

		// create the result.
		indicesPtr = IndicesPtr(new Indices());

		// do the filtering
		FilteringType filteringType = this->getFilteringType();
		if (filteringType == LEFT) {
			mLeftCameraModelFilter.filter(indicesPtr);
		} else if (filteringType == RIGHT) {
			mRightCameraModelFilter.filter(indicesPtr);
		} else if (filteringType == BOTH) {
			// get the left filter
			IndicesPtr leftIndicesPtr(new Indices());
			mLeftCameraModelFilter.filter(leftIndicesPtr);

			// get the right filter
			IndicesPtr rightIndicesPtr(new Indices());
			mRightCameraModelFilter.filter(rightIndicesPtr);

			// create the result.
			indicesPtr = IndicesPtr(new Indices(leftIndicesPtr->size() + rightIndicesPtr->size()));
			//  merge both vectors
			std::merge(leftIndicesPtr->begin(), leftIndicesPtr->end(), rightIndicesPtr->begin(), rightIndicesPtr->end(), indicesPtr->begin());

			// ensure they are sorted to be considered unique
			std::sort(indicesPtr->begin(), indicesPtr->end());
			Indices::iterator uniqueEndIterator = std::unique(indicesPtr->begin(), indicesPtr->end());

			// resize the resulting vector
			indicesPtr->resize(std::distance(indicesPtr->begin(), uniqueEndIterator));
		} else {
			// get the left filter
			IndicesPtr leftIndicesPtr(new Indices());
			mLeftCameraModelFilter.filter(leftIndicesPtr);

			// setting the left indices as resource
			mRightCameraModelFilter.setIndices(leftIndicesPtr);
			mRightCameraModelFilter.filter(indicesPtr);
		}
	}

	void StereoCameraModelFilter::setFilteringType(FilteringType type) {
		mFilteringType = type;
	}

	StereoCameraModelFilter::FilteringType StereoCameraModelFilter::getFilteringType() {
		return mFilteringType;
	}

	void StereoCameraModelFilter::setLeftCameraPivotPointOffset(const SimpleVector3 &cameraPivotPointOffset) {
		mLeftCameraModelFilter.setPivotPointOffset(cameraPivotPointOffset);
	}

	SimpleVector3 StereoCameraModelFilter::getLeftCameraPivotPointOffset() {
		return mLeftCameraModelFilter.getPivotPointOffset();
	}

	void StereoCameraModelFilter::setRightCameraPivotPointOffset(const SimpleVector3 &cameraPivotPointOffset) {
		mRightCameraModelFilter.setPivotPointOffset(cameraPivotPointOffset);
	}

	SimpleVector3 StereoCameraModelFilter::getRightCameraPivotPointOffset() {
		return mRightCameraModelFilter.getPivotPointOffset();
	}

	viz::MarkerArrayPtr StereoCameraModelFilter::getVisualizationMarkerArray(uint32_t &sequence, double lifetime) {
		this->copySettings();

		viz::MarkerArrayPtr leftMarkerArrayPtr = mLeftCameraModelFilter.getVisualizationMarkerArray(sequence, lifetime);
		viz::MarkerArrayPtr rightMarkerArrayPtr = mRightCameraModelFilter.getVisualizationMarkerArray(sequence, lifetime);

		viz::MarkerArrayPtr markerArrayPtr = viz::MarkerArrayPtr(new viz::MarkerArray());
		markerArrayPtr->markers = std::vector<viz::Marker>(leftMarkerArrayPtr->markers.size() + rightMarkerArrayPtr->markers.size());
		std::copy(leftMarkerArrayPtr->markers.begin(), leftMarkerArrayPtr->markers.end(), markerArrayPtr->markers.begin());
		std::copy(rightMarkerArrayPtr->markers.begin(), rightMarkerArrayPtr->markers.end(), markerArrayPtr->markers.begin() + leftMarkerArrayPtr->markers.size());

		return markerArrayPtr;
	}
}

