/*
 * StereoCameraModelFilter.hpp
 *
 *  Created on: Sep 25, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/camera_model_filter/impl/CostmapBasedStereoCameraModelFilter.hpp"
#include <boost/foreach.hpp>

namespace next_best_view {
	CostmapBasedStereoCameraModelFilter::CostmapBasedStereoCameraModelFilter(const MapUtilityPtr &mapUtilPtr, const SimpleVector3 &leftCameraPivotPointOffset, const SimpleVector3 &rightCameraPivotPointOffset) :
		CameraModelFilter(), mMapUtilityPtr(mapUtilPtr) {
		mCameraModelFilter = StereoCameraModelFilter(leftCameraPivotPointOffset, rightCameraPivotPointOffset);
	}

	void CostmapBasedStereoCameraModelFilter::copySettings() {
		// set up left camera
		mCameraModelFilter.setHorizontalFOV(this->getHorizontalFOV());
		mCameraModelFilter.setVerticalFOV(this->getVerticalFOV());
		mCameraModelFilter.setNearClippingPlane(this->getNearClippingPlane());
		mCameraModelFilter.setFarClippingPlane(this->getFarClippingPlane());
		mCameraModelFilter.setPivotPointPosition(this->getPivotPointPosition());
		mCameraModelFilter.setOrientation(this->getOrientation());
		mCameraModelFilter.setInputCloud(this->getInputCloud());
		mCameraModelFilter.setIndices(this->getIndices());
	}

	void CostmapBasedStereoCameraModelFilter::filter(IndicesPtr &indicesPtr) {
		this->copySettings();

		// create the result.
		IndicesPtr intermediateIndicesPtr = IndicesPtr(new Indices());
		mCameraModelFilter.filter(intermediateIndicesPtr);

		SimpleVector3 leftCameraPosition = this->getPivotPointPosition() + mCameraModelFilter.getLeftCameraPivotPointOffset();
		SimpleVector3 rightCameraPosition = this->getPivotPointPosition() + mCameraModelFilter.getRightCameraPivotPointOffset();

		indicesPtr = IndicesPtr(new Indices());
		BOOST_FOREACH(int index, *intermediateIndicesPtr) {
			ObjectPoint &point = this->getInputCloud()->at(index);

			if (!mMapUtilityPtr->doRaytracing(leftCameraPosition, point.getSimpleVector3())) {
				continue;
			}

			if (!mMapUtilityPtr->doRaytracing(rightCameraPosition, point.getSimpleVector3())) {
				continue;
			}

			indicesPtr->push_back(index);
		}
	}

	viz::MarkerArrayPtr CostmapBasedStereoCameraModelFilter::getVisualizationMarkerArray(uint32_t &sequence, double lifetime) {
		this->copySettings();
		return mCameraModelFilter.getVisualizationMarkerArray(sequence, lifetime);
	}
}

