/*
 * StereoCameraModelFilter.hpp
 *
 *  Created on: Sep 25, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/camera_model_filter/impl/MapBasedStereoCameraModelFilter.hpp"
#include <boost/foreach.hpp>

namespace next_best_view {
    Raytracing2DBasedStereoCameraModelFilter::Raytracing2DBasedStereoCameraModelFilter(const MapHelperPtr &mapUtilPtr, const SimpleVector3 &leftCameraPivotPointOffset, const SimpleVector3 &rightCameraPivotPointOffset) :
		StereoCameraModelFilter(leftCameraPivotPointOffset, rightCameraPivotPointOffset), mMapHelperPtr(mapUtilPtr) {
	}

    void Raytracing2DBasedStereoCameraModelFilter::doFiltering(IndicesPtr &indicesPtr) {
		this->copySettings();

		// create the result.
		IndicesPtr intermediateIndicesPtr = IndicesPtr(new Indices());
		StereoCameraModelFilter::doFiltering(intermediateIndicesPtr);

		SimpleVector3 leftCameraPosition = this->getPivotPointPosition() + this->getLeftCameraPivotPointOffset();
		SimpleVector3 rightCameraPosition = this->getPivotPointPosition() + this->getRightCameraPivotPointOffset();

		indicesPtr = IndicesPtr(new Indices());
		BOOST_FOREACH(int index, *intermediateIndicesPtr) {
			ObjectPoint &point = this->getInputCloud()->at(index);

			if (!mMapHelperPtr->doRaytracing(leftCameraPosition, point.getPosition())) {
				continue;
			}

			if (!mMapHelperPtr->doRaytracing(rightCameraPosition, point.getPosition())) {
				continue;
			}

			indicesPtr->push_back(index);
		}
	}

    viz::MarkerArrayPtr Raytracing2DBasedStereoCameraModelFilter::getVisualizationMarkerArray(uint32_t &sequence, double lifetime) {
		this->copySettings();
		return StereoCameraModelFilter::getVisualizationMarkerArray(sequence, lifetime);
	}
}

