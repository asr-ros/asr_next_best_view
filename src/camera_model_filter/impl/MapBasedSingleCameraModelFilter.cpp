/*
 * StereoCameraModelFilter.hpp
 *
 *  Created on: Sep 25, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/camera_model_filter/impl/MapBasedSingleCameraModelFilter.hpp"
#include <boost/foreach.hpp>

namespace next_best_view {
    Raytracing2DBasedSingleCameraModelFilter::Raytracing2DBasedSingleCameraModelFilter(const MapHelperPtr &mapUtilPtr, const SimpleVector3 &pivotPointOffset) :
		SingleCameraModelFilter(pivotPointOffset), mMapHelperPtr(mapUtilPtr) {
	}

    void Raytracing2DBasedSingleCameraModelFilter::doFiltering(IndicesPtr &indicesPtr) {
		// create the result.
		IndicesPtr intermediateIndicesPtr = IndicesPtr(new Indices());
		SingleCameraModelFilter::doFiltering(intermediateIndicesPtr);

		SimpleVector3 cameraPosition = this->getPivotPointPosition() + this->getPivotPointOffset();

		indicesPtr = IndicesPtr(new Indices());
		BOOST_FOREACH(int index, *intermediateIndicesPtr) {
			ObjectPoint &point = this->getInputCloud()->at(index);

			if (!mMapHelperPtr->doRaytracing(cameraPosition, point.getPosition())) {
				continue;
			}

			indicesPtr->push_back(index);
		}
	}

    viz::MarkerArrayPtr Raytracing2DBasedSingleCameraModelFilter::getVisualizationMarkerArray(uint32_t &sequence, double lifetime) {
		return SingleCameraModelFilter::getVisualizationMarkerArray(sequence, lifetime);
	}
}

