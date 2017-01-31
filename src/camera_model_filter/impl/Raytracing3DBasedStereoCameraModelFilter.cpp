#include "next_best_view/camera_model_filter/impl/Raytracing3DBasedStereoCameraModelFilter.hpp"

namespace next_best_view {
    Raytracing3DBasedStereoCameraModelFilter::Raytracing3DBasedStereoCameraModelFilter(const WorldHelperPtr &worldHelperPtr,
                                                                                       const SimpleVector3 &leftCameraPivotPointOffset,
                                                                                       const SimpleVector3 &rightCameraPivotPointOffset) :
        StereoCameraModelFilter(leftCameraPivotPointOffset, rightCameraPivotPointOffset), mWorldHelperPtr(worldHelperPtr) {
	}

    void Raytracing3DBasedStereoCameraModelFilter::doFiltering(IndicesPtr &indicesPtr) {
		this->copySettings();

		// create the result.
		IndicesPtr intermediateIndicesPtr = IndicesPtr(new Indices());
		StereoCameraModelFilter::doFiltering(intermediateIndicesPtr);

		SimpleVector3 leftCameraPosition = this->getPivotPointPosition() + this->getLeftCameraPivotPointOffset();
		SimpleVector3 rightCameraPosition = this->getPivotPointPosition() + this->getRightCameraPivotPointOffset();

        mWorldHelperPtr->filterOccludedObjects(leftCameraPosition, this->getInputPointCloud(), intermediateIndicesPtr, indicesPtr);
        mWorldHelperPtr->filterOccludedObjects(rightCameraPosition, this->getInputPointCloud(), intermediateIndicesPtr, indicesPtr);
	}

}

