#include "next_best_view/camera_model_filter/impl/Raytracing3DBasedSingleCameraModelFilter.hpp"

namespace next_best_view {
    Raytracing3DBasedSingleCameraModelFilter::Raytracing3DBasedSingleCameraModelFilter(const WorldHelperPtr &worldHelperPtr,
                                                                                       const SimpleVector3 &pivotPointOffset) :
        SingleCameraModelFilter(pivotPointOffset), mWorldHelperPtr(worldHelperPtr) {
	}

    void Raytracing3DBasedSingleCameraModelFilter::doFiltering(IndicesPtr &indicesPtr) {
		// create the result.
		IndicesPtr intermediateIndicesPtr = IndicesPtr(new Indices());
		SingleCameraModelFilter::doFiltering(intermediateIndicesPtr);

		SimpleVector3 cameraPosition = this->getPivotPointPosition() + this->getPivotPointOffset();

        mWorldHelperPtr->filterOccludedObjects(cameraPosition, this->getInputPointCloud(), intermediateIndicesPtr, indicesPtr);
	}
}

