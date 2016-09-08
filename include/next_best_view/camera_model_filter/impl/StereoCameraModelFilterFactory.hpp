/*
 * StereoCameraModelFilterFactory.hpp
 *
 * Created on: Aug 29, 2016
 *      Author: Daniel Stroh
 */

#pragma once

#include "next_best_view/camera_model_filter/impl/StereoCameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/CameraModelFilterAbstractFactory.hpp"

namespace next_best_view {

    class StereoCameraModelFilterFactory : public CameraModelFilterAbstractFactory {
    private:
        SimpleVector3 leftCameraPivotPointOffset;
        SimpleVector3 rightCameraPivotPointOffset;
        double fovx, fovy;
        double fcp, ncp;
        double speedFactorRecognizer;

    public:
        StereoCameraModelFilterFactory(SimpleVector3 leftCameraPivotPointOffset,
                                       SimpleVector3 rightCameraPivotPointOffset,
                                       double fovx, double fovy,
                                       double fcp, double ncp, double speedFactorRecognizer)
            : leftCameraPivotPointOffset(leftCameraPivotPointOffset),
              rightCameraPivotPointOffset(rightCameraPivotPointOffset),
              fovx(fovx), fovy(fovy),
              fcp(fcp), ncp(ncp),
              speedFactorRecognizer(speedFactorRecognizer)
        { }

        CameraModelFilterPtr createCameraModelFilter() {
            CameraModelFilterPtr cameraModelFilter = CameraModelFilterPtr(new StereoCameraModelFilter(leftCameraPivotPointOffset, rightCameraPivotPointOffset));
            cameraModelFilter->setHorizontalFOV(fovx);
            cameraModelFilter->setVerticalFOV(fovy);
            cameraModelFilter->setNearClippingPlane(ncp);
            cameraModelFilter->setFarClippingPlane(fcp);
            cameraModelFilter->setRecognizerCosts((float) speedFactorRecognizer, "");
            return cameraModelFilter;
        }
    };
    typedef boost::shared_ptr<StereoCameraModelFilterFactory> StereoCameraModelFilterFactoryPtr;
}
