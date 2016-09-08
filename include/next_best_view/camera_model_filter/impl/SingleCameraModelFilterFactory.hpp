/*
 * SingleCameraModelFilterFactory.hpp
 *
 * Created on: Aug 29, 2016
 *      Author: Daniel Stroh
 */

#pragma once

#include "next_best_view/camera_model_filter/impl/SingleCameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/CameraModelFilterAbstractFactory.hpp"

namespace next_best_view {

    class SingleCameraModelFilterFactory : public CameraModelFilterAbstractFactory {
    private:
        SimpleVector3 oneCameraPivotPointOffset;
        double fovx, fovy;
        double fcp, ncp;
        double speedFactorRecognizer;

    public:
        SingleCameraModelFilterFactory(SimpleVector3 oneCameraPivotPointOffset,
                                       double fovx, double fovy,
                                       double fcp, double ncp, double speedFactorRecognizer)
            : oneCameraPivotPointOffset(oneCameraPivotPointOffset),
              fovx(fovx), fovy(fovy),
              fcp(fcp), ncp(ncp),
              speedFactorRecognizer(speedFactorRecognizer)
        { }

        CameraModelFilterPtr createCameraModelFilter() {
            CameraModelFilterPtr cameraModelFilter = CameraModelFilterPtr(new SingleCameraModelFilter(oneCameraPivotPointOffset));
            cameraModelFilter->setHorizontalFOV(fovx);
            cameraModelFilter->setVerticalFOV(fovy);
            cameraModelFilter->setNearClippingPlane(ncp);
            cameraModelFilter->setFarClippingPlane(fcp);
            cameraModelFilter->setRecognizerCosts((float) speedFactorRecognizer, "");
            return cameraModelFilter;
        }
    };
    typedef boost::shared_ptr<SingleCameraModelFilterFactory> SingleCameraModelFilterFactoryPtr;
}

