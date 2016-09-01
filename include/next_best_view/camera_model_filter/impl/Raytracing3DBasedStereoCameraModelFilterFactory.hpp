/*
 * Raytracing2DBasedStereoCameraModelFilterFactory.hpp
 *
 * Created on: Aug 29, 2016
 *      Author: Daniel Stroh
 */

#pragma once

#include "next_best_view/helper/MapHelperFactory.hpp"
#include "next_best_view/camera_model_filter/impl/Raytracing3DBasedStereoCameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/CameraModelFilterAbstractFactory.hpp"

namespace next_best_view {

    class Raytracing3DBasedStereoCameraModelFilterFactory : public CameraModelFilterAbstractFactory {
    private:
        WorldHelperPtr worldHelperPtr;
        SimpleVector3 leftCameraPivotPointOffset;
        SimpleVector3 rightCameraPivotPointOffset;
        double fovx, fovy;
        double fcp, ncp;
        double speedFactorRecognizer;

    public:
        Raytracing3DBasedStereoCameraModelFilterFactory(const WorldHelperPtr &worldHelperPtr,
                                                        SimpleVector3 leftCameraPivotPointOffset,
                                                        SimpleVector3 rightCameraPivotPointOffset,
                                                        double fovx, double fovy,
                                                        double fcp, double ncp, double speedFactorRecognizer)
            : worldHelperPtr(worldHelperPtr),
              leftCameraPivotPointOffset(leftCameraPivotPointOffset),
              rightCameraPivotPointOffset(rightCameraPivotPointOffset),
              fovx(fovx), fovy(fovy),
              fcp(fcp), ncp(ncp),
              speedFactorRecognizer(speedFactorRecognizer)
        { }

        CameraModelFilterPtr createCameraModelFilter() {
            CameraModelFilterPtr cameraModelFilter = CameraModelFilterPtr(new Raytracing3DBasedStereoCameraModelFilter(worldHelperPtr, leftCameraPivotPointOffset, rightCameraPivotPointOffset));
            cameraModelFilter->setHorizontalFOV(fovx);
            cameraModelFilter->setVerticalFOV(fovy);
            cameraModelFilter->setNearClippingPlane(ncp);
            cameraModelFilter->setFarClippingPlane(fcp);
            cameraModelFilter->setRecognizerCosts((float) speedFactorRecognizer, "");
            return cameraModelFilter;
        }
    };
    typedef boost::shared_ptr<Raytracing3DBasedStereoCameraModelFilterFactory> Raytracing3DBasedStereoCameraModelFilterFactoryPtr;
}

