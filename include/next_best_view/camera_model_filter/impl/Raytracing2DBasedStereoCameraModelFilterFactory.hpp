/*
 * Raytracing2DBasedStereoCameraModelFilterFactory.hpp
 *
 * Created on: Aug 29, 2016
 *      Author: Daniel Stroh
 */

#pragma once

#include "next_best_view/helper/MapHelperFactory.hpp"
#include "next_best_view/camera_model_filter/impl/Raytracing2DBasedStereoCameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/CameraModelFilterAbstractFactory.hpp"

namespace next_best_view {

    class Raytracing2DBasedStereoCameraModelFilterFactory : public CameraModelFilterAbstractFactory {
    private:
        MapHelperFactoryPtr mapHelperFactory;
        SimpleVector3 leftCameraPivotPointOffset;
        SimpleVector3 rightCameraPivotPointOffset;
        double fovx, fovy;
        double fcp, ncp;
        double speedFactorRecognizer;

    public:
        Raytracing2DBasedStereoCameraModelFilterFactory(MapHelperFactoryPtr mapHelperFactory,
                                                        SimpleVector3 leftCameraPivotPointOffset,
                                                        SimpleVector3 rightCameraPivotPointOffset,
                                                        double fovx, double fovy,
                                                        double fcp, double ncp, double speedFactorRecognizer)
            : mapHelperFactory(mapHelperFactory),
              leftCameraPivotPointOffset(leftCameraPivotPointOffset),
              rightCameraPivotPointOffset(rightCameraPivotPointOffset),
              fovx(fovx), fovy(fovy),
              fcp(fcp), ncp(ncp),
              speedFactorRecognizer(speedFactorRecognizer)
        { }

        CameraModelFilterPtr createCameraModelFilter() {
            CameraModelFilterPtr cameraModelFilter = CameraModelFilterPtr(new Raytracing2DBasedStereoCameraModelFilter(mapHelperFactory->createMapHelper(), leftCameraPivotPointOffset, rightCameraPivotPointOffset));
            cameraModelFilter->setHorizontalFOV(fovx);
            cameraModelFilter->setVerticalFOV(fovy);
            cameraModelFilter->setNearClippingPlane(ncp);
            cameraModelFilter->setFarClippingPlane(fcp);
            cameraModelFilter->setRecognizerCosts((float) speedFactorRecognizer, "");
            return cameraModelFilter;
        }
    };
    typedef boost::shared_ptr<Raytracing2DBasedStereoCameraModelFilterFactory> Raytracing2DBasedStereoCameraModelFilterFactoryPtr;
}

