/*
 * Raytracing2DBasedSingleCameraModelFilterFactory.hpp
 *
 * Created on: Aug 29, 2016
 *      Author: Daniel Stroh
 */

#pragma once

#include "next_best_view/helper/MapHelperFactory.hpp"
#include "next_best_view/camera_model_filter/impl/Raytracing2DBasedSingleCameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/CameraModelFilterAbstractFactory.hpp"

namespace next_best_view {

   class Raytracing2DBasedSingleCameraModelFilterFactory : public CameraModelFilterAbstractFactory {
   private:
       MapHelperFactoryPtr mapHelperFactory;
       SimpleVector3 oneCameraPivotPointOffset;
       double fovx, fovy;
       double fcp, ncp;
       double speedFactorRecognizer;

   public:
       Raytracing2DBasedSingleCameraModelFilterFactory(MapHelperFactoryPtr mapHelperFactory,
                                                       SimpleVector3 oneCameraPivotPointOffset,
                                                       double fovx, double fovy,
                                                       double fcp, double ncp, double speedFactorRecognizer)
           : mapHelperFactory(mapHelperFactory),
             oneCameraPivotPointOffset(oneCameraPivotPointOffset),
             fovx(fovx), fovy(fovy),
             fcp(fcp), ncp(ncp),
             speedFactorRecognizer(speedFactorRecognizer)
       { }

       CameraModelFilterPtr createCameraModelFilter() {
           CameraModelFilterPtr cameraModelFilter = CameraModelFilterPtr(new Raytracing2DBasedSingleCameraModelFilter(mapHelperFactory->createMapHelper(), oneCameraPivotPointOffset));
           cameraModelFilter->setHorizontalFOV(fovx);
           cameraModelFilter->setVerticalFOV(fovy);
           cameraModelFilter->setNearClippingPlane(ncp);
           cameraModelFilter->setFarClippingPlane(fcp);
           cameraModelFilter->setRecognizerCosts((float) speedFactorRecognizer, "");
           return cameraModelFilter;
       }
   };
   typedef boost::shared_ptr<Raytracing2DBasedSingleCameraModelFilterFactory> Raytracing2DBasedSingleCameraModelFilterFactoryPtr;
}


