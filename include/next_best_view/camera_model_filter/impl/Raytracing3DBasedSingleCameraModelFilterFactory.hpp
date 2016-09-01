/*
 * Raytracing2DBasedSingleCameraModelFilterFactory.hpp
 *
 * Created on: Aug 29, 2016
 *      Author: Daniel Stroh
 */

#pragma once

#include "next_best_view/helper/WorldHelper.hpp"
#include "next_best_view/camera_model_filter/impl/Raytracing3DBasedSingleCameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/CameraModelFilterAbstractFactory.hpp"

namespace next_best_view {

   class Raytracing3DBasedSingleCameraModelFilterFactory : public CameraModelFilterAbstractFactory {
   private:
       WorldHelperPtr worldHelperPtr;
       SimpleVector3 oneCameraPivotPointOffset;
       double fovx, fovy;
       double fcp, ncp;
       double speedFactorRecognizer;

   public:
       Raytracing3DBasedSingleCameraModelFilterFactory(const WorldHelperPtr &worldHelperPtr,
                                                       SimpleVector3 oneCameraPivotPointOffset,
                                                       double fovx, double fovy,
                                                       double fcp, double ncp, double speedFactorRecognizer)
           : worldHelperPtr(worldHelperPtr),
             oneCameraPivotPointOffset(oneCameraPivotPointOffset),
             fovx(fovx), fovy(fovy),
             fcp(fcp), ncp(ncp),
             speedFactorRecognizer(speedFactorRecognizer)
       { }

       CameraModelFilterPtr createCameraModelFilter() {
           CameraModelFilterPtr cameraModelFilter = CameraModelFilterPtr(new Raytracing3DBasedSingleCameraModelFilter(worldHelperPtr, oneCameraPivotPointOffset));
           cameraModelFilter->setHorizontalFOV(fovx);
           cameraModelFilter->setVerticalFOV(fovy);
           cameraModelFilter->setNearClippingPlane(ncp);
           cameraModelFilter->setFarClippingPlane(fcp);
           cameraModelFilter->setRecognizerCosts((float) speedFactorRecognizer, "");
           return cameraModelFilter;
       }
   };
   typedef boost::shared_ptr<Raytracing3DBasedSingleCameraModelFilterFactory> Raytracing3DBasedSingleCameraModelFilterFactoryPtr;
}


