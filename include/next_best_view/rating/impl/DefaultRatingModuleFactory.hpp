/*
 * DefaultRatingModuleFactory.hpp
 *
 * Created on: Aug 29, 2016
 *      Author: Daniel Stroh
 */

#pragma once

#include "next_best_view/camera_model_filter/CameraModelFilterAbstractFactory.hpp"
#include "next_best_view/robot_model/RobotModelAbstractFactory.hpp"
#include "next_best_view/rating/impl/DefaultRatingModule.hpp"
#include "next_best_view/rating/RatingModuleAbstractFactory.hpp"

namespace next_best_view {

    class DefaultRatingModuleFactory : public RatingModuleAbstractFactory {
    private:
        double fovx, fovy;
        double fcp, ncp;
        RobotModelAbstractFactoryPtr robotModelFactory;
        CameraModelFilterAbstractFactoryPtr cameraModelFilterFactory;
        double angleThreshold;
        double omegaUtility, omegaPan, omegaTilt;
        double omegaRot, omegaBase, omegaRecognizer;

    public:
        DefaultRatingModuleFactory(double fovx, double fovy,
                                   double fcp, double ncp,
                                   RobotModelAbstractFactoryPtr robotModelFactory, CameraModelFilterAbstractFactoryPtr cameraModelFilterFactory,
                                   double angleThreshold,
                                   double omegaUtility, double omegaPan, double omegaTilt,
                                   double omegaRot, double omegaBase, double omegaRecognizer)
            : fovx(fovx), fovy(fovy),
              fcp(fcp), ncp(ncp),
              robotModelFactory(robotModelFactory), cameraModelFilterFactory(cameraModelFilterFactory),
              angleThreshold(angleThreshold),
              omegaUtility(omegaUtility), omegaPan(omegaPan), omegaTilt(omegaTilt),
              omegaRot(omegaRot), omegaBase(omegaBase), omegaRecognizer(omegaRecognizer)

        { }

        RatingModulePtr createRatingModule() {
            DefaultRatingModulePtr defaultRatingModule = DefaultRatingModulePtr(new DefaultRatingModule(fovx, fovy, fcp, ncp, robotModelFactory->createRobotModel(), cameraModelFilterFactory->createCameraModelFilter()));
            defaultRatingModule->setNormalAngleThreshold(angleThreshold);
            defaultRatingModule->setOmegaParameters(omegaUtility, omegaPan, omegaTilt, omegaRot, omegaBase, omegaRecognizer);
            return defaultRatingModule;
        }
    };
    typedef boost::shared_ptr<DefaultRatingModuleFactory> DefaultRatingModuleFactoryPtr;
}
