/**

Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Heller Florian, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Stroh Daniel, Trautmann Jeremias, Walter Milena, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

4. The use is explicitly not permitted to any application which deliberately try to kill or do harm to any living creature.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

#include "next_best_view/camera_model_filter/CameraModelFilterAbstractFactory.hpp"
#include <robot_model_services/robot_model/RobotModelAbstractFactory.hpp>
#include "next_best_view/rating/impl/DefaultRatingModule.hpp"
#include "next_best_view/rating/RatingModuleAbstractFactory.hpp"
#include "next_best_view/helper/MapHelper.hpp"

namespace next_best_view {

    class DefaultRatingModuleFactory : public RatingModuleAbstractFactory {
    private:
        double mFovx, mFovy;
        double mFcp, mNcp;
        double mUseTargetRobotState;
        robot_model_services::RobotModelAbstractFactoryPtr mRobotModelFactoryPtr;
        CameraModelFilterAbstractFactoryPtr mCameraModelFilterFactoryPtr;
        MapHelperPtr mMapHelperPtr;
        double mAngleThreshold;
        double mOmegaUtility, mOmegaPan, mOmegaTilt;
        double mOmegaRot, mOmegaBase, mOmegaRecognizer;
        bool mUseOrientationUtility, mUseProximityUtility, mUseSideUtility;

    public:
        DefaultRatingModuleFactory(double fovx, double fovy,
                                   double fcp, double ncp,
                                   double useTargetRobotState,
                                   robot_model_services::RobotModelAbstractFactoryPtr robotModelFactory, CameraModelFilterAbstractFactoryPtr cameraModelFilterFactory,
				   MapHelperPtr mapHelper,
                                   double angleThreshold,
                                   double omegaUtility, double omegaPan, double omegaTilt,
                                   double omegaRot, double omegaBase, double omegaRecognizer,
                                   bool useOrientationUtility, bool useProximityUtility, bool useSideUtility)
            : mFovx(fovx), mFovy(fovy),
              mFcp(fcp), mNcp(ncp),
              mUseTargetRobotState(useTargetRobotState),
              mRobotModelFactoryPtr(robotModelFactory), mCameraModelFilterFactoryPtr(cameraModelFilterFactory),
              mMapHelperPtr(mapHelper),
              mAngleThreshold(angleThreshold),
              mOmegaUtility(omegaUtility), mOmegaPan(omegaPan), mOmegaTilt(omegaTilt),
              mOmegaRot(omegaRot), mOmegaBase(omegaBase), mOmegaRecognizer(omegaRecognizer),
              mUseOrientationUtility(useOrientationUtility), mUseProximityUtility(useProximityUtility), mUseSideUtility(useSideUtility)
        { }

        RatingModulePtr createRatingModule() {
            DefaultRatingModulePtr defaultRatingModule = DefaultRatingModulePtr(new DefaultRatingModule(mFovx, mFovy, mFcp, mNcp, mUseTargetRobotState, mRobotModelFactoryPtr->createRobotModel(), mMapHelperPtr, mCameraModelFilterFactoryPtr->createCameraModelFilter()));
            defaultRatingModule->setNormalAngleThreshold(mAngleThreshold);
            defaultRatingModule->setOmegaParameters(mOmegaUtility, mOmegaPan, mOmegaTilt, mOmegaRot, mOmegaBase, mOmegaRecognizer);
            defaultRatingModule->setUtilityParameters(mUseOrientationUtility, mUseProximityUtility, mUseSideUtility);
            return defaultRatingModule;
        }
    };
    typedef boost::shared_ptr<DefaultRatingModuleFactory> DefaultRatingModuleFactoryPtr;
}
