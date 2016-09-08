/*
 * MILDRobotModelWithExactIKFactory.hpp
 *
 * Created on: Aug 29, 2016
 *      Author: Daniel Stroh
 */

#pragma once

#include "next_best_view/robot_model/impl/MILDRobotModelWithExactIK.hpp"
#include "next_best_view/robot_model/RobotModelAbstractFactory.hpp"

namespace next_best_view {

    class MILDRobotModelWithExactIKFactory : public RobotModelAbstractFactory {
    private:
        float minTiltAngleDegrees, maxTiltAngleDegrees;
        float minPanAngleDegrees, maxPanAngleDegrees;

    public:
        MILDRobotModelWithExactIKFactory(float minTiltAngleDegrees, float maxTiltAngleDegrees,
                                                float minPanAngleDegrees, float maxPanAngleDegrees)
            : minTiltAngleDegrees(minTiltAngleDegrees),
              maxTiltAngleDegrees(maxTiltAngleDegrees),
              minPanAngleDegrees(minPanAngleDegrees),
              maxPanAngleDegrees(maxPanAngleDegrees)
        { }

        RobotModelPtr createRobotModel() {
            MILDRobotModel* robotModel = new MILDRobotModelWithExactIK();
            robotModel->setPanAngleLimits(minPanAngleDegrees, maxPanAngleDegrees);
            robotModel->setTiltAngleLimits(minTiltAngleDegrees, maxTiltAngleDegrees);
            return RobotModelPtr(robotModel);
        }
    };
    typedef boost::shared_ptr<MILDRobotModelWithExactIKFactory> MILDRobotModelWithExactIKFactoryPtr;
}

