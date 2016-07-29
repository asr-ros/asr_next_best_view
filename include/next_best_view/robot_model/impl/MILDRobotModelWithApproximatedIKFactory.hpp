/*
 * MILDRobotModelWithApproximatedIKFactory.hpp
 *
 * Created on: Aug 29, 2016
 *      Author: Daniel Stroh
 */

#pragma once

#include "next_best_view/robot_model/impl/MILDRobotModelWithApproximatedIK.hpp"
#include "next_best_view/robot_model/RobotModelAbstractFactory.hpp"

namespace next_best_view {

    class MILDRobotModelWithApproximatedIKFactory : public RobotModelAbstractFactory {
    private:
        float minTiltAngleDegrees, maxTiltAngleDegrees;
        float minPanAngleDegrees, maxPanAngleDegrees;

    public:
        MILDRobotModelWithApproximatedIKFactory(float minTiltAngleDegrees, float maxTiltAngleDegrees,
                                                float minPanAngleDegrees, float maxPanAngleDegrees)
            : minTiltAngleDegrees(minTiltAngleDegrees),
              maxTiltAngleDegrees(maxTiltAngleDegrees),
              minPanAngleDegrees(minPanAngleDegrees),
              maxPanAngleDegrees(maxPanAngleDegrees)
        { }

        RobotModelPtr createRobotModel() {
            MILDRobotModel* robotModel = new MILDRobotModelWithApproximatedIK();
            robotModel->setPanAngleLimits(minPanAngleDegrees, maxPanAngleDegrees);
            robotModel->setTiltAngleLimits(minTiltAngleDegrees, maxTiltAngleDegrees);
            return RobotModelPtr(robotModel);
        }
    };
    typedef boost::shared_ptr<MILDRobotModelWithApproximatedIKFactory> MILDRobotModelWithApproximatedIKFactoryPtr;
}
