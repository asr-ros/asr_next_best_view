/*
 * RobotModelAbstractFactory.hpp
 *
 * Created on: Aug 29, 2016
 *      Author: Daniel Stroh
 */

#pragma once

#include "next_best_view/robot_model/RobotModel.hpp"

namespace next_best_view {

    class RobotModelAbstractFactory {

    public:
        virtual RobotModelPtr createRobotModel() = 0;
    };
    typedef boost::shared_ptr<RobotModelAbstractFactory> RobotModelAbstractFactoryPtr;
}

