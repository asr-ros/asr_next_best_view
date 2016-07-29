/*
 * UnitSphereSamplerAbstractFactory.hpp
 *
 * Created on: Aug 29, 2016
 *      Author: Daniel Stroh
 */

#pragma once

#include "next_best_view/unit_sphere_sampler/UnitSphereSampler.hpp"

namespace next_best_view {

    class UnitSphereSamplerAbstractFactory {

    public:
        virtual UnitSphereSamplerPtr createUnitSphereSampler() = 0;
    };
    typedef boost::shared_ptr<UnitSphereSamplerAbstractFactory> UnitSphereSamplerAbstractFactoryPtr;
}
