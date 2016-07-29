/*
 * SpiralApproxUnitSphereSamplerFactory.hpp
 *
 * Created on: Aug 29, 2016
 *      Author: Daniel Stroh
 */

#pragma once

#include "next_best_view/unit_sphere_sampler/impl/SpiralApproxUnitSphereSampler.hpp"
#include "next_best_view/unit_sphere_sampler/UnitSphereSamplerAbstractFactory.hpp"

namespace next_best_view {

    class SpiralApproxUnitSphereSamplerFactory : public UnitSphereSamplerAbstractFactory {
    private:
        int sampleSize;

    public:
        SpiralApproxUnitSphereSamplerFactory(int sampleSize)
            : sampleSize(sampleSize)
        { }

        UnitSphereSamplerPtr createUnitSphereSampler() {
            SpiralApproxUnitSphereSamplerPtr sphereSampler = SpiralApproxUnitSphereSamplerPtr(new SpiralApproxUnitSphereSampler());
            sphereSampler->setSamples(sampleSize);
            return sphereSampler;
        }
    };
    typedef boost::shared_ptr<SpiralApproxUnitSphereSamplerFactory> SpiralApproxUnitSphereSamplerFactoryPtr;
}

