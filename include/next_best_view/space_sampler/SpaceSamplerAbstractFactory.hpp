/*
 * SpaceSamplerAbstractFactory.hpp
 *
 * Created on: Aug 29, 2016
 *      Author: Daniel Stroh
 */

#pragma once

#include "next_best_view/space_sampler/SpaceSampler.hpp"

namespace next_best_view {

    class SpaceSamplerAbstractFactory {

    public:
        virtual SpaceSamplerPtr createSpaceSampler() = 0;
    };
    typedef boost::shared_ptr<SpaceSamplerAbstractFactory> SpaceSamplerAbstractFactoryPtr;
}
