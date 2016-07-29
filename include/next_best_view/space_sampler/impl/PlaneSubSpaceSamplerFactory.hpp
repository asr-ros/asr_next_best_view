/*
 * PlaneSubSpaceSamplerFactory.hpp
 *
 * Created on: Aug 29, 2016
 *      Author: Daniel Stroh
 */

#pragma once

#include "next_best_view/space_sampler/impl/PlaneSubSpaceSampler.hpp"
#include "next_best_view/space_sampler/SpaceSamplerAbstractFactory.hpp"

namespace next_best_view {

    class PlaneSubSpaceSamplerFactory : public SpaceSamplerAbstractFactory {
    private:

    public:
        PlaneSubSpaceSamplerFactory()
        { }

        SpaceSamplerPtr createSpaceSampler() {
            PlaneSubSpaceSamplerPtr planeSubSpaceSampler = PlaneSubSpaceSamplerPtr(new PlaneSubSpaceSampler());
            return planeSubSpaceSampler;
        }
    };
    typedef boost::shared_ptr<PlaneSubSpaceSamplerFactory> PlaneSubSpaceSamplerFactoryPtr;
}


