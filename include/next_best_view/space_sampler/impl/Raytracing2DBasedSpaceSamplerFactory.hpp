/*
 * Raytracing2DBasedSpaceSamplerFactory.hpp
 *
 * Created on: Aug 29, 2016
 *      Author: Daniel Stroh
 */

#pragma once

#include "next_best_view/helper/MapHelperFactory.hpp"
#include "next_best_view/space_sampler/impl/Raytracing2DBasedSpaceSampler.hpp"
#include "next_best_view/space_sampler/SpaceSamplerAbstractFactory.hpp"

namespace next_best_view {

    class Raytracing2DBasedSpaceSamplerFactory : public SpaceSamplerAbstractFactory {
    private:
        MapHelperFactoryPtr mapHelperFactory;

    public:
        Raytracing2DBasedSpaceSamplerFactory(MapHelperFactoryPtr mapHelperFactory)
            : mapHelperFactory(mapHelperFactory)
        { }

        SpaceSamplerPtr createSpaceSampler() {
            MapBasedSpaceSamplerPtr raytracing2DBasedSpaceSampler = MapBasedSpaceSamplerPtr(new Raytracing2DBasedSpaceSampler(mapHelperFactory->createMapHelper()));
            return raytracing2DBasedSpaceSampler;
        }
    };
    typedef boost::shared_ptr<Raytracing2DBasedSpaceSamplerFactory> Raytracing2DBasedSpaceSamplerFactoryPtr;
}

