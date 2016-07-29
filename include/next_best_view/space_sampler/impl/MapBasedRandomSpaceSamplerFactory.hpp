/*
 * MapBasedRandomSpaceSamplerFactory.hpp
 *
 * Created on: Aug 29, 2016
 *      Author: Daniel Stroh
 */

#pragma once

#include "next_best_view/helper/MapHelperFactory.hpp"
#include "next_best_view/space_sampler/impl/MapBasedRandomSpaceSampler.hpp"
#include "next_best_view/space_sampler/SpaceSamplerAbstractFactory.hpp"

namespace next_best_view {

    class MapBasedRandomSpaceSamplerFactory : public SpaceSamplerAbstractFactory {
    private:
        MapHelperFactoryPtr mapHelperFactory;
        int sampleSize;

    public:
        MapBasedRandomSpaceSamplerFactory(MapHelperFactoryPtr mapHelperFactory, int sampleSize)
            : mapHelperFactory(mapHelperFactory),
              sampleSize(sampleSize)
        { }

        SpaceSamplerPtr createSpaceSampler() {
            MapBasedRandomSpaceSamplerPtr mapBasedRandomSpaceSampler = MapBasedRandomSpaceSamplerPtr(new MapBasedRandomSpaceSampler(mapHelperFactory->createMapHelper(), sampleSize));
            return mapBasedRandomSpaceSampler;
        }
    };
    typedef boost::shared_ptr<MapBasedRandomSpaceSamplerFactory> MapBasedRandomSpaceSamplerFactoryPtr;
}

