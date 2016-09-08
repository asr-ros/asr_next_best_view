/*
 * MapBasedHexagonSpaceSamplerFactory.hpp
 *
 * Created on: Aug 29, 2016
 *      Author: Daniel Stroh
 */

#pragma once

#include "next_best_view/helper/MapHelperFactory.hpp"
#include "next_best_view/space_sampler/impl/MapBasedHexagonSpaceSampler.hpp"
#include "next_best_view/space_sampler/SpaceSamplerAbstractFactory.hpp"

namespace next_best_view {

    class MapBasedHexagonSpaceSamplerFactory : public SpaceSamplerAbstractFactory {
    private:
        MapHelperFactoryPtr mapHelperFactory;
        double radius;

    public:
        MapBasedHexagonSpaceSamplerFactory(MapHelperFactoryPtr mapHelperFactory, double radius)
            : mapHelperFactory(mapHelperFactory),
              radius(radius)
        { }

        SpaceSamplerPtr createSpaceSampler() {
            MapBasedHexagonSpaceSamplerPtr mapBasedHexagonSpaceSampler = MapBasedHexagonSpaceSamplerPtr(new MapBasedHexagonSpaceSampler(mapHelperFactory->createMapHelper()));
            mapBasedHexagonSpaceSampler->setHexagonRadius(radius);
            return mapBasedHexagonSpaceSampler;
        }
    };
    typedef boost::shared_ptr<MapBasedHexagonSpaceSamplerFactory> MapBasedHexagonSpaceSamplerFactoryPtr;
}


