/*
 * MapHelperFactory.hpp
 *
 * Created on: Aug 29, 2016
 *      Author: Daniel Stroh
 */

#pragma once

#include "next_best_view/helper/MapHelper.hpp"

namespace next_best_view {

    class MapHelperFactory {
    private:
        double colThresh;
        std::string mapTopicName, getPlanServiceName;

    public:
        MapHelperFactory(double colThresh, std::string mapTopicName = "map", std::string getPlanServiceName = "move_base/make_plan")
            : colThresh(colThresh),
              mapTopicName(mapTopicName),
              getPlanServiceName(getPlanServiceName)
        { }

        MapHelperPtr createMapHelper() {
            MapHelperPtr mapHelper = MapHelperPtr(new MapHelper(mapTopicName, getPlanServiceName));
            mapHelper->setCollisionThreshold(colThresh);
            return mapHelper;
        }
    };
    typedef boost::shared_ptr<MapHelperFactory> MapHelperFactoryPtr;
}


