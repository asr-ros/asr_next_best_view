/*
 * RatingModuleAbstractFactory.hpp
 *
 * Created on: Aug 29, 2016
 *      Author: Daniel Stroh
 */

#pragma once

#include "next_best_view/rating/RatingModule.hpp"

namespace next_best_view {

    class RatingModuleAbstractFactory {

    public:
        virtual RatingModulePtr createRatingModule() = 0;
    };
    typedef boost::shared_ptr<RatingModuleAbstractFactory> RatingModuleAbstractFactoryPtr;
}
