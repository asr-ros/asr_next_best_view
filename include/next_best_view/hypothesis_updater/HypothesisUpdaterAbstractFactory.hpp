/*
 * HypothesisUpdaterAbstractFactory.hpp
 *
 * Created on: Aug 29, 2016
 *      Author: Daniel Stroh
 */

#pragma once

#include "next_best_view/hypothesis_updater/HypothesisUpdater.hpp"

namespace next_best_view {

    class HypothesisUpdaterAbstractFactory {

    public:
        virtual HypothesisUpdaterPtr createHypothesisUpdater() = 0;
    };
    typedef boost::shared_ptr<HypothesisUpdaterAbstractFactory> HypothesisUpdaterAbstractFactoryPtr;
}
