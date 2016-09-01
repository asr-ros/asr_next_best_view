/*
 * DefaultHypothesisUpdaterFactory.hpp
 *
 * Created on: Aug 29, 2016
 *      Author: Daniel Stroh
 */

#pragma once

#include "next_best_view/rating/impl/DefaultRatingModuleFactory.hpp"
#include "next_best_view/hypothesis_updater/impl/DefaultHypothesisUpdater.hpp"
#include "next_best_view/hypothesis_updater/HypothesisUpdaterAbstractFactory.hpp"

namespace next_best_view {

    class DefaultHypothesisUpdaterFactory : public HypothesisUpdaterAbstractFactory {
    private:

    public:
        DefaultHypothesisUpdaterFactory()
        { }

        HypothesisUpdaterPtr createHypothesisUpdater() {
            return DefaultHypothesisUpdaterPtr(new DefaultHypothesisUpdater());
        }
    };
    typedef boost::shared_ptr<DefaultHypothesisUpdaterFactory> DefaultHypothesisUpdaterFactoryPtr;
}
