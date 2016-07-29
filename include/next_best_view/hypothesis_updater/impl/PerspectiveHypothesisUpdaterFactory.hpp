/*
 * PerspectiveHypothesisUpdaterFactory.hpp
 *
 * Created on: Aug 29, 2016
 *      Author: Daniel Stroh
 */

#pragma once

#include "next_best_view/rating/impl/DefaultRatingModuleFactory.hpp"
#include "next_best_view/hypothesis_updater/impl/PerspectiveHypothesisUpdater.hpp"
#include "next_best_view/hypothesis_updater/HypothesisUpdaterAbstractFactory.hpp"

namespace next_best_view {

    class PerspectiveHypothesisUpdaterFactory : public HypothesisUpdaterAbstractFactory {
    private:
        DefaultRatingModuleFactoryPtr defaultRatingModuleFactory;

    public:
        PerspectiveHypothesisUpdaterFactory(DefaultRatingModuleFactoryPtr defaultRatingModuleFactory)
            : defaultRatingModuleFactory(defaultRatingModuleFactory)
        { }

        HypothesisUpdaterPtr createHypothesisUpdater() {
            PerspectiveHypothesisUpdaterPtr perspectiveHypothesisUpdater = PerspectiveHypothesisUpdaterPtr(new PerspectiveHypothesisUpdater());
            DefaultRatingModulePtr defaultRatingModule = boost::static_pointer_cast<DefaultRatingModule>(defaultRatingModuleFactory->createRatingModule());
            perspectiveHypothesisUpdater->setDefaultRatingModule(defaultRatingModule);
            return perspectiveHypothesisUpdater;
        }
    };
    typedef boost::shared_ptr<PerspectiveHypothesisUpdaterFactory> PerspectiveHypothesisUpdaterFactoryPtr;
}
