/*
 * DefaultRating.cpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/rating/impl/DefaultScoreContainer.hpp"

namespace next_best_view {
    DefaultScoreContainer::DefaultScoreContainer() : BaseScoreContainer(), mMovementCostsBaseTranslation(0.0),
                                                                            mMovementCostsBaseRotation(0.0),
                                                                            mMovementCostsPTU(0.0),
                                                                            mRecognitionCosts(0.0) {
	}

	DefaultScoreContainer::~DefaultScoreContainer() { }
}
