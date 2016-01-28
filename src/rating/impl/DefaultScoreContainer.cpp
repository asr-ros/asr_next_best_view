/*
 * DefaultRating.cpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/rating/impl/DefaultScoreContainer.hpp"

namespace next_best_view {
    DefaultScoreContainer::DefaultScoreContainer() : BaseScoreContainer(), mInverseMovementCostsBaseTranslation(0.0),
                                                                            mInverseMovementCostsBaseRotation(0.0),
                                                                            mInverseMovementCostsPTU(0.0),
                                                                            mInverseRecognitionCosts(0.0) {
	}

	DefaultScoreContainer::~DefaultScoreContainer() { }
}
