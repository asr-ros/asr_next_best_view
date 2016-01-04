/*
 * DefaultRating.cpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/rating/impl/DefaultScoreContainer.hpp"

namespace next_best_view {
    DefaultScoreContainer::DefaultScoreContainer() : BaseScoreContainer(), mPositionUtility(0.0), mOrientationUtility(0.0) {
	}

	DefaultScoreContainer::~DefaultScoreContainer() { }
}
