/*
 * DefaultRating.cpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/rating/impl/DefaultScoreContainer.hpp"

namespace next_best_view {
	DefaultScoreContainer::DefaultScoreContainer() : BaseScoreContainer(), mElementDensity(0.0), mNormality(0.0) {
	}

	DefaultScoreContainer::~DefaultScoreContainer() { }
}
