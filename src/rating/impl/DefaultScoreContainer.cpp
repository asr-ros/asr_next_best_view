/*
 * DefaultRating.cpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/rating/impl/DefaultScoreContainer.hpp"

namespace next_best_view {
	DefaultScoreContainer::DefaultScoreContainer() : BaseScoreContainer(), element_density(0.0), normality(0.0) {
	}

	DefaultScoreContainer::~DefaultScoreContainer() { }
}
