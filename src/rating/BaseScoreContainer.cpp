/*
 * Rating.cpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/rating/BaseScoreContainer.hpp"

namespace next_best_view {
	BaseScoreContainer::BaseScoreContainer() : mUtility(0), mInverseCosts(0) { }
	BaseScoreContainer::~BaseScoreContainer() { }
}

