/*
 * DefaultRating.cpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/rating/impl/DefaultScoreContainer.hpp"
#include <ostream>

namespace next_best_view {
    DefaultScoreContainer::DefaultScoreContainer() : BaseScoreContainer(), mMovementCostsBaseTranslation(0.0),
                                                                            mMovementCostsBaseRotation(0.0),
                                                                            mMovementCostsPTU(0.0),
                                                                            mRecognitionCosts(0.0) {
	}

	DefaultScoreContainer::~DefaultScoreContainer() { }

    std::ostream& operator<<(std::ostream &strm, const next_best_view::DefaultScoreContainer &score) {
        using std::endl;
        strm << "UnweightedInverseMovementCostsBaseTranslation: " << score.getUnweightedInverseMovementCostsBaseTranslation() << endl
             << "unweightedInverseMovementCostsBaseRotation: " << score.getUnweightedInverseMovementCostsBaseRotation() << endl
             << "unweightedInverseMovementCostsPTU: " << score.getUnweightedInverseMovementCostsPTU() << endl
             << "unweightedInverseRecognitionCosts: " << score.getUnweightedInverseRecognitionCosts() << endl
             << "utilityNormalization: " << score.getUtilityNormalization() << endl
             << "weightedUnnormalizedUtility: " << score.getWeightedUnnormalizedUtility() << endl
             << "weightedNormalizedUtility: " << score.getWeightedNormalizedUtility() << endl
             << "weightedInverseCosts: " << score.getWeightedInverseCosts() << endl
             << "objectUtility: {" << endl;
        for (auto objType : *score.getObjectTypes()) {
            strm << objType << " -> " << score.getUnweightedUnnormalizedObjectUtility(objType) << endl;
        }
        return strm << "}" << endl;
    }

    std::ostream& operator<<(std::ostream &strm, const next_best_view::DefaultScoreContainerPtr &score) {
        return strm << *score;
    }
}
