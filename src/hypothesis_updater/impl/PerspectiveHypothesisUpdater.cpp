/*
 * PerspectiveDependantHypothesisUpdater.cpp
 *
 *  Created on: Oct 2, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/hypothesis_updater/impl/PerspectiveHypothesisUpdater.hpp"

namespace next_best_view {
	PerspectiveHypothesisUpdater::PerspectiveHypothesisUpdater() { }
	PerspectiveHypothesisUpdater::~PerspectiveHypothesisUpdater() { }

	void PerspectiveHypothesisUpdater::update(const ViewportPoint &viewportPoint) {
		SimpleVector3 viewportNormalVector = viewportPoint.getSimpleQuaternion().toRotationMatrix() * SimpleVector3::UnitX();

		BOOST_FOREACH(int index, *viewportPoint.child_indices) {
			ObjectPoint &objectPoint = viewportPoint.child_point_cloud->at(index);

			SimpleVector3CollectionPtr normalVectorCollectionPtr(new SimpleVector3Collection());
			BOOST_FOREACH(SimpleVector3 normalVector, *objectPoint.normal_vectors) {
				float rating = mDefaultRatingModulePtr->getSingleNormalityRating(viewportNormalVector, normalVector, mDefaultRatingModulePtr->getNormalityRatingAngle());
				if (rating == 0.0) {
					normalVectorCollectionPtr->push_back(normalVector);
				}
			}
			objectPoint.normal_vectors = normalVectorCollectionPtr;
		}
	}

	void PerspectiveHypothesisUpdater::setDefaultRatingModule(const DefaultRatingModulePtr &defaultRatingModulePtr) {
		mDefaultRatingModulePtr = defaultRatingModulePtr;
	}

	DefaultRatingModulePtr PerspectiveHypothesisUpdater::getDefaultRatingModule() {
		return mDefaultRatingModulePtr;
	}
}
