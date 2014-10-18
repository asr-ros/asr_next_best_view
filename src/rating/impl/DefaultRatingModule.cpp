/*
 * DefaultRatingModule.cpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/rating/impl/DefaultRatingModule.hpp"
#include "next_best_view/helper/MathHelper.hpp"

namespace next_best_view {
	float DefaultRatingModule::getNormalityRating(const ViewportPoint &viewportPoint, ObjectPoint &objectPoint) {
		float maxRating = 0.0;
		SimpleQuaternion viewportOrientation = viewportPoint.getSimpleQuaternion();
		SimpleVector3 viewportNormalVector = viewportOrientation.toRotationMatrix() * SimpleVector3::UnitX();

		BOOST_FOREACH(SimpleVector3 objectSurfaceNormalVector, *objectPoint.normal_vectors) {
			maxRating = std::max(this->getSingleNormalityRating(viewportNormalVector, objectSurfaceNormalVector, mNormalityRatingAngle), maxRating);
		}

		return maxRating;
	}

	float DefaultRatingModule::getSingleNormalityRating(const SimpleVector3 &viewportNormalVector, const SimpleVector3 &objectSurfaceNormalVector, float angleThreshold) {
		float cosinus = MathHelper::getCosinus(viewportNormalVector, objectSurfaceNormalVector);
		float angle = acos(cosinus);
		return MathHelper::getRatingFunction(angle, angleThreshold);
	}

	float DefaultRatingModule::getSingleNormalityRatingByQuaternion(const SimpleQuaternion &quaternionA, const SimpleQuaternion &quaternionB, float angleThreshold) {
		SimpleVector3 a = quaternionA.toRotationMatrix() * SimpleVector3::UnitX();
		SimpleVector3 b = quaternionB.toRotationMatrix() * SimpleVector3::UnitX();

		return getSingleNormalityRating(a, b, angleThreshold);
	}

	float DefaultRatingModule::getWeightedRating(const DefaultScoreContainerPtr &a) {
		static float elementCountWeight = 1.0;
		static float normalityWeight = 1.0;
		static float localityWeight = 1.0;
		static float equalityWeight = 1.25;
		static float angleWeight = 1.0;
		return elementCountWeight * a->element_density + normalityWeight * a->normality + localityWeight * a->locality + equalityWeight * a->equality + angleWeight * a->angle * a->equality;
	}

	DefaultRatingModule::DefaultRatingModule() : RatingModule(), mNormalityRatingAngle(M_PI * .5) {
	}

	DefaultRatingModule::~DefaultRatingModule() { }

	bool DefaultRatingModule::getScoreContainer(const ViewportPoint &currentViewport, const ViewportPoint &candidateViewportPoint, BaseScoreContainerPtr &scoreContainerPtr) {
		SimpleVector3 currentPosition = currentViewport.getSimpleVector3();
		SimpleQuaternion currentOrientation = currentViewport.getSimpleQuaternion();
		SimpleVector3 candidatePosition = candidateViewportPoint.getSimpleVector3();
		SimpleQuaternion candidateOrientation = candidateViewportPoint.getSimpleQuaternion();

		DefaultScoreContainerPtr defRatingPtr(new DefaultScoreContainer());

		BOOST_FOREACH(int index, *this->getIndices()) {
			ObjectPoint &objectPoint = this->getInputCloud()->at(index);

			if (objectPoint.normal_vectors->size() == 0) {
				continue;
			}

			float currentNormalityRating = this->getNormalityRating(candidateViewportPoint, objectPoint);

			if (currentNormalityRating == 0.0) {
				continue;
			}

			defRatingPtr->element_density += 1.0;
			defRatingPtr->normality = std::max(defRatingPtr->normality, currentNormalityRating);
		}

		defRatingPtr->locality = (currentPosition - candidatePosition).lpNorm<2>();
		defRatingPtr->equality = (currentPosition == candidatePosition ? 1.0 : 0.0);
		defRatingPtr->angle = getSingleNormalityRatingByQuaternion(currentOrientation, candidateOrientation);

		scoreContainerPtr = defRatingPtr;

		return (defRatingPtr->element_density > 0.0);
	}

	void DefaultRatingModule::normalizeScoreContainer(const BaseScoreContainerPtr &normalizingScoreContainerPtr, BaseScoreContainerPtr &subjectScoreContainerPtr) {
		// cast defScoreContainerPtr to used defScoreContainerPtr type
		DefaultScoreContainerPtr defScoreContainerPtr = boost::static_pointer_cast<DefaultScoreContainer>(subjectScoreContainerPtr);
		// get the normalizingScoreContainerPtr
		DefaultScoreContainerPtr defNormalizerScoreContainerPtr = boost::static_pointer_cast<DefaultScoreContainer>(normalizingScoreContainerPtr);

		defScoreContainerPtr->element_density /= defNormalizerScoreContainerPtr->element_density;
		defScoreContainerPtr->locality = (defNormalizerScoreContainerPtr->locality == 0 ? 1.0 : 1.0 - (defScoreContainerPtr->locality / defNormalizerScoreContainerPtr->locality));
		defScoreContainerPtr->normality /= defNormalizerScoreContainerPtr->normality;
	}

	void DefaultRatingModule::maximizeScoreContainer(const BaseScoreContainerPtr &scoreContainerPtr, BaseScoreContainerPtr &currentMaximumScoreContainerPtr) {
		// cast rating to used rating type
		DefaultScoreContainerPtr curMax = boost::static_pointer_cast<DefaultScoreContainer>(currentMaximumScoreContainerPtr);
		// get the normalizer
		DefaultScoreContainerPtr ratingPtr = boost::static_pointer_cast<DefaultScoreContainer>(scoreContainerPtr);

		curMax->normality = std::max(curMax->normality, ratingPtr->normality);
		curMax->locality = std::max(curMax->locality, ratingPtr->locality);
		curMax->element_density = std::max(curMax->element_density, ratingPtr->element_density);
	}

	bool DefaultRatingModule::compareScoreContainer(const BaseScoreContainerPtr &a, const BaseScoreContainerPtr &b) {
		DefaultScoreContainerPtr defA = boost::static_pointer_cast<DefaultScoreContainer>(a);
		DefaultScoreContainerPtr defB = boost::static_pointer_cast<DefaultScoreContainer>(b);

		return this->getWeightedRating(defA) < this->getWeightedRating(defB);
	}

	void DefaultRatingModule::updateObjectPoints(const ViewportPoint &viewportPoint) {
		SimpleVector3 viewportNormalVector = viewportPoint.getSimpleQuaternion().toRotationMatrix() * SimpleVector3::UnitX();

		BOOST_FOREACH(int index, *viewportPoint.child_indices) {
			ObjectPoint &objectPoint = viewportPoint.child_point_cloud->at(index);

			SimpleVector3CollectionPtr normalVectorCollectionPtr(new SimpleVector3Collection());
			BOOST_FOREACH(SimpleVector3 normalVector, *objectPoint.normal_vectors) {
				float rating = this->getSingleNormalityRating(viewportNormalVector, normalVector, mNormalityRatingAngle);
				if (rating == 0.0) {
					normalVectorCollectionPtr->push_back(normalVector);
				}
			}
			objectPoint.normal_vectors = normalVectorCollectionPtr;
		}
	}

	BaseScoreContainerPtr DefaultRatingModule::getScoreContainerInstance() {
		return BaseScoreContainerPtr(new DefaultScoreContainer());
	}

	void DefaultRatingModule::setNormalityRatingAngle(double angle) {
		mNormalityRatingAngle = angle;
	}

	double DefaultRatingModule::getNormalityRatingAngle() {
		return mNormalityRatingAngle;
	}
}

