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
		SimpleVector3 viewportNormalVector = MathHelper::getVisualAxis(viewportOrientation);

		BOOST_FOREACH(int index, *objectPoint.active_normal_vectors) {
			SimpleVector3 objectSurfaceNormalVector = objectPoint.normal_vectors->at(index);
			maxRating = std::max(this->getSingleNormalityRating(viewportNormalVector, objectSurfaceNormalVector, mNormalityRatingAngle), maxRating);
		}

		return maxRating;
	}

	float DefaultRatingModule::getSingleNormalityRating(const SimpleVector3 &viewportNormalVector, const SimpleVector3 &objectSurfaceNormalVector, float angleThreshold) {
		float cosinus = MathHelper::getCosinus(-viewportNormalVector, objectSurfaceNormalVector);
		float angle = acos(cosinus);
		if (angle < angleThreshold) {
			return .5 + .5 * cos(angle * M_PI / angleThreshold);
		}
		return 0.0;
	}

	float DefaultRatingModule::getRating(const DefaultScoreContainerPtr &a) {
		return (a->element_density * a->normality) / (1 + a->costs);
	}

	DefaultRatingModule::DefaultRatingModule() : RatingModule(), mNormalityRatingAngle(M_PI * .5) {
	}

	DefaultRatingModule::~DefaultRatingModule() { }

	bool DefaultRatingModule::getScoreContainer(const ViewportPoint &currentViewport, ViewportPoint &candidateViewportPoint, BaseScoreContainerPtr &scoreContainerPtr) {
		SimpleVector3 currentPosition = currentViewport.getSimpleVector3();
		SimpleQuaternion currentOrientation = currentViewport.getSimpleQuaternion();
		SimpleVector3 candidatePosition = candidateViewportPoint.getSimpleVector3();
		SimpleQuaternion candidateOrientation = candidateViewportPoint.getSimpleQuaternion();

		DefaultScoreContainerPtr defRatingPtr(new DefaultScoreContainer());

		double maxElements = this->getInputCloud()->size();

		BOOST_FOREACH(int index, *candidateViewportPoint.child_indices) {
			ObjectPoint &objectPoint = this->getInputCloud()->at(index);

			if (objectPoint.active_normal_vectors->size() == 0) {
				continue;
			}

			float currentNormalityRating = this->getNormalityRating(candidateViewportPoint, objectPoint);
			defRatingPtr->element_density += 1.0;

			if (currentNormalityRating == 0.0) {
				continue;
			}
			defRatingPtr->normality += currentNormalityRating;
		}

		defRatingPtr->normality /= maxElements;
		defRatingPtr->element_density /= maxElements;

		scoreContainerPtr = defRatingPtr;

		return (defRatingPtr->element_density > 0.0 && defRatingPtr->normality > 0.0);
	}
	bool DefaultRatingModule::compareScoreContainer(const BaseScoreContainerPtr &a, const BaseScoreContainerPtr &b) {
		DefaultScoreContainerPtr defA = boost::static_pointer_cast<DefaultScoreContainer>(a);
		DefaultScoreContainerPtr defB = boost::static_pointer_cast<DefaultScoreContainer>(b);

		return this->getRating(defA) < this->getRating(defB);
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

