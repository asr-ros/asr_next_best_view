/*
 * DefaultRatingModule.cpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "next_best_view/rating/impl/DefaultRatingModule.hpp"
#include "next_best_view/helper/MathHelper.hpp"
#include "next_best_view/helper/MarkerHelper.hpp"

namespace next_best_view {

    float DefaultRatingModule::getOrientationRating(const ViewportPoint &cameraViewport, ObjectPoint &objectPoint) {
		float maxRating = 0.0;

        // check the ratings for all normals and pick the best
        BOOST_FOREACH(int index, *objectPoint.active_normal_vectors) {
            SimpleVector3 objectNormalVector = objectPoint.normal_vectors->at(index);
            maxRating = std::max(this->getNormalRating(cameraViewport, objectNormalVector), maxRating);
		}

		return maxRating;
	}

    float DefaultRatingModule::getNormalRating(const ViewportPoint &cameraViewport, const SimpleVector3 &objectNormalVector) {
        SimpleQuaternion cameraOrientation = cameraViewport.getSimpleQuaternion();
        SimpleVector3 cameraOrientationVector = MathHelper::getVisualAxis(cameraOrientation);

        // rate the angle between the camera orientation and the object normal
        float angle = MathHelper::getAngle(-cameraOrientationVector, objectNormalVector);
        float rating = this->getNormalizedRating(angle, mNormalAngleThreshold);

        return rating;
	}

    float DefaultRatingModule::getProximityRating(const ViewportPoint &cameraViewport, const ObjectPoint &objectPoint) {

        SimpleVector3 cameraPosition = cameraViewport.getPosition();
        SimpleQuaternion cameraOrientation = cameraViewport.getSimpleQuaternion();
        SimpleVector3 cameraOrientationVector = MathHelper::getVisualAxis(cameraOrientation);
        SimpleVector3 objectPosition = objectPoint.getPosition();

        SimpleVector3 objectToCameraVector = cameraPosition - objectPosition;

        // project the object to the camera orientation vector in order to determine the distance to the mid
        float projection = MathHelper::getDotProduct(-cameraOrientationVector, objectToCameraVector);
        ROS_DEBUG_STREAM("DotProduct value" << projection << " fcp " << fcp << " ncp " << ncp);

        // determine the distance of the object to the mid of the frustum
        float distanceToMid = abs(projection-(fcp+ncp)/2.0);
        float distanceThreshold = (fcp-ncp)/2.0;
        ROS_DEBUG_STREAM("distance to mid " << distanceToMid << " thresh "  << distanceThreshold );

        float rating = this->getNormalizedRating(distanceToMid, distanceThreshold);

        return rating;
    }


    float DefaultRatingModule::getFrustumPositionRating(const ViewportPoint &cameraViewport, ObjectPoint &objectPoint)
    {
        SimpleVector3 cameraPosition = cameraViewport.getPosition();
        SimpleQuaternion cameraOrientation = cameraViewport.getSimpleQuaternion();
        SimpleVector3 cameraOrientationVector = MathHelper::getVisualAxis(cameraOrientation);

        float angleMin = (float) MathHelper::degToRad(std::min(fovV,fovH));
        SimpleVector3 objectPosition = objectPoint.getPosition();
        SimpleVector3 objectToCameraVector = cameraPosition - objectPosition;
        SimpleVector3 objectToCameraVectorNormalized = objectToCameraVector.normalized();

        // rating for how far the object is on the side of the camera view
        float angle = MathHelper::getAngle(-cameraOrientationVector, objectToCameraVectorNormalized);
        float sideRating = this->getNormalizedRating(angle, angleMin);

        // rating for how far the object is away from the camera
        float proximityRating = this->getProximityRating(cameraViewport,objectPoint);

        // the complete frumstum position rating
        float rating = sideRating * proximityRating;

        ROS_DEBUG_STREAM("Frustum side rating "<< sideRating);
        ROS_DEBUG_STREAM("Frustum proximity rating  " << proximityRating);
        return rating;
    }

	float DefaultRatingModule::getRating(const DefaultScoreContainerPtr &a) {
        return (a->getUtility() * a->getCosts());
	}

    DefaultRatingModule::DefaultRatingModule() : RatingModule(), mNormalAngleThreshold(M_PI * .5) { }

	DefaultRatingModule::~DefaultRatingModule() { }

	bool DefaultRatingModule::getScoreContainer(const ViewportPoint &currentViewport, ViewportPoint &candidateViewportPoint, BaseScoreContainerPtr &scoreContainerPtr) {
        DefaultScoreContainerPtr defRatingPtr(new DefaultScoreContainer());

        float orientationRating = 0.0;
        float positionRating = 0.0;

		double maxElements = this->getInputCloud()->size();

        // build the sum of the orientation and frustum position ratings of all object points in the candidate camera view
		BOOST_FOREACH(int index, *candidateViewportPoint.child_indices) {
			ObjectPoint &objectPoint = this->getInputCloud()->at(index);

			if (objectPoint.active_normal_vectors->size() == 0) {
				continue;
			}

            float currentOrientationRating = this->getOrientationRating(candidateViewportPoint, objectPoint);
            float currentFrustumPositionRating = this->getFrustumPositionRating(candidateViewportPoint, objectPoint);

            orientationRating += currentOrientationRating;
            positionRating += currentFrustumPositionRating;
		}

        // set the orientation and position ratings in relation to the amount of object points
        orientationRating /= maxElements;
        positionRating /= maxElements;

        // set the ratings for the orientations and the positions of the objects in the candidate
        defRatingPtr->setOrientationRating(orientationRating);
        defRatingPtr->setPositionRating(positionRating);

		// set the utility
        defRatingPtr->setUtility(orientationRating * positionRating);

		scoreContainerPtr = defRatingPtr;
        ROS_DEBUG("Orientation %f, Position %f", defRatingPtr->getOrientationRating(), defRatingPtr->getPositionRating());
		return (defRatingPtr->getUtility() > 0);
	}

	bool DefaultRatingModule::compareScoreContainer(const BaseScoreContainerPtr &a, const BaseScoreContainerPtr &b) {
		DefaultScoreContainerPtr defA = boost::static_pointer_cast<DefaultScoreContainer>(a);
		DefaultScoreContainerPtr defB = boost::static_pointer_cast<DefaultScoreContainer>(b);

		Precision ratingA = this->getRating(defA);
		Precision ratingB = this->getRating(defB);

		return ratingA < ratingB;
	}

    void DefaultRatingModule::updateObjectPoints(const ViewportPoint &cameraViewport) {
        BOOST_FOREACH(int index, *cameraViewport.child_indices) {
            ObjectPoint &objectPoint = cameraViewport.child_point_cloud->at(index);

			SimpleVector3CollectionPtr normalVectorCollectionPtr(new SimpleVector3Collection());
			BOOST_FOREACH(SimpleVector3 normalVector, *objectPoint.normal_vectors) {
                float rating = this->getNormalRating(cameraViewport, normalVector);
                // filter all the object normals that are in the given viewport
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

    void DefaultRatingModule::setNormalAngleThreshold(double angle) {
        mNormalAngleThreshold = angle;
	}

    double DefaultRatingModule::getNormalAngleThreshold() {
        return mNormalAngleThreshold;
	}

    float DefaultRatingModule::getNormalizedRating(float deviation, float threshold) {
        if (deviation < threshold) {
            return .5 + .5 * cos(deviation * M_PI / threshold);
        }
        return 0.0;
    }
}

