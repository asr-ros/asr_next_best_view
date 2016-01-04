/*
 * DefaultRatingModule.cpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/rating/impl/DefaultRatingModule.hpp"

namespace next_best_view {

    DefaultRatingModule::DefaultRatingModule() : RatingModule(), mNormalAngleThreshold(M_PI * .5) { }

    DefaultRatingModule::~DefaultRatingModule() { }

    void DefaultRatingModule::setInputCloud(const ObjectPointCloudPtr &pointCloudPtr) {
        // set input cloud in parent class
        RatingModule::setInputCloud(pointCloudPtr);

        mInputCloudChanged = true;
    }

    bool DefaultRatingModule::setBestScoreContainer(const ViewportPoint &currentViewport, ViewportPoint &candidateViewport) {
        // reset the cached data
        this->resetCache();

        // get the power set of the object name set
        ObjectNameSetPtr objectNameSetPtr = candidateViewport.object_name_set;
        ObjectNamePowerSetPtr powerSetPtr = MathHelper::powerSet<ObjectNameSet> (objectNameSetPtr);

        //Create list of all viewports
        ViewportPointCloudPtr viewports = ViewportPointCloudPtr(new ViewportPointCloud());

        // do the filtering for each combination of object types
        for (std::set<ObjectNameSetPtr>::iterator subSetIter = powerSetPtr->begin(); subSetIter != powerSetPtr->end(); ++subSetIter) {
            ViewportPoint viewport;
            if (!candidateViewport.filterObjectNames(*subSetIter, viewport)) {
                continue;
            }

            // if the rating is not feasible: skip adding to point cloud
            if (!this->setSingleScoreContainer(currentViewport, viewport)) {
                continue;
            }

            if (!viewport.score) {
                ROS_ERROR("Single score result is nullpointer.");
            }

            viewports->push_back(viewport);
        }

        if (!this->getBestViewport(viewports, candidateViewport)) {
            return false;
        }

        return true;
    }

    bool DefaultRatingModule::getBestViewport(const ViewportPointCloudPtr &viewports, ViewportPoint &bestViewport) {
        // if there aren't any viewports, the search failed.
        ROS_DEBUG_STREAM("Viewport Point size : " << viewports->size());
        if (viewports->size() == 0) {
            return false;
        }

        ViewportPointCloud::iterator maxElement = std::max_element(viewports->begin(), viewports->end(), boost::bind(&RatingModule::compareViewports, *this, _1, _2));

        if (maxElement == viewports->end()) {
            ROS_DEBUG_STREAM("No maximum element found.");
            return false;
        }

        bestViewport = *maxElement;

        ROS_DEBUG_STREAM("bestViewport position: (" << bestViewport.getPosition()(0,0) << ","
                            << bestViewport.getPosition()(1,0) << "," << bestViewport.getPosition()(2,0) << ")"
                            << " rating: " << this->getRating(bestViewport.score));

        return true;
    }

    bool DefaultRatingModule::compareViewports(ViewportPoint &a, ViewportPoint &b) {
        return this->compareScoreContainer(a.score, b.score);
    }

    bool DefaultRatingModule::compareScoreContainer(const BaseScoreContainerPtr &a, const BaseScoreContainerPtr &b) {
        if (!a) {
            ROS_ERROR("Score container a is nullpointer");
        }

        if (!b) {
            ROS_ERROR("Score container b is nullpointer");
        }

        Precision ratingA = this->getRating(a);
        Precision ratingB = this->getRating(b);

        return ratingA < ratingB;
    }

    BaseScoreContainerPtr DefaultRatingModule::getScoreContainerInstance() {
        return BaseScoreContainerPtr(new DefaultScoreContainer());
    }

    float DefaultRatingModule::getOrientationUtility(const ViewportPoint &viewport, ObjectPoint &objectPoint) {
        float maxUtility= 0.0;

        // check the utilities for each normal and pick the best
        BOOST_FOREACH(int index, *objectPoint.active_normal_vectors) {
            SimpleVector3 objectNormalVector = objectPoint.normal_vectors->at(index);
            maxUtility = std::max(this->getNormalUtility(viewport, objectNormalVector), maxUtility);
		}

        return maxUtility;
	}

    float DefaultRatingModule::getNormalUtility(const ViewportPoint &viewport, const SimpleVector3 &objectNormalVector) {
        SimpleQuaternion cameraOrientation = viewport.getSimpleQuaternion();
        SimpleVector3 cameraOrientationVector = MathHelper::getVisualAxis(cameraOrientation);

        // rate the angle between the camera orientation and the object normal
        float angle = MathHelper::getAngle(-cameraOrientationVector, objectNormalVector);
        float utility = this->getNormalizedRating(angle, mNormalAngleThreshold);

        return utility;
	}

    float DefaultRatingModule::getProximityUtility(const ViewportPoint &viewport, const ObjectPoint &objectPoint) {

        SimpleVector3 cameraPosition = viewport.getPosition();
        SimpleQuaternion cameraOrientation = viewport.getSimpleQuaternion();
        SimpleVector3 cameraOrientationVector = MathHelper::getVisualAxis(cameraOrientation);
        SimpleVector3 objectPosition = objectPoint.getPosition();

        SimpleVector3 objectToCameraVector = cameraPosition - objectPosition;

        // project the object to the camera orientation vector in order to determine the distance to the mid
        float projection = MathHelper::getDotProduct(-cameraOrientationVector, objectToCameraVector);
        ROS_DEBUG_STREAM("projection value" << projection << " fcp " << fcp << " ncp " << ncp);

        // determine the distance of the object to the mid of the frustum
        float distanceToMid = abs(projection-(fcp+ncp)/2.0);
        float distanceThreshold = (fcp-ncp)/2.0;
        ROS_DEBUG_STREAM("distance to mid " << distanceToMid << " thresh "  << distanceThreshold );

        float utility = this->getNormalizedRating(distanceToMid, distanceThreshold);

        return utility;
    }


    float DefaultRatingModule::getFrustumPositionUtility(const ViewportPoint &viewport, ObjectPoint &objectPoint)
    {
        SimpleVector3 cameraPosition = viewport.getPosition();
        SimpleQuaternion cameraOrientation = viewport.getSimpleQuaternion();
        SimpleVector3 cameraOrientationVector = MathHelper::getVisualAxis(cameraOrientation);

        float angleMin = (float) MathHelper::degToRad(std::min(fovV,fovH));
        SimpleVector3 objectPosition = objectPoint.getPosition();
        SimpleVector3 objectToCameraVector = cameraPosition - objectPosition;
        SimpleVector3 objectToCameraVectorNormalized = objectToCameraVector.normalized();

        // utility for how far the object is on the side of the camera view
        float angle = MathHelper::getAngle(-cameraOrientationVector, objectToCameraVectorNormalized);
        float sideUtility = this->getNormalizedRating(angle, angleMin);

        // utility for how far the object is away from the camera
        float proximityUtility = this->getProximityUtility(viewport,objectPoint);

        // the complete frumstum position utility
        float utility = sideUtility * proximityUtility;

        ROS_DEBUG_STREAM("Frustum side utility "<< sideUtility);
        ROS_DEBUG_STREAM("Frustum proximity utility  " << proximityUtility);
        return utility;
    }

    float DefaultRatingModule::getRating(const BaseScoreContainerPtr &a) {
        if (!a) {
            ROS_ERROR("Score container is nullpointer");
        }

        float result = a->getUtility() * a->getCosts();

        ROS_DEBUG_STREAM("rating: " << result << " utility: " << a->getUtility() << " costs: " << a->getCosts());

        return result;
	}

    bool DefaultRatingModule::setSingleScoreContainer(const ViewportPoint &currentViewport,
                                                        ViewportPoint &candidateViewport) {
        DefaultScoreContainerPtr defRatingPtr(new DefaultScoreContainer());

        // set the utility
        float utility = this->getUtility(candidateViewport);

        if (utility <= 0) {
            return false;
        }

        defRatingPtr->setUtility(utility);

        // set the ratings for the orientations and the positions of the objects in the candidate viewport
        float orientationUtility = this->getFullOrientationUtility(candidateViewport);
        float positionUtility = this->getFullPositionUtility(candidateViewport);

        defRatingPtr->setOrientationUtility(orientationUtility);
        defRatingPtr->setPositionUtility(positionUtility);

        // set the costs
        double costs = this->getCosts(currentViewport, candidateViewport);
        defRatingPtr->setCosts(costs);

        candidateViewport.score = defRatingPtr;
        ROS_DEBUG("Utility %f, Orientation utility %f, Position utility %f", utility, orientationUtility, positionUtility);
        return true;
    }

    void DefaultRatingModule::setNormalAngleThreshold(double angle) {
        mNormalAngleThreshold = angle;
	}

    double DefaultRatingModule::getNormalAngleThreshold() {
        return mNormalAngleThreshold;
	}

    void DefaultRatingModule::setOmegaParameters(double omegaPan, double omegaTilt, double omegaRot, double omegaBase, double omegaRecognition) {
        this->mOmegaPan = omegaPan;
        this->mOmegaTilt = omegaTilt;
        this->mOmegaRot = omegaRot;
        this->mOmegaBase = omegaBase;
        this->mOmegaRecognition = omegaRecognition;
    }

    float DefaultRatingModule::getNormalizedRating(float deviation, float threshold) {
        if (deviation < threshold) {
            return .5 + .5 * cos(deviation * M_PI / threshold);
        }
        return 0.0;
    }

    double DefaultRatingModule::getUtility(const ViewportPoint &candidateViewport) {
        double utility = 0.0;

        // get the utility for each object type and sum them up
        BOOST_FOREACH(string objectType, *(candidateViewport.object_name_set)) {
            // set the utility for the object type if not already done
            if (mObjectUtilities.count(objectType) == 0) {
                setObjectUtilities(candidateViewport, objectType);
            }

            // if one of the objects has a utility of 0, it shouldn't be searched at all
            if (mObjectUtilities[objectType] == 0) {
                return 0;
            }

            utility += mObjectUtilities[objectType];
        }

        return utility;
    }

    double DefaultRatingModule::getFullOrientationUtility(const ViewportPoint &candidateViewport) {
        double orientationUtility = 0.0;

        // get the orientation utility for each object type and sum them up
        BOOST_FOREACH(string objectType, *(candidateViewport.object_name_set)) {
            // set the orientation utility for the object type if not already done
            if (mObjectOrientationUtilities.count(objectType) == 0) {
                setObjectUtilities(candidateViewport, objectType);
            }

            orientationUtility += mObjectOrientationUtilities[objectType];
        }

        return orientationUtility;
    }

    double DefaultRatingModule::getFullPositionUtility(const ViewportPoint &candidateViewport) {
        double positionUtility = 0.0;

        // get the orientation utility for each object type and sum them up
        BOOST_FOREACH(string objectType, *(candidateViewport.object_name_set)) {
            // set the orientation utility for the object type if not already done
            if (mObjectPositionUtilities.count(objectType) == 0) {
                setObjectUtilities(candidateViewport, objectType);
            }

            positionUtility += mObjectPositionUtilities[objectType];
        }

        return positionUtility;
    }

    void DefaultRatingModule::setObjectUtilities(const ViewportPoint &candidateViewport, string objectType) {
        float orientationUtility = 0.0;
        float positionUtility = 0.0;

        double maxElements = this->getInputCloud()->size();

        // build the sum of the orientation and frustum position utilities of all object points in the candidate camera view with the given type
        BOOST_FOREACH(int index, *(candidateViewport.child_indices)) {
            ObjectPoint &objectPoint = this->getInputCloud()->at(index);

            if (objectPoint.type != objectType) {
                continue;
            }

            if (objectPoint.active_normal_vectors->size() == 0) {
                continue;
            }

            float currentOrientationUtility = this->getOrientationUtility(candidateViewport, objectPoint);
            float currentFrustumPositionUtility = this->getFrustumPositionUtility(candidateViewport, objectPoint);

            // TODO calculate utility here and sum it up
            orientationUtility += currentOrientationUtility;
            positionUtility += currentFrustumPositionUtility;
        }

        // TODO normalize utility ( /= maxElements)

        // set the orientation and position utilities in relation to the amount of object points
        orientationUtility /= maxElements;
        positionUtility /= maxElements;

        // set the utility
        float utility = orientationUtility * positionUtility;

        // cache the utility and the orientation and position utilities
        mObjectUtilities[objectType] = utility;
        mObjectOrientationUtilities[objectType] = orientationUtility;
        mObjectPositionUtilities[objectType] = positionUtility;
    }

    double DefaultRatingModule::getCosts(const ViewportPoint &sourceViewport,
                                            const ViewportPoint &targetViewport) {
        // set the cache members needed for the costs if not already done
        if (!mTargetState) {
            this->setRobotState(sourceViewport, targetViewport);
        }

        if (mOmegaPTU < 0) {
            this->setOmegaPTU();
        }

        if (mMovementCosts < 0) {
            this->setMovementCosts();
        }

        if (mCostsNormalization < 0) {
            this->setCostsNormalization();
        }

        if (mInputCloudChanged) {
            this->setMaxRecognitionCosts();
        }

        // TODO save and return in srv
        // get the costs for the recoginition of the objects
        double normalizedRecognitionCosts = this->getNormalizedRecognitionCosts(targetViewport);

        // calculate the full movement costs
        double fullCosts = (mMovementCosts + normalizedRecognitionCosts * mOmegaRecognition) / mCostsNormalization;

        return fullCosts;
    }

    double DefaultRatingModule::getNormalizedRecognitionCosts(const ViewportPoint &targetViewport) {
        // avoid dividing by 0
        if (mMaxRecognitionCosts == 0) {
            // TODO throw exception
            return 1.0;
        }

        // get the costs for the recoginition of each object type
        Precision recognitionCosts = 0;
        BOOST_FOREACH(string objectName, *(targetViewport.object_name_set)) {
            recognitionCosts += mCameraModelFilterPtr->getRecognizerCosts(objectName);
        }
        double normalizedRecognitionCosts = 1.0 - recognitionCosts / mMaxRecognitionCosts;

        ROS_DEBUG_STREAM("Recognitioncosts: " << recognitionCosts);

        return normalizedRecognitionCosts;
    }

    void DefaultRatingModule::setRobotState(const ViewportPoint &sourceViewport, const ViewportPoint &targetViewport) {
        // set the source robot state
        RobotStatePtr sourceState = mRobotModelPtr->calculateRobotState(sourceViewport.getPosition(), sourceViewport.getSimpleQuaternion());
        mRobotModelPtr->setCurrentRobotState(sourceState);

        // set the target robot state
        mTargetState = mRobotModelPtr->calculateRobotState(targetViewport.getPosition(), targetViewport.getSimpleQuaternion());
    }

    void DefaultRatingModule::setOmegaPTU() {
        Precision movementCostsPTU_Pan = mRobotModelPtr->getPTU_PanMovementCosts(mTargetState);
        Precision movementCostsPTU_Tilt = mRobotModelPtr->getPTU_TiltMovementCosts(mTargetState);

        // set the PTU movement omega parameter
        if (movementCostsPTU_Tilt*mOmegaTilt > movementCostsPTU_Pan*mOmegaPan)
        {
            mOmegaPTU = mOmegaPan;
        }
        else
        {
            mOmegaPTU = mOmegaTilt;
        }
    }

    void DefaultRatingModule::setMovementCosts() {
        // TODO save each movement cost and return in srv
        // get the different movement costs
        Precision movementCostsBase_Translation = mRobotModelPtr->getBase_TranslationalMovementCosts(mTargetState);
        Precision movementCostsBase_Rotation = mRobotModelPtr->getBase_RotationalMovementCosts(mTargetState);
        Precision movementCostsPTU;

        if (mOmegaPTU == mOmegaPan) {
            movementCostsPTU = mRobotModelPtr->getPTU_PanMovementCosts(mTargetState);
        }
        else {
            movementCostsPTU = mRobotModelPtr->getPTU_TiltMovementCosts(mTargetState);
        }

        ROS_DEBUG_STREAM("PTU Costs Tilt: " << movementCostsPTU);

        // set the movement costs
        mMovementCosts = movementCostsBase_Translation * mOmegaBase + movementCostsPTU * mOmegaPTU
                            + movementCostsBase_Rotation * mOmegaRot;

        ROS_DEBUG_STREAM("Movement; " << movementCostsBase_Translation << ", rotation " << movementCostsBase_Rotation << ", movement PTU: " << movementCostsPTU);
    }

    void DefaultRatingModule::setCostsNormalization() {
        mCostsNormalization = mOmegaBase + mOmegaPTU + mOmegaRot + mOmegaRecognition;
    }

    void DefaultRatingModule::setMaxRecognitionCosts() {
        mMaxRecognitionCosts = 0;
        ObjectPointCloudPtr inputCloud = this->getInputCloud();

        double maxRecognitionCosts = 0;
        BOOST_FOREACH(ObjectPoint objectPoint, *(inputCloud)) {
            // TODO only check each type once
            maxRecognitionCosts += mCameraModelFilterPtr->getRecognizerCosts(objectPoint.type);
        }
        mMaxRecognitionCosts = maxRecognitionCosts;
        mInputCloudChanged = false;

        ROS_DEBUG_STREAM("maxRecognitionCosts: " << mMaxRecognitionCosts);
    }

    void DefaultRatingModule::resetCache() {
        mObjectUtilities.clear();
        mObjectOrientationUtilities.clear();
        mObjectPositionUtilities.clear();
        mMovementCosts = -1;
        mCostsNormalization = -1;
        mOmegaPTU = -1;
        mTargetState = NULL;
    }
}

