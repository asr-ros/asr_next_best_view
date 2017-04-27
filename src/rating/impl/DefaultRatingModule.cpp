/**

Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Heller Florian, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Stroh Daniel, Trautmann Jeremias, Walter Milena, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

4. The use is explicitly not permitted to any application which deliberately try to kill or do harm to any living creature.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "next_best_view/rating/impl/DefaultRatingModule.hpp"
#include <robot_model_services/robot_model/impl/MILDRobotState.hpp>

namespace next_best_view {

DefaultRatingModule::DefaultRatingModule() : RatingModule(), mNormalAngleThreshold(M_PI * .5) {
    mDebugHelperPtr = DebugHelper::getInstance();
    if (!mDebugHelperPtr)
        ROS_ERROR("DefaultRatingModule constructor: mDebugHelperPtr is null");
}

DefaultRatingModule::DefaultRatingModule(double fovV, double fovH, double fcp, double ncp,
                                         bool useTargetRobotState,
                                         const robot_model_services::RobotModelPtr &robotModelPtr,
                                         const MapHelperPtr &mapHelperPtr,
                                         const CameraModelFilterPtr &cameraModelFilterPtr) :
                                     DefaultRatingModule() {
    mFovV = fovV;
    mFovH = fovH;
    mFcp = fcp;
    mNcp = ncp;
    mUseTargetRobotState = useTargetRobotState;
    mRobotModelPtr = robotModelPtr;
    mMapHelperPtr = mapHelperPtr;
    mCameraModelFilterPtr = cameraModelFilterPtr;
}

DefaultRatingModule::~DefaultRatingModule() { }

void DefaultRatingModule::setInputCloud(const ObjectPointCloudPtr &pointCloudPtr) {
    // set input cloud in parent class
    RatingModule::setInputCloud(pointCloudPtr);

    mInputCloudChanged = true;
}

bool DefaultRatingModule::setBestScoreContainer(const ViewportPoint &currentViewport, ViewportPoint &candidateViewport) {
    mDebugHelperPtr->writeNoticeably("STARTING DEFAULTRATINGMODULE::SET-BEST-SCORE-CONTAINER METHOD",
                DebugHelper::RATING);
    // reset the cached data
    this->resetCache();

    // get the power set of the object name set
    ObjectTypeSetPtr objectTypeSetPtr = candidateViewport.object_type_set;
    ObjectTypePowerSetPtr powerSetPtr = MathHelper::powerSet<ObjectTypeSet> (objectTypeSetPtr);

    //Create list of all viewports
    ViewportPointCloudPtr viewports = ViewportPointCloudPtr(new ViewportPointCloud());

    // do the filtering for each combination of object types
    for (ObjectTypePowerSet::iterator subSetIter = powerSetPtr->begin(); subSetIter != powerSetPtr->end(); ++subSetIter) {
        ViewportPoint viewport;
        if (!candidateViewport.filterObjectPointCloudByTypes(*subSetIter, viewport)) {
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

    bool success = this->getBestViewport(viewports, candidateViewport);
    mDebugHelperPtr->writeNoticeably("ENDING DEFAULTRATINGMODULE::SET-BEST-SCORE-CONTAINER METHOD",
                DebugHelper::RATING);
    return success;
}

bool DefaultRatingModule::getBestViewport(ViewportPointCloudPtr &viewports, ViewportPoint &bestViewport) {
    mDebugHelperPtr->writeNoticeably("STARTING DEFAULTRATINGMODULE::GET-BEST-VIEWPORT METHOD",
                DebugHelper::RATING);

    // if there aren't any viewports, the search failed.
    if (viewports->size() == 0) {
        mDebugHelperPtr->writeNoticeably("ENDING DEFAULTRATINGMODULE::GET-BEST-VIEWPORT METHOD",
                    DebugHelper::RATING);
        return false;
    }

    // set utility normalization and normalized utility depending on maximal amount of objects in viewport
    float utilityNormalization = 0.0;

    BOOST_FOREACH(ViewportPoint viewport, *viewports) {
        float utility = viewport.score->getUnweightedUnnormalizedUtility();
        if (utility > utilityNormalization) {
            utilityNormalization = utility;
        }
    }

    BOOST_FOREACH(ViewportPoint viewport, *viewports) {
        viewport.score->setUtilityNormalization(utilityNormalization);

        float weightedNormalizedUtility = mOmegaUtility * viewport.score->getUnweightedUnnormalizedUtility() / utilityNormalization;
        viewport.score->setWeightedNormalizedUtility(weightedNormalizedUtility);
    }

    // sort the viewports by rating
    if (mDebugHelperPtr->getLevel() & DebugHelper::RATING) {
        // sorting all viewports
        ViewportPointCloudPtr viewportsSorted(new ViewportPointCloud(*viewports));
        std::sort(viewportsSorted->begin(), viewportsSorted->end(), boost::bind(&RatingModule::compareViewports, *this, _1, _2));

        // output the sorted list of viewports
        for (unsigned int i = 0; i < viewportsSorted->size(); i++) {
            mDebugHelperPtr->write(std::stringstream() << "THIS IS VIEWPORT NR. " << i+1 << " IN THE SORTED LIST.",
                        DebugHelper::RATING);
            ViewportPoint viewport = viewportsSorted->at(i);
            mDebugHelperPtr->write(std::stringstream() << viewport, DebugHelper::RATING);
            mDebugHelperPtr->write(std::stringstream() << "rating: " << this->getRating(viewport.score), DebugHelper::RATING);
        }
    }
    // just getting the best viewport
    bestViewport = *std::max_element(viewports->begin(), viewports->end(), boost::bind(&RatingModule::compareViewports, *this, _1, _2));


    // output best viewport
    mDebugHelperPtr->write("THIS IS THE BEST VIEWPORT IN THE SORTED LIST.", DebugHelper::RATING);
    mDebugHelperPtr->write(std::stringstream() << bestViewport, DebugHelper::RATING);
    mDebugHelperPtr->write(std::stringstream() << "rating: " << this->getRating(bestViewport.score), DebugHelper::RATING);

    mDebugHelperPtr->writeNoticeably("ENDING DEFAULTRATINGMODULE::GET-BEST-VIEWPORT METHOD",
                DebugHelper::RATING);
    return true;
}

bool DefaultRatingModule::compareViewports(const ViewportPoint &a, const ViewportPoint &b) {
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
    return DefaultScoreContainerPtr(new DefaultScoreContainer());
}

float DefaultRatingModule::getOrientationUtility(const ViewportPoint &viewport, ObjectPoint &objectPoint) {
    if (!mUseOrientationUtility)
        return 1.0;

    float maxUtility = 0.0;

    // check the utilities for each normal and pick the best
    BOOST_FOREACH(int index, *objectPoint.active_normal_vectors) {
        SimpleVector3 objectNormalVector = objectPoint.normal_vectors->at(index);
        SimpleVector3 objectPosition = objectPoint.getPosition();
        maxUtility = std::max(this->getNormalUtility(viewport, objectNormalVector, objectPosition), maxUtility);
    }

    return maxUtility;
}

float DefaultRatingModule::getNormalUtility(const ViewportPoint &viewport, const SimpleVector3 &objectNormalVector, const SimpleVector3 &objectPosition)
{
    return getNormalUtility(viewport, objectNormalVector, objectPosition, mNormalAngleThreshold);
}

float DefaultRatingModule::getNormalUtility(const ViewportPoint &viewport, const SimpleVector3 &objectNormalVector, const SimpleVector3 &objectPosition, double angleThreshold) {
    SimpleVector3 cameraPosition = viewport.getPosition();

    SimpleVector3 objectToCameraVector = (cameraPosition - objectPosition).normalized();

    // rate the angle between the camera orientation and the object normal
    return this->getNormalizedAngleUtility(objectToCameraVector, objectNormalVector, angleThreshold);
}

float DefaultRatingModule::getProximityUtility(const ViewportPoint &viewport, const ObjectPoint &objectPoint) {
    if (!mUseProximityUtility)
        return 1.0;

    SimpleVector3 cameraPosition = viewport.getPosition();
    SimpleQuaternion cameraOrientation = viewport.getSimpleQuaternion();
    SimpleVector3 cameraOrientationVector = MathHelper::getVisualAxis(cameraOrientation);
    SimpleVector3 objectPosition = objectPoint.getPosition();

    SimpleVector3 objectToCameraVector = cameraPosition - objectPosition;

    // project the object to the camera orientation vector in order to determine the distance to the mid
    float projection = MathHelper::getDotProduct(-cameraOrientationVector, objectToCameraVector);

    // determine the distance of the object to the mid of the frustum
    float distanceToMid = std::abs(projection-(mFcp+mNcp)/2.0);
    float distanceThreshold = std::abs((mFcp-mNcp)/2.0);

    float utility = this->getNormalizedRating(distanceToMid, distanceThreshold);

    return utility;
}

float DefaultRatingModule::getSideUtility(const ViewportPoint &viewport, const ObjectPoint &objectPoint) {
    if (!mUseSideUtility)
        return 1.0;

    SimpleVector3 cameraPosition = viewport.getPosition();
    SimpleQuaternion cameraOrientation = viewport.getSimpleQuaternion();
    SimpleVector3 cameraOrientationVector = MathHelper::getVisualAxis(cameraOrientation);

    float angleMin = (float) MathHelper::degToRad(std::min(mFovV, mFovH)) / 2.0;
    SimpleVector3 objectPosition = objectPoint.getPosition();
    SimpleVector3 objectToCameraVector = cameraPosition - objectPosition;
    SimpleVector3 objectToCameraVectorNormalized = objectToCameraVector.normalized();

    return this->getNormalizedAngleUtility(-cameraOrientationVector, objectToCameraVectorNormalized, angleMin);
}


float DefaultRatingModule::getFrustumPositionUtility(const ViewportPoint &viewport, ObjectPoint &objectPoint)
{
    // utility for how far the object is on the side of the camera view
    float sideUtility = this->getSideUtility(viewport, objectPoint);

    // utility for how far the object is away from the camera
    float proximityUtility = this->getProximityUtility(viewport, objectPoint);

    // the complete frumstum position utility
    float utility = sideUtility * proximityUtility;

    return utility;
}

float DefaultRatingModule::getNormalizedAngleUtility(const SimpleVector3 v1, const SimpleVector3 v2, double angleThreshold) {
    float angle = MathHelper::getAngle(v1, v2);
    float utility = this->getNormalizedRating(angle, angleThreshold);
    return utility;
}

float DefaultRatingModule::getRating(const BaseScoreContainerPtr &a) {
    if (!a) {
        ROS_ERROR("Score container is nullpointer");
    }

    if (mRatingNormalization <= 0) {
        ROS_ERROR("Omega parameters were not set correctly");
    }

    float result = (a->getWeightedNormalizedUtility() + a->getWeightedInverseCosts()) / mRatingNormalization;

    return result;
}

bool DefaultRatingModule::setSingleScoreContainer(const ViewportPoint &currentViewport,
                                                  ViewportPoint &candidateViewport) {
    DefaultScoreContainerPtr defRatingPtr(new DefaultScoreContainer());

    // set the utility
    float utility = this->getUnweightedUnnormalizedUtility(candidateViewport);

    if (utility <= 0) {
        return false;
    }

    defRatingPtr->setUnweightedUnnormalizedUtility(utility);

    BOOST_FOREACH(std::string objectType, *(candidateViewport.object_type_set)) {
        defRatingPtr->setUnweightedUnnormalizedObjectUtilitiy(objectType, mUnweightedUnnormalizedObjectUtilities[objectType]);
    }

    // set the costs
    double costs = this->getWeightedInverseCosts(currentViewport, candidateViewport);
    if (costs < 0.0) {
        return false;
    }
    defRatingPtr->setWeightedInverseCosts(costs);

    defRatingPtr->setUnweightedInverseMovementCostsBaseTranslation(mUnweightedInverseMovementCostsBaseTranslation);
    defRatingPtr->setUnweightedInverseMovementCostsBaseRotation(mUnweightedInverseMovementCostsBaseRotation);
    defRatingPtr->setUnweightedInverseMovementCostsPTU(mUnweightedInverseMovementCostsPTU);
    defRatingPtr->setUnweightedInverseRecognitionCosts(mUnweightedInverseRecognitionCosts);

    candidateViewport.score = defRatingPtr;
    return true;
}

void DefaultRatingModule::setNormalAngleThreshold(double angle) {
    mNormalAngleThreshold = angle;
}

double DefaultRatingModule::getNormalAngleThreshold() {
    return mNormalAngleThreshold;
}

void DefaultRatingModule::setOmegaParameters(double omegaUtility, double omegaPan, double omegaTilt, double omegaRot, double omegaBase, double omegaRecognition) {
    this->mOmegaUtility = omegaUtility;
    this->mOmegaPan = omegaPan;
    this->mOmegaTilt = omegaTilt;
    this->mOmegaRot = omegaRot;
    this->mOmegaBase = omegaBase;
    this->mOmegaRecognizer = omegaRecognition;

    this->setRatingNormalization();
}

void DefaultRatingModule::setFrustumParameters(double fovV, double fovH, double fcp, double ncp)
{
    mFovV = fovV;
    mFovH = fovH;
    mFcp = fcp;
    mNcp = ncp;
}

void DefaultRatingModule::setUseTargetRobotState(bool useTargetRobotState)
{
    mUseTargetRobotState = useTargetRobotState;
}

void DefaultRatingModule::setUtilityParameters(bool useOrientationUtility, bool useProximityUtility, bool useSideUtility)
{
    this->mUseOrientationUtility = useOrientationUtility;
    this->mUseProximityUtility = useProximityUtility;
    this->mUseSideUtility = useSideUtility;
}

float DefaultRatingModule::getNormalizedRating(float deviation, float threshold) {
    if (deviation < threshold) {
        return .5 + .5 * cos(deviation * M_PI / threshold);
    }
    return 0.0;
}

double DefaultRatingModule::getUnweightedUnnormalizedUtility(const ViewportPoint &candidateViewport) {
    double utility = 0.0;

    // get the utility for each object type and sum them up
    BOOST_FOREACH(std::string objectType, *(candidateViewport.object_type_set)) {
        // set the utility for the object type if not already done
        if (mUnweightedUnnormalizedObjectUtilities.count(objectType) == 0) {
            setUnweightedUnnormalizedObjectUtilities(candidateViewport, objectType);
        }

        utility += mUnweightedUnnormalizedObjectUtilities[objectType];
    }

    return utility;
}

void DefaultRatingModule::setUnweightedUnnormalizedObjectUtilities(const ViewportPoint &candidateViewport, std::string objectType) {
    float utility = 0;

    // build the sum of the orientation and frustum position utilities of all object points in the candidate camera view with the given type
    BOOST_FOREACH(int index, *(candidateViewport.child_indices)) {
        ObjectPoint& objectPoint = this->getInputCloud()->at(index);

        if (objectPoint.type != objectType) {
            continue;
        }

        if (objectPoint.active_normal_vectors->size() == 0) {
            continue;
        }

        float orientationUtility = this->getOrientationUtility(candidateViewport, objectPoint);
        float positionUtility = this->getFrustumPositionUtility(candidateViewport, objectPoint);

        // calculate utility
        utility += orientationUtility * positionUtility * objectPoint.intermediate_object_weight;
        //ROS_ERROR_STREAM("DefaultRatingModule::setObjectUtilities weight "<< objectPoint.intermediate_object_weight);
    }

    // cache the utility and the orientation and position utilities
    mUnweightedUnnormalizedObjectUtilities[objectType] = utility;
}

double DefaultRatingModule::getWeightedInverseCosts(const ViewportPoint &sourceViewport,
                                            const ViewportPoint &targetViewport) {
    // set the cache members needed for the costs if not already done
    if (!mTargetState) {
        this->setRobotState(sourceViewport, targetViewport);
    }

    if (mWeightedInverseMovementCosts < 0) {
        this->setWeightedInverseMovementCosts();
    }

    if (mInputCloudChanged) {
        this->setMaxRecognitionCosts();
    }

    // get the costs for the recoginition of the objects
    mUnweightedInverseRecognitionCosts = this->getUnweightedInverseRecognitionCosts(targetViewport);

    // calculate the full movement costs
    double fullCosts = mWeightedInverseMovementCosts + mUnweightedInverseRecognitionCosts * mOmegaRecognizer;

    if (mUseTargetRobotState)
    {
        robot_model_services::MILDRobotStatePtr state = boost::static_pointer_cast<robot_model_services::MILDRobotState>(mTargetState);
        SimpleVector3 basePosition(state->x, state->y, 0);
        if (!mMapHelperPtr->isOccupancyValueAcceptable(mMapHelperPtr->getRaytracingMapOccupancyValue(basePosition))) {
            fullCosts = -1.0;
            mDebugHelperPtr->write(std::stringstream() << "View removed because the calculated robot state isn't reachable.",
                        DebugHelper::RATING);
        }
    }

    return fullCosts;
}

double DefaultRatingModule::getUnweightedInverseRecognitionCosts(const ViewportPoint &targetViewport) {
    // avoid dividing by 0
    if (mMaxRecognitionCosts == 0) {
        throw "Maximum recognition costs are 0.";
    }

    // get the costs for the recoginition of each object type
    Precision recognitionCosts = 0;
    BOOST_FOREACH(std::string objectType, *(targetViewport.object_type_set)) {
        recognitionCosts += mCameraModelFilterPtr->getRecognizerCosts(objectType);
    }
    double normalizedRecognitionCosts = 1.0 - recognitionCosts / mMaxRecognitionCosts;

    return normalizedRecognitionCosts;
}

void DefaultRatingModule::setRobotState(const ViewportPoint &sourceViewport, const ViewportPoint &targetViewport) {
    // set the source robot state
    const next_best_view::SimpleVector3 src_pos = sourceViewport.getPosition();
    const next_best_view::SimpleQuaternion src_ori = sourceViewport.getSimpleQuaternion();
  
    robot_model_services::RobotStatePtr sourceState = mRobotModelPtr->calculateRobotState(robot_model_services::SimpleVector3(src_pos[0], src_pos[1], src_pos[2]), robot_model_services::SimpleQuaternion(src_ori.w(), src_ori.x(), src_ori.y(), src_ori.z()));
    mRobotModelPtr->setCurrentRobotState(sourceState);

    // set the target robot state
    mTargetState = mRobotModelPtr->calculateRobotState(targetViewport.getPosition(), targetViewport.getSimpleQuaternion());
}

void DefaultRatingModule::setWeightedInverseMovementCosts() {
    // get the different movement costs
    mUnweightedInverseMovementCostsBaseTranslation = mRobotModelPtr->getBase_TranslationalMovementCosts(mTargetState);
    mUnweightedInverseMovementCostsBaseRotation = mRobotModelPtr->getBase_RotationalMovementCosts(mTargetState);

    // set the omega parameter for PTU movement and the PTU movement costs
    Precision movementCostsPTU_Pan = mRobotModelPtr->getPTU_PanMovementCosts(mTargetState);
    Precision movementCostsPTU_Tilt = mRobotModelPtr->getPTU_TiltMovementCosts(mTargetState);

    // set the PTU movement omega parameter
    if (movementCostsPTU_Tilt*mOmegaTilt > movementCostsPTU_Pan*mOmegaPan)
    {
        mOmegaPTU = mOmegaPan;
        mUnweightedInverseMovementCostsPTU = movementCostsPTU_Pan;
    }
    else
    {
        mOmegaPTU = mOmegaTilt;
        mUnweightedInverseMovementCostsPTU = movementCostsPTU_Tilt;
    }

    // set the movement costs
    mWeightedInverseMovementCosts = mUnweightedInverseMovementCostsBaseTranslation * mOmegaBase + mUnweightedInverseMovementCostsPTU * mOmegaPTU
            + mUnweightedInverseMovementCostsBaseRotation * mOmegaRot;
}

void DefaultRatingModule::setRatingNormalization() {
    mRatingNormalization = mOmegaUtility + mOmegaBase + mOmegaPTU + mOmegaRot + mOmegaRecognizer;
}

void DefaultRatingModule::setMaxRecognitionCosts() {
    mMaxRecognitionCosts = 0;
    ObjectPointCloudPtr inputCloud = this->getInputCloud();

    double maxRecognitionCosts = 0;
    std::vector<std::string> types;

    BOOST_FOREACH(ObjectPoint objectPoint, *(inputCloud)) {
        // only check each type once
        if (std::find(types.begin(), types.end(), objectPoint.type) == types.end()) {
            maxRecognitionCosts += mCameraModelFilterPtr->getRecognizerCosts(objectPoint.type);
            types.push_back(objectPoint.type);
        }
    }
    mMaxRecognitionCosts = maxRecognitionCosts;
    mInputCloudChanged = false;
}

void DefaultRatingModule::resetCache() {
    mUnweightedUnnormalizedObjectUtilities.clear();
    mWeightedInverseMovementCosts = -1;
    mUnweightedInverseMovementCostsBaseTranslation = -1;
    mUnweightedInverseMovementCostsBaseRotation = -1;
    mUnweightedInverseMovementCostsPTU = -1;
    mUnweightedInverseRecognitionCosts = -1;
    mOmegaPTU = -1;
    mTargetState = NULL;
}

void DefaultRatingModule::setRobotState(robot_model_services::RobotStatePtr robotState) {
    mRobotModelPtr->setCurrentRobotState(robotState);
}

}

