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

#pragma once

#include "typedef.hpp"
#include <vector>
#include <map>
#include <chrono>

#include <boost/foreach.hpp>
#include <boost/range/algorithm_ext/iota.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/range/irange.hpp>

#include "next_best_view/hypothesis_updater/HypothesisUpdater.hpp"
#include <robot_model_services/robot_model/RobotModel.hpp>
#include "next_best_view/crop_box/CropBoxFilter.hpp"
#include "next_best_view/camera_model_filter/CameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/CameraModelFilterAbstractFactory.hpp"
#include "next_best_view/unit_sphere_sampler/UnitSphereSampler.hpp"
#include "next_best_view/space_sampler/SpaceSampler.hpp"
#include "next_best_view/rating/RatingModule.hpp"
#include "next_best_view/rating/RatingModuleAbstractFactory.hpp"
#include "next_best_view/rating/impl/DefaultScoreContainer.hpp"
#include "asr_msgs/AsrAttributedPointCloud.h"
#include "asr_msgs/AsrAttributedPoint.h"
#include "next_best_view/helper/DebugHelper.hpp"
#include "next_best_view/helper/MapHelper.hpp"
#include "next_best_view/helper/ObjectHelper.h"
#include "next_best_view/helper/VisualizationsHelper.hpp"

namespace next_best_view {
class NextBestViewCalculator {
private:
    ObjectPointCloudPtr mPointCloudPtr;
    // this is basically unused, but "might" be used to remove objects from mPointCloudPtr
    IndicesPtr mActiveIndicesPtr;
    KdTreePtr mKdTreePtr;
    std::map<std::string, std::string> objectsResources;

    // modules
    UnitSphereSamplerPtr mUnitSphereSamplerPtr;
    MapHelperPtr mMapHelperPtr;
    SpaceSamplerPtr mSpaceSamplerPtr;
    robot_model_services::RobotModelPtr mRobotModelPtr;
    CameraModelFilterPtr mCameraModelFilterPtr;
    RatingModulePtr mRatingModulePtr;
    HypothesisUpdaterPtr mHypothesisUpdaterPtr;

    RatingModuleAbstractFactoryPtr mRatingModuleAbstractFactoryPtr;
    CameraModelFilterAbstractFactoryPtr mCameraModelFilterAbstractFactoryPtr;

    CropBoxFilterPtr mCropBoxFilterPtr;
    float mEpsilon;
    ObjectTypeSetPtr mObjectTypeSetPtr;
    DebugHelperPtr mDebugHelperPtr;
    VisualizationHelperPtr mVisHelperPtr;

    bool mEnableCropBoxFiltering;
    bool mEnableIntermediateObjectWeighting;

    int mMaxIterationSteps;

    int mNumberOfThreads;
    std::vector<CameraModelFilterPtr> mThreadCameraModels;
    std::vector<RatingModulePtr> mThreadRatingModules;

    double mMinUtility;
    bool mRequireMinUtility;

public:

    NextBestViewCalculator(const UnitSphereSamplerPtr & unitSphereSamplerPtr = UnitSphereSamplerPtr(),
                           const MapHelperPtr &mapHelperPtr = MapHelperPtr(),
                           const SpaceSamplerPtr &spaceSamplerPtr = SpaceSamplerPtr(),
                           const robot_model_services::RobotModelPtr &robotModelPtr = robot_model_services::RobotModelPtr(),
                           const CameraModelFilterPtr &cameraModelFilterPtr = CameraModelFilterPtr(),
                           const RatingModulePtr &ratingModulePtr = RatingModulePtr())
        : objectsResources(),
          mUnitSphereSamplerPtr(unitSphereSamplerPtr),
          mMapHelperPtr(mapHelperPtr),
          mSpaceSamplerPtr(spaceSamplerPtr),
          mRobotModelPtr(robotModelPtr),
          mCameraModelFilterPtr(cameraModelFilterPtr),
          mRatingModulePtr(ratingModulePtr),
          mEpsilon(10E-3),
          mNumberOfThreads(boost::thread::hardware_concurrency()),
          mThreadCameraModels(mNumberOfThreads),
          mThreadRatingModules(mNumberOfThreads) {

        setMapHelper(mapHelperPtr);
        mDebugHelperPtr = DebugHelper::getInstance();
    }

    /**
     * Calculates the next best view. Starting point of iterative calculations for getNextBestView() service call.
     */
    bool calculateNextBestView(const ViewportPoint &currentCameraViewport, ViewportPoint &resultViewport) {
        auto begin = std::chrono::high_resolution_clock::now();
        mDebugHelperPtr->writeNoticeably("STARTING CALCULATE-NEXT-BEST-VIEW METHOD", DebugHelper::CALCULATION);

        //Calculate robot configuration corresponding to current camera viewport of robot.
        initializeRobotState(currentCameraViewport);

        mDebugHelperPtr->write("Calculate discrete set of view orientations on unit sphere",
                                DebugHelper::CALCULATION);
        //Get discretized set of camera orientations (pan, tilt) that are to be considered at each robot position considered during iterative optimization.
        SimpleQuaternionCollectionPtr feasibleOrientationsCollectionPtr = generateOrientationSamples();

        // create the next best view point cloud
        bool success = this->doIteration(currentCameraViewport, feasibleOrientationsCollectionPtr, resultViewport);

        auto finish = std::chrono::high_resolution_clock::now();
        // cast timediff to flaot in seconds
        ROS_INFO_STREAM("Iteration took " << std::chrono::duration<float>(finish-begin).count() << " seconds.");
        mDebugHelperPtr->writeNoticeably("ENDING CALCULATE-NEXT-BEST-VIEW METHOD", DebugHelper::CALCULATION);
        return success;
    }

    /**
     * @brief initializeRobotState initializes the robotstate to the given viewport
     * @param currentCameraViewport the camera viewport of the robot
     */
    void initializeRobotState(const ViewportPoint &currentCameraViewport) {
        robot_model_services::RobotStatePtr currentState = mRobotModelPtr->calculateRobotState(currentCameraViewport.getPosition(), currentCameraViewport.getSimpleQuaternion());
        //Save it.
        mRobotModelPtr->setCurrentRobotState(currentState);
        mRatingModulePtr->setRobotState(currentState);
        for (int i : boost::irange(0, mNumberOfThreads)) {
            mThreadRatingModules[i]->setRobotState(currentState);
        }
    }

    void getFeasibleSamplePoints(const SamplePointCloudPtr &sampledSpacePointCloudPtr, IndicesPtr &resultIndicesPtr) {
        resultIndicesPtr = IndicesPtr(new Indices());
        //Go through all
        for(std::size_t index = 0; index < sampledSpacePointCloudPtr->size(); index++) {
            // get the point
            SamplePoint &spaceSamplePoint = sampledSpacePointCloudPtr->at(index);


            IndicesPtr childIndicesPtr;
            if (getFeasibleHypothesis(spaceSamplePoint.getSimpleVector3(), childIndicesPtr)) {
                // set the indices
                spaceSamplePoint.child_indices = childIndicesPtr;

                // add the index to active indices.
                resultIndicesPtr->push_back(index);
            }
        }
    }

    /**
     * @brief getFeasibleViewports returns a vector of indices/sampleViewports which contain nearby (hypothesis)
     * @param sampleViewportsPtr viewports which may contain nearby object hypothesis
     * @param resultIndicesPtr
     */
    void getFeasibleViewports(const ViewportPointCloudPtr &sampleViewportsPtr, IndicesPtr &resultIndicesPtr) {
        resultIndicesPtr = IndicesPtr(new Indices());
        //Go through all
        for(std::size_t index = 0; index < sampleViewportsPtr->size(); index++) {
            // get the point
            ViewportPoint &spaceViewport = sampleViewportsPtr->at(index);

            IndicesPtr childIndicesPtr;
            if (getFeasibleHypothesis(spaceViewport.getPosition(), childIndicesPtr)) {
                // set the indices
                spaceViewport.child_indices = childIndicesPtr;

                // add the index to active indices.
                resultIndicesPtr->push_back(index);
            }
        }
    }


    /**
     * @brief getFeasibleHypothesis finds object hypothesis near samplePoint
     * @param samplePoint the point to find nearby hypothesis
     * @param resultIndicesPtr nearby object hypothesis
     * @return true if more than 0 object hypothesis are in range of samplePoint
     */
    bool getFeasibleHypothesis(SimpleVector3 samplePoint, IndicesPtr &resultIndicesPtr) {
        // get max radius by far clipping plane of camera. this marks the limit for the visible object distance.
        double radius = mCameraModelFilterPtr->getFarClippingPlane();

        ObjectPoint comparablePoint(samplePoint);

        // the resulting child indices will be written in here
        IndicesPtr childIndicesPtr(new Indices());
        // we don't need distances, but distances are written in here
        SquaredDistances dismissDistances;

        // this is a radius search - which reduces the complexity level for frustum culling.
        int k = mKdTreePtr->radiusSearch(comparablePoint, radius, *childIndicesPtr, dismissDistances);

        // if there is no result of neighboured points, no need to add this point.
        if (k == 0) {
            return false;
        }

        // set the indices
        resultIndicesPtr = childIndicesPtr;
        return true;
    }

    /**
     * @brief rates given viewports, which must contain child_indices
     * @param sampleNextBestViewports [in] viewports to rate
     * @param currentCameraViewport [in] current camera viewport to rate
     * @param resultViewport [out] viewport with best rating
     * @param objectTypeSetIsKnown
     * @return if valid result
     */
    bool rateViewports(const ViewportPointCloudPtr &sampleNextBestViewportsPtr, const ViewportPoint &currentCameraViewport, ViewportPoint &resultViewport, bool objectTypeSetIsKnown = false) {
        ViewportPointCloudPtr ratedNextBestViewportsPtr = ViewportPointCloudPtr(new ViewportPointCloud());
        return rateViewportsInternal(sampleNextBestViewportsPtr, currentCameraViewport, ratedNextBestViewportsPtr, resultViewport, objectTypeSetIsKnown);
    }

    /**
     * @brief rates given viewports, which must contain child_indices
     * @param sampleNextBestViewports [in] viewports to rate
     * @param currentCameraViewport [in] current camera viewport to rate
     * @param ratedNextBestViewports [out] rated viewports, which might be fewer
     * @param objectTypeSetIsKnown
     * @return if valid result
     */
    bool rateViewports(const ViewportPointCloudPtr &sampleNextBestViewports, const ViewportPoint &currentCameraViewport, ViewportPointCloudPtr &ratedNextBestViewportsPtr, bool objectTypeSetIsKnown = false) {
        ratedNextBestViewportsPtr = ViewportPointCloudPtr(new ViewportPointCloud());
        ViewportPoint resultViewport;
        return rateViewportsInternal(sampleNextBestViewports, currentCameraViewport, ratedNextBestViewportsPtr, resultViewport, objectTypeSetIsKnown);
    }

private:

    /**
     * @brief rates viewports and returns vector with rated wiewports and the best rated viewport
     * @param sampleNextBestViewports [in] viewports to rate
     * @param currentCameraViewport [in] current camera viewport to rate
     * @param ratedNextBestViewports [out] rated (unsorted in most cases) vector of viewports
     * @param resultViewport [out] best viewport
     * @param objectTypeSetIsKnown
     * @return if valid result
     */
    bool rateViewportsInternal(const ViewportPointCloudPtr &sampleNextBestViewports, const ViewportPoint &currentCameraViewport,
                               ViewportPointCloudPtr &ratedNextBestViewports, ViewportPoint &resultViewport, bool objectTypeSetIsKnown) {
        // start threads
        boost::thread_group threadGroup;
        boost::mutex mutex;
        for (int i : boost::irange(0, mNumberOfThreads)) {
            threadGroup.add_thread(new boost::thread(&NextBestViewCalculator::ratingThread, this, i, boost::ref(mutex),
                                                     boost::ref(sampleNextBestViewports),
                                                     boost::ref(currentCameraViewport),
                                                     boost::ref(ratedNextBestViewports),
                                                     objectTypeSetIsKnown));
        }
        threadGroup.join_all();

        mDebugHelperPtr->write("Sorted list of all viewports (each best for pos & orient combi) in this iteration step.",
                                    DebugHelper::RATING);
        return mRatingModulePtr->getBestViewport(ratedNextBestViewports, resultViewport);
    }

    /**
     * @brief ratingThread
     * @param threadId
     * @param mutex
     * @param sampleNextBestViewports viewport sampling, which contains a good estimation for nearby objects per viewport
     * @param currentCameraViewport current camera viewport, used to rate
     * @param ratedNextBestViewports results
     * @param objectTypeSetIsKnown if object_types_set is set fixed and hould not be optimized
     */
    void ratingThread(int threadId, boost::mutex &mutex,
                      const ViewportPointCloudPtr &sampleNextBestViewports,
                      const ViewportPoint &currentCameraViewport,
                      const ViewportPointCloudPtr &ratedNextBestViewports,
                      bool objectTypeSetIsKnown) {
        unsigned int nViewportSamples = sampleNextBestViewports->size();
        double startFactor = (double) threadId / mNumberOfThreads;
        double endFactor = (threadId + 1.0) / mNumberOfThreads;
        unsigned int startIdx = (int) (startFactor * nViewportSamples);
        unsigned int endIdx = (int) (endFactor * nViewportSamples);
        //Go through all interesting space sample points for one iteration step to create candidate viewports.
        for (int i : boost::irange(startIdx, endIdx)) {
            ViewportPoint fullViewportPoint = sampleNextBestViewports->at(i);
            //Calculate which object points lie within frustum for given viewport.
            if (!this->doFrustumCulling(mThreadCameraModels[threadId], fullViewportPoint)) {
                //Skip viewport if no object point is within frustum.
                continue;
            }

            //For given viewport(combination of robot position and camera viewing direction)
            // get combination of objects (all present in frustum) to search for and the corresponding score for viewport, given that combination.
            mDebugHelperPtr->write("Getting viewport with optimal object constellation for given position & orientation combination.",
                                        DebugHelper::RATING);
            if (objectTypeSetIsKnown) {
                if (!rateSingleViewportFixedObjectTypes(mThreadRatingModules[threadId], currentCameraViewport, fullViewportPoint))
                    continue;
            } else {
                if (!rateSingleViewportOptimizeObjectTypes(mThreadRatingModules[threadId], currentCameraViewport, fullViewportPoint))
                    continue;
            }
            //Keep viewport with optimal subset of objects within frustum to search for.
            mutex.lock();
            ratedNextBestViewports->push_back(fullViewportPoint);
            mutex.unlock();
        }
    }

    /**
     * @brief uses setBestScoreContainer to optimize which objects should be recognized
     * @param ratingModulePtr [in] used to rate
     * @param currentCameraViewport [in] used to rate
     * @param fullViewportPoint [out] resulting rated viewport
     * @return
     */
    bool rateSingleViewportOptimizeObjectTypes(const RatingModulePtr &ratingModulePtr, const ViewportPoint &currentCameraViewport, ViewportPoint &fullViewportPoint) {
        mDebugHelperPtr->write("Getting viewport with optimal object constellation for given position & orientation combination.",
                                    DebugHelper::RATING);
        return ratingModulePtr->setBestScoreContainer(currentCameraViewport, fullViewportPoint);
    }

    /**
     * @brief uses setSingleScoreContainer to use object_type_set of fullViewportPoint to rate.
     * @param ratingModulePtr [in] used to rate
     * @param currentCameraViewport [in] used to rate
     * @param fullViewportPoint [out] resulting rated viewport
     * @return
     */
    bool rateSingleViewportFixedObjectTypes(const RatingModulePtr &ratingModulePtr, const ViewportPoint &currentCameraViewport, ViewportPoint &fullViewportPoint) {
        ratingModulePtr->resetCache();
        return ratingModulePtr->setSingleScoreContainer(currentCameraViewport, fullViewportPoint);
    }

    bool doIteration(const ViewportPoint &currentCameraViewport, const SimpleQuaternionCollectionPtr &sampledOrientationsPtr, ViewportPoint &resultViewport) {
        mDebugHelperPtr->writeNoticeably("STARTING DO-ITERATION METHOD", DebugHelper::CALCULATION);

        int iterationStep = 0;
        //Best viewport at the end of each iteration step and starting point for optimization (grid alignment) for each following step.
        ViewportPoint currentBestViewport = currentCameraViewport;
        float currentBestRating = 0;
        
        //Enables to interrupt iterating if it takes too long.
        while (ros::ok()) {
            ViewportPoint intermediateResultViewport;

            //Contractor is always divided by two.
            if (!this->doIterationStep(currentCameraViewport, currentBestViewport,
                                       sampledOrientationsPtr, 1.0 / pow(2.0, iterationStep),
                                       intermediateResultViewport, iterationStep)) {
                //Happens, when no valid viewport is found in that iteration step (including current viewport). E.g. when all normals are invalidated.
                mDebugHelperPtr->writeNoticeably("ENDING DO-ITERATION METHOD", DebugHelper::CALCULATION);
                return false;
            }

            //Iteration step must be increased before the following check.
            iterationStep ++;
            float rating = mRatingModulePtr->getRating(intermediateResultViewport.score);
            mDebugHelperPtr->write("THIS IS THE BEST VIEWPORT IN THE GIVEN ITERATION STEP.",
                                        DebugHelper::CALCULATION);
            mDebugHelperPtr->write(std::stringstream() << intermediateResultViewport, DebugHelper::CALCULATION);
            mDebugHelperPtr->write(std::stringstream() << "rating: " << rating, DebugHelper::CALCULATION);
            mDebugHelperPtr->write(std::stringstream() << "IterationStep: " << iterationStep, DebugHelper::CALCULATION);

            //First condition is runtime optimization to not iterate around current pose. Second is general abort criterion.
            if (std::abs(rating - currentBestRating) <= this->getEpsilon() || iterationStep >= mMaxIterationSteps) {
                //Stop once position displacement (resp. differing view at sufficient space sampling resolution) is small enough.
                resultViewport = intermediateResultViewport;
                ROS_INFO_STREAM ("Next-best-view estimation SUCCEEDED. Took " << iterationStep << " iterations");
                mDebugHelperPtr->write("THIS IS THE BEST VIEWPORT FOR ALL ITERATION STEPS.",
                                            DebugHelper::CALCULATION);
                mDebugHelperPtr->write(std::stringstream() << resultViewport, DebugHelper::CALCULATION);
                mDebugHelperPtr->write(std::stringstream() << "rating: " << rating, DebugHelper::CALCULATION);
                mDebugHelperPtr->write(std::stringstream() << "IterationStep: " << iterationStep,
                                            DebugHelper::CALCULATION);
                mDebugHelperPtr->writeNoticeably("ENDING DO-ITERATION METHOD", DebugHelper::CALCULATION);
                return true;
            }

            currentBestViewport = intermediateResultViewport;
            currentBestRating = rating;

        }
        mDebugHelperPtr->writeNoticeably("ENDING DO-ITERATION METHOD", DebugHelper::CALCULATION);
        //Only reached when iteration fails or is interrupted.
        return false;
    }

    bool doIterationStep(const ViewportPoint &currentCameraViewport, const ViewportPoint &currentBestViewport,
                         const SimpleQuaternionCollectionPtr &sampledOrientationsPtr, float contractor,
                         ViewportPoint &resultViewport, int iterationStep) {
        mDebugHelperPtr->writeNoticeably("STARTING DO-ITERATION-STEP METHOD", DebugHelper::CALCULATION);

        // current camera position
        SimpleVector3 currentBestPosition = currentBestViewport.getPosition();

        //Calculate grid for resolution given in this iteration step.
        SamplePointCloudPtr sampledSpacePointCloudPtr = generateSpaceSamples(currentBestPosition, contractor, currentBestPosition[2]);

        //Skip rating all orientations (further code here) if we can only consider our current best robot position and increase sampling resolution
        if (sampledSpacePointCloudPtr->size() == 1 && this->getEpsilon() < contractor) {
            mDebugHelperPtr->write("No RViz visualization for this iteration step, since no new next-best-view found for that resolution.",
                                   DebugHelper::VISUALIZATION);
            bool success = doIterationStep(currentCameraViewport, currentBestViewport, sampledOrientationsPtr, contractor * .5, resultViewport, iterationStep);
            mDebugHelperPtr->writeNoticeably("ENDING DO-ITERATION-STEP METHOD", DebugHelper::CALCULATION);
            return success;
        }

        //Create list of all view ports that are checked during this iteration step.
        ViewportPointCloudPtr sampleNextBestViewports = combineSamples(sampledSpacePointCloudPtr, sampledOrientationsPtr);

        // rate
        ViewportPointCloudPtr ratedNextBestViewportsPtr;
        if (!rateViewports(sampleNextBestViewports, currentCameraViewport, ratedNextBestViewportsPtr)) {
            mDebugHelperPtr->writeNoticeably("ENDING DO-ITERATION-STEP METHOD", DebugHelper::CALCULATION);
            return false;
        }

        // sort
        // ascending -> last element hast best rating
        auto sortFunction = [this](const ViewportPoint &a, const ViewportPoint &b) {
            // a < b
            return mRatingModulePtr->compareViewports(a, b);
        };
        std::sort(ratedNextBestViewportsPtr->begin(), ratedNextBestViewportsPtr->end(), sortFunction);

        if (mRequireMinUtility) {
            bool foundUtility = false;
            BOOST_REVERSE_FOREACH (ViewportPoint &ratedViewport, *ratedNextBestViewportsPtr) {
                if (ratedViewport.score->getUnweightedUnnormalizedUtility() > mMinUtility) {
                    resultViewport = ratedViewport;
                    foundUtility = true;
                    break;
                }
            }
            if (!foundUtility) {
                ROS_WARN_STREAM("every nbv has too little utility");
                resultViewport = ratedNextBestViewportsPtr->back();
            }
        } else {
            resultViewport = ratedNextBestViewportsPtr->back();
        }

//        mVisHelperPtr->triggerSamplingVisualization(ratedNextBestViewportsPtr, Color(1, 0, 0, 1), "ratedViewports");

        //Visualize iteration step and its result.
        mVisHelperPtr->triggerIterationVisualization(iterationStep, sampledOrientationsPtr, resultViewport,
                                                    sampledSpacePointCloudPtr, mSpaceSamplerPtr);

        mDebugHelperPtr->writeNoticeably("ENDING DO-ITERATION-STEP METHOD", DebugHelper::CALCULATION);
        return true;
    }

public:
    void setHeight(SamplePointCloudPtr pointCloudPtr, double height) {
        for (unsigned int i = 0; i < pointCloudPtr->size(); i++) {
            pointCloudPtr->at(i).z = height;
        }
    }

    /*!
         * \brief creates a new camera viewport with the given data
         * \param position [in] the position of the camera
         * \param orientation [in] the orientation of the camera
         * \param indices [in] the object point indices to be used
         * \param viewportPoint [out] the resulting camera viewport
         * \return whether there are objects in the resulting viewport
         */
    bool doFrustumCulling(const SimpleVector3 &position, const SimpleQuaternion &orientation, const IndicesPtr &indices, ViewportPoint &viewportPoint) {
        return doFrustumCulling(mCameraModelFilterPtr, position, orientation, indices, viewportPoint);
    }

    /**
     * @brief creates a new camera viewport with the given data
     * @param cameraModelFilterPtr [in] the cameraModel used to filter objects
     * @param position [in] the position of the camera
     * @param orientation [in] the orientation of the camera
     * @param indices [in] the object point indices to be used
     * @param viewportPoint [out] the resulting camera viewport
     * @return whether there are objects in the resulting viewport
     */
    bool doFrustumCulling(const CameraModelFilterPtr &cameraModelFilterPtr, const SimpleVector3 &position, const SimpleQuaternion &orientation, const IndicesPtr &indices, ViewportPoint &resultViewport) {
        resultViewport = ViewportPoint(position, orientation);
        resultViewport.child_indices = indices;
        return doFrustumCulling(cameraModelFilterPtr, resultViewport);
    }

    /**
     * @brief creates a new camera viewport with the given data
     * @param resultViewportPoint
     * @return whether there are objects in the resulting viewport
     */
    bool doFrustumCulling(ViewportPoint &resultViewportPoint) {
        return doFrustumCulling(mCameraModelFilterPtr, resultViewportPoint);
    }

    /**
     * @brief creates a new camera viewport with the given data
     * @param cameraModelFilterPtr [in] the cameraModel used to filter objects
     * @param resultViewportPoint [out|in] the resulting camera viewport containg the camera position and orientation and the object point indices to be used
     * @return whether there are objects in the resulting viewport
     */
    bool doFrustumCulling(const CameraModelFilterPtr &cameraModelFilterPtr, ViewportPoint &resultViewportPoint) {
        cameraModelFilterPtr->setIndices(resultViewportPoint.child_indices);
        cameraModelFilterPtr->setPivotPointPose(resultViewportPoint.getPosition(), resultViewportPoint.getSimpleQuaternion());

        // do the frustum culling
        IndicesPtr frustumIndicesPtr;
        //Call wrapper (with next-best-view data structures) for PCL frustum culling call.
        cameraModelFilterPtr->filter(frustumIndicesPtr);

        resultViewportPoint.child_indices = frustumIndicesPtr;
        resultViewportPoint.child_point_cloud = cameraModelFilterPtr->getInputCloud();
        resultViewportPoint.point_cloud = mPointCloudPtr;

        if (frustumIndicesPtr->size() == 0) {
            return false;
        }

        return true;
    }

    /**
     * @brief updates point cloud with external viewport point list
     * @param viewportPointList the list of viewport points
     * @return the number of deactivated normals
     */
    void updateFromExternalViewportPointList(const std::vector<ViewportPoint> &viewportPointList) {
        mDebugHelperPtr->writeNoticeably("STARTING UPDATE-FROM-EXTERNAL-OBJECT-POINT-LIST", DebugHelper::CALCULATION);

        mDebugHelperPtr->write(std::stringstream() << "Number of viewports: " << viewportPointList.size(), DebugHelper::CALCULATION);

        for (unsigned int i = 0; i < viewportPointList.size(); i++) {
            ViewportPoint viewportPoint = viewportPointList.at(i);

            mDebugHelperPtr->write(std::stringstream() << "THIS IS VIEWPORT NR. " << i+1 << " IN THE LIST OF EXTERNAL VIEWPORTS.",
                                        DebugHelper::CALCULATION);
            mDebugHelperPtr->write(std::stringstream() << viewportPoint, DebugHelper::CALCULATION);

            ViewportPoint culledViewportPoint;
            if (!this->doFrustumCulling(viewportPoint.getPosition(), viewportPoint.getSimpleQuaternion(), this->getActiveIndices(), culledViewportPoint)) {
                mDebugHelperPtr->write("Viewpoint SKIPPED by Culling", DebugHelper::CALCULATION);
                continue;
            }

            ViewportPoint resultingViewportPoint;
            if (!culledViewportPoint.filterObjectPointCloudByTypes(viewportPoint.object_type_set, resultingViewportPoint)) {
                mDebugHelperPtr->write("Viewpoint SKIPPED by NameFiltering", DebugHelper::CALCULATION);
                continue;
            }

            mDebugHelperPtr->write(std::stringstream() << "Viewpoint TAKEN",
                                        DebugHelper::CALCULATION);

           this->updateObjectPointCloud(mObjectTypeSetPtr, resultingViewportPoint);
        }

        mDebugHelperPtr->writeNoticeably("ENDING UPDATE-FROM-EXTERNAL-OBJECT-POINT-LIST", DebugHelper::CALCULATION);
    }

    /*!
         * \brief Updates the point cloud under the assumption that the given viewport was chosen.
         * \param objectTypeSetPtr the object type names that shall be updated.
         * \param viewportPoint the viewport that was chosen
         * \return the number of deactivated normals
         */
    unsigned int updateObjectPointCloud(const ObjectTypeSetPtr &objectTypeSetPtr, const ViewportPoint &viewportPoint) {

        mDebugHelperPtr->write(std::stringstream() << "Number of active normals before update: " << getNumberActiveNormals(),
                                DebugHelper::CALCULATION);

        unsigned int deactivatedNormals = mHypothesisUpdaterPtr->update(objectTypeSetPtr, viewportPoint);

        mDebugHelperPtr->write(std::stringstream() << "Deactivated normals in viewport: " << deactivatedNormals, DebugHelper::CALCULATION);

        mDebugHelperPtr->write(std::stringstream() << "Number of active normals after update: " << getNumberActiveNormals(),
                                DebugHelper::CALCULATION);

        return deactivatedNormals;

    }

    /////
    ///
    // GETTER AND SETTER
    ///
    //////

    /**
         * Sets the point cloud points from point cloud message
         * @param message - message containing the point cloud
         */
    bool setPointCloudFromMessage(const asr_msgs::AsrAttributedPointCloud &msg) {
        // create a new point cloud
        ObjectPointCloudPtr originalPointCloudPtr = ObjectPointCloudPtr(new ObjectPointCloud());

        ObjectHelper objectHelper;

        // empty object name set
        mObjectTypeSetPtr = ObjectTypeSetPtr(new ObjectTypeSet);

        // put each element into the point cloud
        BOOST_FOREACH(asr_msgs::AsrAttributedPoint element, msg.elements) {
            // Create a new point with pose and set object type
            ObjectPoint pointCloudPoint(element.pose);
            pointCloudPoint.color.r = 0;
            pointCloudPoint.color.g = 255;
            pointCloudPoint.color.b = 0;
            pointCloudPoint.type = element.type;
            pointCloudPoint.identifier = element.identifier;

            // add type name to list if not already inserted
            if (mObjectTypeSetPtr->find(element.type) == mObjectTypeSetPtr->end())
                mObjectTypeSetPtr->insert(element.type);


            // translating from std::vector<geometry_msgs::Point> to std::vector<SimpleVector3>
            if (!mEnableCropBoxFiltering) {
                if (!setNormals(pointCloudPoint)) {
                    return false;
                }
            }

            //Insert color
            std_msgs::ColorRGBA colorByID = VisualizationHelper::getMeshColor(element.identifier);
            pointCloudPoint.color = colorByID;

            if(mEnableIntermediateObjectWeighting)
            {
                //get the weight for the object from world model
                IntermediateObjectWeightResponsePtr responsePtr_Intermediate = objectHelper.getIntermediateObjectValue(pointCloudPoint.type);

                if(responsePtr_Intermediate)
                {
                    pointCloudPoint.intermediate_object_weight = responsePtr_Intermediate->value;
                    //ROS_ERROR_STREAM("Set input cloud " << responsePtr_Intermediate->value);
                }
                else
                {
                    ROS_ERROR("Invalid object name %s or world model service call failed. Point Cloud not set!", pointCloudPoint.type.c_str());
                    return false;
                }
            }
            else
            {
                pointCloudPoint.intermediate_object_weight = 1;
            }

            // add point to array
            originalPointCloudPtr->push_back(pointCloudPoint);
        }

        ObjectPointCloudPtr outputPointCloudPtr;
        if(mEnableCropBoxFiltering)
        {
            IndicesPtr filteredObjectIndices = IndicesPtr(new Indices());
            mCropBoxFilterPtr->setInputCloud(originalPointCloudPtr);
            mCropBoxFilterPtr->filter(filteredObjectIndices);

            mDebugHelperPtr->write("setPointCloudFromMessage::Filtering point cloud finished.",
                                        DebugHelper::CALCULATION);

            mVisHelperPtr->triggerCropBoxVisualization(mCropBoxFilterPtr->getCropBoxWrapperPtrList());

            outputPointCloudPtr = ObjectPointCloudPtr(new ObjectPointCloud(*originalPointCloudPtr, *filteredObjectIndices));

            // we have to set now the object hypothesis normals
            for (ObjectPoint& pointCloudPoint : *outputPointCloudPtr) {
                if (!setNormalsInCropBoxMode(pointCloudPoint)) {
                    return false;
                }
            }
        }
        else
        {
            outputPointCloudPtr = originalPointCloudPtr;
        }

        //If point cloud is empty, getting the indices lead to an Error.
        if(outputPointCloudPtr->size() > 0)
        {
            // the active indices.
            IndicesPtr activeIndicesPtr = IndicesPtr(new Indices(outputPointCloudPtr->size()));
            boost::range::iota(boost::iterator_range<Indices::iterator>(activeIndicesPtr->begin(), activeIndicesPtr->end()), 0);


            // set the point cloud
            this->setActiveIndices(activeIndicesPtr);
        }
        else
        {
            mDebugHelperPtr->write("setPointCloudFromMessage::output point cloud is empty.",
                                        DebugHelper::CALCULATION);
        }

        this->setPointCloudPtr(outputPointCloudPtr);

        return true;
    }

private:

    /**
     * @brief generateSampleViewports
     * @param spaceSampleStartVector
     * @param contractor
     * @param pointCloudHeight
     * @return
     */
    ViewportPointCloudPtr generateSampleViewports(SimpleVector3 spaceSampleStartVector, double contractor, double pointCloudHeight) {
        SimpleQuaternionCollectionPtr feasibleOrientationsCollectionPtr = generateOrientationSamples();

        SamplePointCloudPtr sampledSpacePointCloudPtr = generateSpaceSamples(spaceSampleStartVector, contractor, pointCloudHeight);
        ROS_INFO_STREAM("sampledSpacePointCloudPtr->size: " << sampledSpacePointCloudPtr->size());

        ViewportPointCloudPtr sampleNextBestViewports = combineSamples(sampledSpacePointCloudPtr, feasibleOrientationsCollectionPtr);

        return sampleNextBestViewports;
    }

    /**
     * @brief generateOrientationSamples
     * @return
     */
    SimpleQuaternionCollectionPtr generateOrientationSamples() {
        SimpleQuaternionCollectionPtr sampledOrientationsPtr = mUnitSphereSamplerPtr->getSampledUnitSphere();
        SimpleQuaternionCollectionPtr feasibleOrientationsCollectionPtr(new SimpleQuaternionCollection());

        BOOST_FOREACH(SimpleQuaternion q, *sampledOrientationsPtr) {
            if (mRobotModelPtr->isPoseReachable(robot_model_services::SimpleVector3(0, 0, 0), q)) {
                feasibleOrientationsCollectionPtr->push_back(q);
            }
        }

        return feasibleOrientationsCollectionPtr;
    }

    /**
     * @brief generateSpaceSamples
     * @param spaceSampleStartVector
     * @param contractor
     * @param pointCloudHeight
     * @return
     */
    SamplePointCloudPtr generateSpaceSamples(SimpleVector3 spaceSampleStartVector, double contractor, double pointCloudHeight) {
        SamplePointCloudPtr sampledSpacePointCloudPtr = mSpaceSamplerPtr->getSampledSpacePointCloud(spaceSampleStartVector, contractor);

        //Set height of sample points as it is set to zero by space sampler
        this->setHeight(sampledSpacePointCloudPtr, pointCloudHeight);

        IndicesPtr feasibleIndicesPtr;
        //Prune space sample points in that iteration step by checking whether there are any surrounding object points (within constant far-clipping plane).
        this->getFeasibleSamplePoints(sampledSpacePointCloudPtr, feasibleIndicesPtr);

        return SamplePointCloudPtr(new SamplePointCloud(*sampledSpacePointCloudPtr, *feasibleIndicesPtr));
    }

    /**
     * @brief combines space samples and orientation samples to viewport samples
     * @param sampledSpacePointCloudPtr
     * @param sampledOrientationsPtr
     * @param feasibleIndicesPtr
     * @return
     */
    ViewportPointCloudPtr combineSamples(SamplePointCloudPtr sampledSpacePointCloudPtr, SimpleQuaternionCollectionPtr sampledOrientationsPtr) {
        ViewportPointCloudPtr sampleNextBestViewports = ViewportPointCloudPtr(new ViewportPointCloud());

        // convert space sampling combined with sphere sampling to viewport sampling
        for (SamplePoint &samplePoint : (*sampledSpacePointCloudPtr)) {
            SimpleVector3 samplePointCoords = samplePoint.getSimpleVector3();
            IndicesPtr samplePointChildIndices = samplePoint.child_indices;

            //For each space sample point: Go through all interesting orientations.
            mDebugHelperPtr->write("Iterating over all orientations for a given robot position.",
                                        DebugHelper::CALCULATION);
            BOOST_FOREACH(SimpleQuaternion orientation, *sampledOrientationsPtr) {
                // get the corresponding viewport
                ViewportPoint sampleViewport(samplePointCoords, orientation);
                // nearby object hypothesis for good estimation of possible hypothesis in frustum
                sampleViewport.child_indices = samplePointChildIndices;
                sampleViewport.object_type_set = mObjectTypeSetPtr;
                sampleNextBestViewports->push_back(sampleViewport);
            }
        }
        return sampleNextBestViewports;
    }

    bool setNormals(const ObjectPoint& pointCloudPoint) {
        ObjectHelper objectHelper;
        // get object type information
        ObjectMetaDataResponsePtr responsePtr_ObjectData = objectHelper.getObjectMetaData(pointCloudPoint.type);
        if (responsePtr_ObjectData) {
            // Get the rotation matrix to translate the normal vectors of the object.
            SimpleMatrix3 rotationMatrix = pointCloudPoint.getSimpleQuaternion().toRotationMatrix();
            int normalVectorCount = 0;
            for (geometry_msgs::Point point : responsePtr_ObjectData->normal_vectors) {
                SimpleVector3 normal(point.x, point.y, point.z);
                normal = rotationMatrix * normal;
                pointCloudPoint.active_normal_vectors->push_back(normalVectorCount);
                pointCloudPoint.normal_vectors->push_back(normal);
                ++normalVectorCount;
            }
        } else {
            ROS_ERROR("Invalid object name '%s' in point cloud or object_database node not started. Point Cloud not set!", pointCloudPoint.type.c_str());
            return false;
        }
        //Insert the meshpath
        objectsResources[pointCloudPoint.type] = responsePtr_ObjectData->object_mesh_resource;
        return true;
    }

    bool setNormalsInCropBoxMode(const ObjectPoint& pointCloudPoint) {
        SimpleMatrix3 rotationMatrix = pointCloudPoint.getSimpleQuaternion().toRotationMatrix();
        // in cropbox filtering we don't use the normals of the point clouds. Instead we use the ones defined in the xml
        auto cropBoxPtrList = mCropBoxFilterPtr->getCropBoxWrapperPtrList();
        int normalVectorCount = 0;
        bool foundNormals = false;
        bool isInCropbox = false;
        for (CropBoxWrapperPtr cropBoxWrapper : *cropBoxPtrList) {
            CropBoxPtr cropBoxPtr = cropBoxWrapper->getCropBox();
            Eigen::Vector4f max = cropBoxPtr->getMax();
            Eigen::Vector3f translation = cropBoxPtr->getTranslation();
            // check if pointCloudPoint is in the cropbox
            if (isPointInCropbox(pointCloudPoint.getPosition(), translation, max))
            {
                isInCropbox = true;
                if (!cropBoxWrapper->getCropBoxNormalsList()->empty()) {
                    foundNormals = true;
                    for (SimpleVector3 normal : *cropBoxWrapper->getCropBoxNormalsList()) {
                        pointCloudPoint.active_normal_vectors->push_back(normalVectorCount);
                        SimpleVector3 rotatedNormal = rotationMatrix * normal;
                        pointCloudPoint.normal_vectors->push_back(rotatedNormal);
                        ++normalVectorCount;
                    }
                }
            }
        }
        if (isInCropbox && !foundNormals) {
            if (!setNormals(pointCloudPoint)) {
                return false;
            }
        } else {
            //Insert the meshpath
            objectsResources[pointCloudPoint.type] = "package://asr_next_best_view/rsc/sphere.dae";
        }
        return true;
    }

    bool isPointInCropbox(const SimpleVector3& position, const Eigen::Vector3f& translation, const Eigen::Vector4f& max) const {
        return isPointInRange(position(0,0), translation(0,0), max(0,0)) &&
                isPointInRange(position(1,0), translation(1,0), max(1,0)) &&
                isPointInRange(position(2,0), translation(2,0), max(2,0));
    }

    bool isPointInRange(float point, float min, float lenght) const {
        return point >= min && point <= (min + lenght);
    }

public:
    /**
         * Returns the path to a meshs resource file
         */
    std::string getMeshPathByName(std::string objectType)
    {
        if(this->objectsResources.find(objectType) != this->objectsResources.end())
        {
            return this->objectsResources[objectType];
        }
        else
        {
            return "-2";
        }
    }


    /**
         * Sets the point cloud ptr.
         */
    void setPointCloudPtr(const ObjectPointCloudPtr &pointCloudPtr) {
        mPointCloudPtr = pointCloudPtr;

        mKdTreePtr = KdTreePtr(new KdTree());
        mKdTreePtr->setInputCloud(mPointCloudPtr);
        mRatingModulePtr->setInputCloud(mPointCloudPtr);
        mCameraModelFilterPtr->setInputCloud(mPointCloudPtr);
        for (int i : boost::irange(0, mNumberOfThreads)) {
            mThreadRatingModules[i]->setInputCloud(mPointCloudPtr);
            mThreadCameraModels[i]->setInputCloud(mPointCloudPtr);
        }
        mSpaceSamplerPtr->setInputCloud(mPointCloudPtr);
    }

    void loadCropBoxListFromFile(const std::string mCropBoxListFilePath)
    {
        this->mCropBoxFilterPtr = CropBoxFilterPtr(new CropBoxFilter(mCropBoxListFilePath));
    }

    void setEnableCropBoxFiltering(const bool mEnableCropBoxFiltering)
    {
        this->mEnableCropBoxFiltering = mEnableCropBoxFiltering;
    }

    void setEnableIntermediateObjectWeighting(const bool mEnableIntermediateObjectWeighting)
    {
        this->mEnableIntermediateObjectWeighting = mEnableIntermediateObjectWeighting;
    }

    /**
         * @return the point cloud pointer
         */
    ObjectPointCloudPtr getPointCloudPtr() {
        return mPointCloudPtr;
    }

    /*!
         * \brief sets the active indices.
         */
    void setActiveIndices(const IndicesPtr &activeIndicesPtr) {
        mActiveIndicesPtr = activeIndicesPtr;
    }

    /*!
         * \return the active Indices.
         */
    IndicesPtr getActiveIndices() {
        return mActiveIndicesPtr;
    }

    int getNumberActiveNormals() {
        int result = 0;

        ObjectPointCloud objectPointCloud = ObjectPointCloud(*getPointCloudPtr(), *getActiveIndices());
        for (ObjectPoint &objPoint : objectPointCloud) {
            result += objPoint.active_normal_vectors->size();
        }

        return result;
    }

    unsigned int getNumberTotalNormals(std::string type, std::string identifier) {
        int result = 0;

        ObjectPointCloud objectPointCloud = ObjectPointCloud(*getPointCloudPtr(), *getActiveIndices());
        for (ObjectPoint &objPoint : objectPointCloud) {
            if (objPoint.identifier.compare(identifier) != 0 || objPoint.type.compare(type) != 0)
                continue;
            result += objPoint.normal_vectors->size();
        }

        return result;
    }

    unsigned int getNumberActiveNormals(std::string type, std::string identifier) {
        int result = 0;

        ObjectPointCloud objectPointCloud = ObjectPointCloud(*getPointCloudPtr(), *getActiveIndices());
        for (ObjectPoint &objPoint : objectPointCloud) {
            if (objPoint.identifier.compare(identifier) != 0 || objPoint.type.compare(type) != 0)
                continue;
            result += objPoint.active_normal_vectors->size();
        }

        return result;
    }

    std::vector<std::pair<std::string, std::string>> getTypeAndIds() {
        std::vector<std::pair<std::string, std::string>> result;
        ObjectPointCloud objectPointCloud = ObjectPointCloud(*getPointCloudPtr(), *getActiveIndices());
        std::set<std::string> foundObjectTypeAndIds;
        for (ObjectPoint &objPoint : objectPointCloud) {
            std::string currentObjTypeAndId = objPoint.type + objPoint.identifier;
            if (foundObjectTypeAndIds.find(currentObjTypeAndId) == foundObjectTypeAndIds.end()) {
                foundObjectTypeAndIds.insert(currentObjTypeAndId);
                result.push_back(std::make_pair(objPoint.type, objPoint.identifier));
            }
        }
        return result;
    }

    bool removeObjects(std::string type, std::string identifier) {
        auto it = mPointCloudPtr->begin();
        while (it != mPointCloudPtr->end()) {
            ObjectPoint objPoint = *it;
            if (objPoint.identifier.compare(identifier) != 0 || objPoint.type.compare(type) != 0) {
                it ++;
            } else {
                it = mPointCloudPtr->erase(it);
            }
        }
        if (mPointCloudPtr->empty()) {
            return false;
        }
        // the active indices.
        IndicesPtr activeIndicesPtr = IndicesPtr(new Indices(mPointCloudPtr->size()));
        boost::range::iota(boost::iterator_range<Indices::iterator>(activeIndicesPtr->begin(), activeIndicesPtr->end()), 0);

        // set the point cloud
        this->setActiveIndices(activeIndicesPtr);
        setPointCloudPtr(mPointCloudPtr);
        return true;
    }

    /**
         * @return the point cloud pointer
         */
    KdTreePtr getKdTreePtr() {
        return mKdTreePtr;
    }

    /**
         * Sets the unit sphere sampler.
         * @param unitSphereSamplerPtr - the pointer to the unit sphere sampler
         */
    void setUnitSphereSampler(const UnitSphereSamplerPtr &unitSphereSamplerPtr) {
        mUnitSphereSamplerPtr = unitSphereSamplerPtr;
    }

    /**
         * @return the unit sphere sampler.
         */
    UnitSphereSamplerPtr getUnitSphereSampler() {
        return mUnitSphereSamplerPtr;
    }

    /**
         * Sets the space sampler.
         * @param spaceSamplerPtr - the pointer to the space sampler
         */
    void setSpaceSampler(const SpaceSamplerPtr &spaceSamplerPtr) {
        mSpaceSamplerPtr = spaceSamplerPtr;
    }

    /**
         * @return the pointer to the space sampler.
         */
    SpaceSamplerPtr getSpaceSampler() {
        return mSpaceSamplerPtr;
    }

    /**
         * sets the robot model
         * @param robotModelPtr - the pointer to robot model.
         */
    void setRobotModel(const robot_model_services::RobotModelPtr &robotModelPtr) {
        mRobotModelPtr = robotModelPtr;
    }

    /**
         *@return the robot ptr
         */
    robot_model_services::RobotModelPtr getRobotModel() {
        return mRobotModelPtr;
    }

    /**
         * Sets the camera model filter.
         * @param cameraModelFilterPtr - the pointer to the camera model filter.
         */
    void setCameraModelFilter(const CameraModelFilterPtr &cameraModelFilterPtr) {
        mCameraModelFilterPtr = cameraModelFilterPtr;
    }

    /**
         * @return the pointer to the camera model filter
         */
    CameraModelFilterPtr getCameraModelFilter() {
        return mCameraModelFilterPtr;
    }

    /**
         * Sets the rating module.
         * @param ratingModulePtr - the pointer to the rating module
         */
    void setRatingModule(const RatingModulePtr &ratingModulePtr) {
        mRatingModulePtr = ratingModulePtr;
    }

    /**
         * @return the pointer to the rating module.
         */
    RatingModulePtr getRatingModule() {
        return mRatingModulePtr;
    }

    /**
      * sets the map helper
      * @param mapHelperPtr - the pointer to map helper.
      */
    void setMapHelper(const MapHelperPtr &mapHelperPtr) {
        mMapHelperPtr = mapHelperPtr;
        mVisHelperPtr = VisualizationHelperPtr(new VisualizationHelper(mMapHelperPtr));
    }

    /**
      *@return the map helper ptr
      */
    MapHelperPtr getMapHelper() {
        return mMapHelperPtr;
    }

    void setHypothesisUpdater(const HypothesisUpdaterPtr &hypothesisUpdaterPtr) {
        mHypothesisUpdaterPtr = hypothesisUpdaterPtr;
    }

    HypothesisUpdaterPtr getHypothesisUpdater() {
        return mHypothesisUpdaterPtr;
    }

    /**
         * @param epsilon - error threshold
         */
    void setEpsilon(float epsilon) {
        mEpsilon = epsilon;
    }

    /**
         * @return epsilon
         */
    float getEpsilon() {
        return mEpsilon;
    }

    /**
     * Max number of hierarchical iterations when calculating a next-best-view.
         * @param maxIterationSteps  - max number of iteration steps
         */
    void setMaxIterationSteps(int maxIterationSteps)
    {
        mMaxIterationSteps  = maxIterationSteps;
    }

    /**
     * @brief setRatingModuleAbstractFactoryPtr used to generate rating modules per thread.
     * @param ratingModuleAbstractFactoryPtr
     */
    void setRatingModuleAbstractFactoryPtr(const RatingModuleAbstractFactoryPtr &ratingModuleAbstractFactoryPtr) {
        mRatingModuleAbstractFactoryPtr = ratingModuleAbstractFactoryPtr;
        for (int i : boost::irange(0, mNumberOfThreads)) {
            if (mThreadRatingModules[i]) {
                mThreadRatingModules[i].reset();
            }
            mThreadRatingModules[i] = mRatingModuleAbstractFactoryPtr->createRatingModule();
        }
    }

    /**
     * @brief getRatingModuleAbstractFactoryPtr
     * @return
     */
    RatingModuleAbstractFactoryPtr getRatingModuleAbstractFactoryPtr() {
        return mRatingModuleAbstractFactoryPtr;
    }

    /**
     * @brief setCameraModelFilterAbstractFactoryPtr used to generate camera models per thread.
     * @param cameraModelFilterAbstractFactoryPtr
     */
    void setCameraModelFilterAbstractFactoryPtr(const CameraModelFilterAbstractFactoryPtr &cameraModelFilterAbstractFactoryPtr) {
        mCameraModelFilterAbstractFactoryPtr = cameraModelFilterAbstractFactoryPtr;
        for (int i : boost::irange(0, mNumberOfThreads)) {
            if (mThreadCameraModels[i]) {
                mThreadCameraModels[i].reset();
            }
            mThreadCameraModels[i] = mCameraModelFilterAbstractFactoryPtr->createCameraModelFilter();
        }
    }

    /**
     * @brief getCameraModelFilterAbstractFactoryPtr
     * @return
     */
    CameraModelFilterAbstractFactoryPtr getCameraModelFilterAbstractFactoryPtr() {
        return mCameraModelFilterAbstractFactoryPtr;
    }

    /**
     * @brief getNumberOfThreads
     * @return
     */
    int getNumberOfThreads() const {
        return mNumberOfThreads;
    }

    /**
     * @brief sets number of threads used to rate samples, negative numbers use boost::thread::hardware_concurrency() as number of threads.
     * @param value
     */
    void setNumberOfThreads(int value) {
        if (value < 0) {
            mNumberOfThreads = boost::thread::hardware_concurrency();
        } else {
            mNumberOfThreads = value;
        }
        // reinitialize camera modules and rating modules per thread
        mThreadCameraModels.clear();
        mThreadRatingModules.clear();
        mThreadCameraModels.resize(mNumberOfThreads);
        mThreadRatingModules.resize(mNumberOfThreads);
        for (int i : boost::irange(0, mNumberOfThreads)) {
            // set RatingModule per thread
            if (mThreadRatingModules[i]) {
                mThreadRatingModules[i].reset();
            }
            mThreadRatingModules[i] = mRatingModuleAbstractFactoryPtr->createRatingModule();
            mThreadRatingModules[i]->setInputCloud(mPointCloudPtr);

            // set CameraModule per thread
            if (mThreadCameraModels[i]) {
                mThreadCameraModels[i].reset();
            }
            mThreadCameraModels[i] = mCameraModelFilterAbstractFactoryPtr->createCameraModelFilter();
            mThreadCameraModels[i]->setInputCloud(mPointCloudPtr);
        }
    }

    double getMinUtility() const {
        return mMinUtility;
    }

    void setMinUtility(double minUtility) {
        mMinUtility = minUtility;
    }

    bool getRequireMinUtility() const {
        return mRequireMinUtility;
    }

    void setRequireMinUtility(bool requireMinUtility) {
        mRequireMinUtility = requireMinUtility;
    }
};

}
