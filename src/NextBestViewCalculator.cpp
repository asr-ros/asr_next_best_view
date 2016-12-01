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

#include "next_best_view/NextBestViewCalculator.hpp"

namespace next_best_view {

    NextBestViewCalculator::NextBestViewCalculator(const UnitSphereSamplerPtr & unitSphereSamplerPtr,
                                                   const MapHelperPtr &mapHelperPtr,
                                                   const SpaceSamplerPtr &spaceSamplerPtr,
                                                   const RobotModelPtr &robotModelPtr,
                                                   const CameraModelFilterPtr &cameraModelFilterPtr,
                                                   const RatingModulePtr &ratingModulePtr)
        : enable_shared_from_this(),
          objectsResources(),
          mUnitSphereSamplerPtr(unitSphereSamplerPtr),
          mMapHelperPtr(mapHelperPtr),
          mSpaceSamplerPtr(spaceSamplerPtr),
          mRobotModelPtr(robotModelPtr),
          mCameraModelFilterPtr(cameraModelFilterPtr),
          mRatingModulePtr(ratingModulePtr),
          mEpsilon(10E-3),
          mNumberOfThreads(boost::thread::hardware_concurrency()),
          mThreadCameraModels(mNumberOfThreads),
          mThreadRatingModules(mNumberOfThreads),
          mNBVCachePtr(new NextBestViewCache()) {

        mFirstNBVCallIsRunning = false;
//        setMapHelper(mapHelperPtr);
        mDebugHelperPtr = DebugHelper::getInstance();
    }

    bool NextBestViewCalculator::calculateNextBestView(const ViewportPoint &currentCameraViewport, ViewportPoint &resultViewport) {
        auto begin = std::chrono::high_resolution_clock::now();
        mDebugHelperPtr->writeNoticeably("STARTING CALCULATE-NEXT-BEST-VIEW METHOD", DebugHelper::CALCULATION);

        if (mEnablePrediction && !mFirstNBVCallIsRunning) {
            // NBVPrediction calls this method, so we have to make sure we won't loop call this method.
            mFirstNBVCallIsRunning = true;
            ViewportPointCloudPtr resultViewports = mNBVPredictionPtr->getNBVPredictions(currentCameraViewport);
            mFirstNBVCallIsRunning = false;
            if (resultViewports->empty()) {
                return false;
            }
            resultViewport = resultViewports->at(0);
            return true;
        }

        //Calculate robot configuration corresponding to current camera viewport of robot.
        initializeRobotState(currentCameraViewport);

        mDebugHelperPtr->write("Calculate discrete set of view orientations on unit sphere",
                               DebugHelper::CALCULATION);
        //Get discretized set of camera orientations (pan, tilt) that are to be considered at each robot position considered during iterative optimization.
        SimpleQuaternionCollectionPtr feasibleOrientationsCollectionPtr = generateOrientationSamples();

        bool success = false;
        if (mCacheResults && !mNBVCachePtr->isEmpty()) {
            success = this->getNBVFromCache(currentCameraViewport, resultViewport);
        } else {
            // create the next best view point cloud
            success = this->doIteration(currentCameraViewport, feasibleOrientationsCollectionPtr, resultViewport);
        }

        auto finish = std::chrono::high_resolution_clock::now();
        // cast timediff to flaot in seconds
        ROS_INFO_STREAM("nbv calculation took " << std::chrono::duration<float>(finish-begin).count() << " seconds.");
        mDebugHelperPtr->writeNoticeably("ENDING CALCULATE-NEXT-BEST-VIEW METHOD", DebugHelper::CALCULATION);
        return success;
    }

    void NextBestViewCalculator::initializeRobotState(const ViewportPoint &currentCameraViewport) {
        RobotStatePtr currentState = mRobotModelPtr->calculateRobotState(currentCameraViewport.getPosition(), currentCameraViewport.getSimpleQuaternion());
        //Save it.
        mRobotModelPtr->setCurrentRobotState(currentState);
        mRatingModulePtr->setRobotState(currentState);
        for (int i : boost::irange(0, mNumberOfThreads)) {
            mThreadRatingModules[i]->setRobotState(currentState);
        }
    }

    void NextBestViewCalculator::getFeasibleSamplePoints(const SamplePointCloudPtr &sampledSpacePointCloudPtr, IndicesPtr &resultIndicesPtr) {
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

    void NextBestViewCalculator::getFeasibleViewports(const ViewportPointCloudPtr &sampleViewportsPtr, IndicesPtr &resultIndicesPtr) {
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

    bool NextBestViewCalculator::getFeasibleHypothesis(SimpleVector3 samplePoint, IndicesPtr &resultIndicesPtr) {
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

    bool NextBestViewCalculator::rateViewports(const ViewportPointCloudPtr &sampleNextBestViewportsPtr, const ViewportPoint &currentCameraViewport, ViewportPoint &resultViewport, bool objectTypeSetIsKnown) {
        ViewportPointCloudPtr ratedNextBestViewportsPtr = ViewportPointCloudPtr(new ViewportPointCloud());
        return rateViewportsInternal(sampleNextBestViewportsPtr, currentCameraViewport, ratedNextBestViewportsPtr, resultViewport, objectTypeSetIsKnown);
    }

    bool NextBestViewCalculator::rateViewports(const ViewportPointCloudPtr &sampleNextBestViewports, const ViewportPoint &currentCameraViewport, ViewportPointCloudPtr &ratedNextBestViewportsPtr, bool objectTypeSetIsKnown) {
        ratedNextBestViewportsPtr = ViewportPointCloudPtr(new ViewportPointCloud());
        ViewportPoint resultViewport;
        return rateViewportsInternal(sampleNextBestViewports, currentCameraViewport, ratedNextBestViewportsPtr, resultViewport, objectTypeSetIsKnown);
    }

    bool NextBestViewCalculator::rateViewportsInternal(const ViewportPointCloudPtr &sampleNextBestViewports, const ViewportPoint &currentCameraViewport,
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

    void NextBestViewCalculator::ratingThread(int threadId, boost::mutex &mutex,
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

    bool NextBestViewCalculator::rateSingleViewportOptimizeObjectTypes(const RatingModulePtr &ratingModulePtr, const ViewportPoint &currentCameraViewport, ViewportPoint &fullViewportPoint) {
        mDebugHelperPtr->write("Getting viewport with optimal object constellation for given position & orientation combination.",
                               DebugHelper::RATING);
        return ratingModulePtr->setBestScoreContainer(currentCameraViewport, fullViewportPoint);
    }

    bool NextBestViewCalculator::rateSingleViewportFixedObjectTypes(const RatingModulePtr &ratingModulePtr, const ViewportPoint &currentCameraViewport, ViewportPoint &fullViewportPoint) {
        ratingModulePtr->resetCache();
        return ratingModulePtr->setSingleScoreContainer(currentCameraViewport, fullViewportPoint);
    }

    bool NextBestViewCalculator::getNBVFromCache(const ViewportPoint &currentCameraViewport, ViewportPoint &resultViewport) {
        ROS_INFO_STREAM("using cached nbv");
        ViewportPointCloudPtr samples = mNBVCachePtr->getAllBestViewports();
        // TODO: configureable/do other stuff/ga
        if (samples->size() > 10) {
            samples->resize(30);
        }

        // rate best ones
        ViewportPointCloudPtr ratedNextBestViewportsPtr;
        if (!rateViewports(samples, currentCameraViewport, ratedNextBestViewportsPtr)) {
            return false;
        }
        auto ratingSortFunction = [this](const ViewportPoint &a, const ViewportPoint &b) {
            // a < b
            return mRatingModulePtr->compareViewports(a, b);
        };
        std::sort(ratedNextBestViewportsPtr->begin(), ratedNextBestViewportsPtr->end(), ratingSortFunction);

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
        return true;
    }

    bool NextBestViewCalculator::doIteration(const ViewportPoint &currentCameraViewport, const SimpleQuaternionCollectionPtr &sampledOrientationsPtr, ViewportPoint &resultViewport) {
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
            if (iterationStep >= mMinIterationSteps && // we have to run at least min number of iterations
                    (abs(rating - currentBestRating) <= this->getEpsilon() || iterationStep >= mMaxIterationSteps)) {
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

    bool NextBestViewCalculator::doIterationStep(const ViewportPoint &currentCameraViewport, const ViewportPoint &currentBestViewport,
                                                 const SimpleQuaternionCollectionPtr &sampledOrientationsPtr, float contractor,
                                                 ViewportPoint &resultViewport, int iterationStep) {
        mDebugHelperPtr->writeNoticeably("STARTING DO-ITERATION-STEP METHOD", DebugHelper::CALCULATION);

        // current camera position
        SimpleVector3 currentBestPosition = currentBestViewport.getPosition();


        ViewportPointCloudPtr sampleNextBestViewports;
        if (!mEnableGA || iterationStep < mMinIterationGA) {
            // we do normal sampling
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
            sampleNextBestViewports = combineSamples(sampledSpacePointCloudPtr, sampledOrientationsPtr);
        } else {
            // we use ga and cache to generate nbv samples
            sampleNextBestViewports = mViewMutationPtr->selectAndMutate(mRatedSortedViewportsPreIteration, iterationStep);
        }

        // rate
        ViewportPointCloudPtr ratedNextBestViewportsPtr;
        if (!rateViewports(sampleNextBestViewports, currentCameraViewport, ratedNextBestViewportsPtr)) {
            mDebugHelperPtr->writeNoticeably("ENDING DO-ITERATION-STEP METHOD", DebugHelper::CALCULATION);
            return false;
        }

        // sort
        // ascending -> last element has best rating
        auto ratingSortFunction = [this](const ViewportPoint &a, const ViewportPoint &b) {
            // a < b
            return mRatingModulePtr->compareViewports(a, b);
        };
        std::sort(ratedNextBestViewportsPtr->begin(), ratedNextBestViewportsPtr->end(), ratingSortFunction);
        mRatedSortedViewportsPreIteration = ratedNextBestViewportsPtr;

        // filter too low utility
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

        if (mCacheResults) {
            // update cache grid
            // TODO: copy, otherwise it interferes with mRatedSortedViewportsPreIteration
            mNBVCachePtr->updateCache(ratedNextBestViewportsPtr);
        }
        //        mVisHelperPtr->triggerSamplingVisualization(ratedNextBestViewportsPtr, Color(1, 0, 0, 1), "ratedViewports");

        //Visualize iteration step and its result.
        mVisHelperPtr->triggerIterationVisualization(iterationStep, sampledOrientationsPtr, resultViewport,
                                                     ratedNextBestViewportsPtr, mSpaceSamplerPtr);

        mDebugHelperPtr->writeNoticeably("ENDING DO-ITERATION-STEP METHOD", DebugHelper::CALCULATION);
        return true;
    }

    void NextBestViewCalculator::setHeight(SamplePointCloudPtr pointCloudPtr, double height) {
        for (unsigned int i = 0; i < pointCloudPtr->size(); i++) {
            pointCloudPtr->at(i).z = height;
        }
    }

    bool NextBestViewCalculator::doFrustumCulling(ViewportPoint &resultViewportPoint) {
        return doFrustumCulling(mCameraModelFilterPtr, resultViewportPoint);
    }

    bool NextBestViewCalculator::doFrustumCulling(const CameraModelFilterPtr &cameraModelFilterPtr, ViewportPoint &resultViewportPoint) {
        cameraModelFilterPtr->setInputPointIndices(resultViewportPoint.child_indices);
        cameraModelFilterPtr->setPivotPointPose(resultViewportPoint.getPosition(), resultViewportPoint.getSimpleQuaternion());

        // do the frustum culling
        IndicesPtr frustumIndicesPtr;
        //Call wrapper (with next-best-view data structures) for PCL frustum culling call.
        cameraModelFilterPtr->filter(frustumIndicesPtr);

        resultViewportPoint.child_indices = frustumIndicesPtr;
        resultViewportPoint.child_point_cloud = cameraModelFilterPtr->getInputPointCloud();
        resultViewportPoint.point_cloud = mPointCloudPtr;

        if (frustumIndicesPtr->size() == 0) {
            return false;
        }

        return true;
    }

    void NextBestViewCalculator::updateFromExternalViewportPointList(const std::vector<ViewportPoint> &viewportPointList) {
        mDebugHelperPtr->writeNoticeably("STARTING UPDATE-FROM-EXTERNAL-OBJECT-POINT-LIST", DebugHelper::CALCULATION);

        mDebugHelperPtr->write(std::stringstream() << "Number of viewports: " << viewportPointList.size(), DebugHelper::CALCULATION);

        for (unsigned int i = 0; i < viewportPointList.size(); i++) {
            ViewportPoint viewportPoint = viewportPointList.at(i);

            mDebugHelperPtr->write(std::stringstream() << "THIS IS VIEWPORT NR. " << i+1 << " IN THE LIST OF EXTERNAL VIEWPORTS.",
                                   DebugHelper::CALCULATION);
            mDebugHelperPtr->write(std::stringstream() << viewportPoint, DebugHelper::CALCULATION);

            ViewportPoint culledViewportPoint(viewportPoint.getPosition(), viewportPoint.getSimpleQuaternion());
            culledViewportPoint.child_indices = this->getActiveIndices();
            if (!this->doFrustumCulling(culledViewportPoint)) {
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

    unsigned int NextBestViewCalculator::updateObjectPointCloud(const ObjectTypeSetPtr &objectTypeSetPtr, const ViewportPoint &viewportPoint) {

        mDebugHelperPtr->write(std::stringstream() << "Number of active normals before update: " << getNumberActiveNormals(),
                               DebugHelper::CALCULATION);

        unsigned int deactivatedNormals = mHypothesisUpdaterPtr->update(objectTypeSetPtr, viewportPoint);

        mDebugHelperPtr->write(std::stringstream() << "Deactivated normals in viewport: " << deactivatedNormals, DebugHelper::CALCULATION);

        mDebugHelperPtr->write(std::stringstream() << "Number of active normals after update: " << getNumberActiveNormals(),
                               DebugHelper::CALCULATION);

        return deactivatedNormals;

    }

    bool NextBestViewCalculator::setPointCloudFromMessage(const pbd_msgs::PbdAttributedPointCloud &msg) {
        // create a new point cloud
        ObjectPointCloudPtr originalPointCloudPtr = ObjectPointCloudPtr(new ObjectPointCloud());

        ObjectHelper objectHelper;

        // empty object name set
        mObjectTypeSetPtr = ObjectTypeSetPtr(new ObjectTypeSet);

        // put each element into the point cloud
        BOOST_FOREACH(pbd_msgs::PbdAttributedPoint element, msg.elements) {
            // Create a new point with pose and set object type
            ObjectPoint pointCloudPoint(element.pose);
            pointCloudPoint.r = 0;
            pointCloudPoint.g = 255;
            pointCloudPoint.b = 0;
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
            mCropBoxFilterPtr->setInputPointCloud(originalPointCloudPtr);
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

        // TODO: generate cluster if enabled

        this->setPointCloudPtr(outputPointCloudPtr);

        if (mRemoveInvalidNormals) {
            filterUnrechableNormals();
        }

        if (mCacheResults) {
            mNBVCachePtr->clearCache();
        }

        return true;
    }

    void NextBestViewCalculator::filterUnrechableNormals() {
        ROS_INFO_STREAM("filtering now unreachable normals");
        auto begin = std::chrono::high_resolution_clock::now();
        // TODO always use hypothesissampler
        // TODO remove objects with 0 normals?
        // hypothesissampler samples at all locations, so the position does not matter much
        ViewportPointCloudPtr sampleNextBestViewports = generateSampleViewports(SimpleVector3(0, 0, 0), 1.0 / pow(2.0, 1.0), 1.32);

        // remove all removeable normals
        for (ViewportPoint sample : *sampleNextBestViewports) {
            if (!doFrustumCulling(mCameraModelFilterPtr, sample))
                continue;
            mHypothesisUpdaterPtr->update(mObjectTypeSetPtr, sample);
        }

        // show samples
        mVisHelperPtr->resetSamplingVisualization();
        mVisHelperPtr->triggerSamplingVisualization(sampleNextBestViewports, Color(1, 0, 1, 1), "ratedViewports");

        // for each object make invalid normals -> valid normals
        for (ObjectPoint &o : *mPointCloudPtr) {
            // debug print invalid normals
            if (mDebugHelperPtr->getLevel() & DebugHelper::CALCULATION) {
                std::stringstream ssInvalidNormals;
                for(size_t i = 0; i < o.active_normal_vectors->size(); ++i) {
                    if(i != 0) {
                        ssInvalidNormals << ",";
                    }
                    ssInvalidNormals << (*o.active_normal_vectors)[i];
                }
                std::string s = ssInvalidNormals.str();
                mDebugHelperPtr->write(std::stringstream() << "invalid ones: " << s, DebugHelper::CALCULATION);
            }

            // copy remaining normals = normals that cannot be removed
            Indices invalidNormalVectorIndices = *o.active_normal_vectors;
            std::sort(invalidNormalVectorIndices.begin(), invalidNormalVectorIndices.end());

            // all normal vector indices
            Indices allNormalIndices(o.normal_vectors->size());
            std::iota(allNormalIndices.begin(), allNormalIndices.end(), 0);

            // all normal vector indices \ invalid ones
            IndicesPtr validNormalVectors(new Indices(allNormalIndices.size()));
            auto it = std::set_difference(allNormalIndices.begin(), allNormalIndices.end(), invalidNormalVectorIndices.begin(), invalidNormalVectorIndices.end(), validNormalVectors->begin());
            validNormalVectors->resize(it - validNormalVectors->begin());
            o.active_normal_vectors = validNormalVectors;

            // set normal_vectors
            SimpleVector3CollectionPtr newNormalVectors = SimpleVector3CollectionPtr(new SimpleVector3Collection());
            for (int i : *o.active_normal_vectors) {
                newNormalVectors->push_back(o.normal_vectors->at(i));
            }
            o.normal_vectors = newNormalVectors;
            // now all normal vectors are active
            std::iota(o.active_normal_vectors->begin(), o.active_normal_vectors->end(), 0);

            // debug print valid normals
            if (mDebugHelperPtr->getLevel() & DebugHelper::CALCULATION) {
                std::stringstream ssValidNormals;
                for(size_t i = 0; i < o.active_normal_vectors->size(); ++i) {
                    if(i != 0) {
                        ssValidNormals << ",";
                    }
                    ssValidNormals << (*o.active_normal_vectors)[i];
                }
                std::string s = ssValidNormals.str();
                mDebugHelperPtr->write(std::stringstream() << "valid ones: " << s, DebugHelper::CALCULATION);
            }
        }
        auto finish = std::chrono::high_resolution_clock::now();
        ROS_INFO_STREAM("removing invalid normals took " << std::chrono::duration<float>(finish - begin).count() << " seconds.");
    }

    ViewportPointCloudPtr NextBestViewCalculator::generateSampleViewports(SimpleVector3 spaceSampleStartVector, double contractor, double pointCloudHeight) {
        SimpleQuaternionCollectionPtr feasibleOrientationsCollectionPtr = generateOrientationSamples();

        SamplePointCloudPtr sampledSpacePointCloudPtr = generateSpaceSamples(spaceSampleStartVector, contractor, pointCloudHeight);

        ViewportPointCloudPtr sampleNextBestViewports = combineSamples(sampledSpacePointCloudPtr, feasibleOrientationsCollectionPtr);

        return sampleNextBestViewports;
    }

    SimpleQuaternionCollectionPtr NextBestViewCalculator::generateOrientationSamples() {
        SimpleQuaternionCollectionPtr sampledOrientationsPtr = mUnitSphereSamplerPtr->getSampledUnitSphere();
        SimpleQuaternionCollectionPtr feasibleOrientationsCollectionPtr(new SimpleQuaternionCollection());

        BOOST_FOREACH(SimpleQuaternion q, *sampledOrientationsPtr) {
            if (mRobotModelPtr->isPoseReachable(SimpleVector3(0, 0, 0), q)) {
                feasibleOrientationsCollectionPtr->push_back(q);
            }
        }

        return feasibleOrientationsCollectionPtr;
    }

    SamplePointCloudPtr NextBestViewCalculator::generateSpaceSamples(SimpleVector3 spaceSampleStartVector, double contractor, double pointCloudHeight) {
        SamplePointCloudPtr sampledSpacePointCloudPtr = mSpaceSamplerPtr->getSampledSpacePointCloud(spaceSampleStartVector, contractor);

        //Set height of sample points as it is set to zero by space sampler
        this->setHeight(sampledSpacePointCloudPtr, pointCloudHeight);

        // generate all indices
        // TODO: filterchain class/setpostfilter/add option to disable filters
        IndicesPtr resultIndicesPtr(new Indices(sampledSpacePointCloudPtr->size()));
        boost::range::iota(boost::iterator_range<Indices::iterator>(resultIndicesPtr->begin(), resultIndicesPtr->end()), 0);
        IndicesPtr filteredIndicesPtr;
        mSpaceSamplingFilterChainPtr->setInputPointCloud(sampledSpacePointCloudPtr);
        mSpaceSamplingFilterChainPtr->setInputPointIndices(resultIndicesPtr);
        auto begin = std::chrono::high_resolution_clock::now();
        mSpaceSamplingFilterChainPtr->filter(filteredIndicesPtr);
        auto finish = std::chrono::high_resolution_clock::now();
        // cast timediff to flaot in seconds
        ROS_INFO_STREAM("space sample filter chain took " << std::chrono::duration<float>(finish-begin).count() << " seconds.");

        return SamplePointCloudPtr(new SamplePointCloud(*sampledSpacePointCloudPtr, *filteredIndicesPtr));
    }

    ViewportPointCloudPtr NextBestViewCalculator::combineSamples(SamplePointCloudPtr sampledSpacePointCloudPtr, SimpleQuaternionCollectionPtr sampledOrientationsPtr) {
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

    bool NextBestViewCalculator::setNormals(const ObjectPoint& pointCloudPoint) {
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

    bool NextBestViewCalculator::setNormalsInCropBoxMode(const ObjectPoint& pointCloudPoint) {
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
            objectsResources[pointCloudPoint.type] = "package://next_best_view/rsc/sphere.dae";
        }
        return true;
    }

    bool NextBestViewCalculator::isPointInCropbox(const SimpleVector3& position, const Eigen::Vector3f& translation, const Eigen::Vector4f& max) const {
        return isPointInRange(position(0,0), translation(0,0), max(0,0)) &&
                isPointInRange(position(1,0), translation(1,0), max(1,0)) &&
                isPointInRange(position(2,0), translation(2,0), max(2,0));
    }

    bool NextBestViewCalculator::isPointInRange(float point, float min, float lenght) const {
        return point >= min && point <= (min + lenght);
    }

    std::string NextBestViewCalculator::getMeshPathByName(std::string objectType) {
        if(this->objectsResources.find(objectType) != this->objectsResources.end())
        {
            return this->objectsResources[objectType];
        }
        else
        {
            return "-2";
        }
    }

    void NextBestViewCalculator::setPointCloudPtr(const ObjectPointCloudPtr &pointCloudPtr) {
        mPointCloudPtr = pointCloudPtr;

        mKdTreePtr = KdTreePtr(new KdTree());
        mKdTreePtr->setInputCloud(mPointCloudPtr);
        mRatingModulePtr->setObjectPointCloud(mPointCloudPtr);
        mCameraModelFilterPtr->setObjectPointCloud(mPointCloudPtr);
        mCameraModelFilterPtr->setInputPointCloud(mPointCloudPtr);
        for (int i : boost::irange(0, mNumberOfThreads)) {
            mThreadRatingModules[i]->setObjectPointCloud(mPointCloudPtr);
            mThreadCameraModels[i]->setObjectPointCloud(mPointCloudPtr);
            mThreadCameraModels[i]->setInputPointCloud(mPointCloudPtr);
        }
        mSpaceSamplerPtr->setObjectPointCloud(mPointCloudPtr);
        if (mEnableClustering) {
            mClusterExtractionPtr->setObjectPointCloud(mPointCloudPtr);
        }
        mHypothesisClusterSpaceSampleFilterPtr->setObjectPointCloud(mPointCloudPtr);
        mHypothesisKDTreeSpaceSampleFilterPtr->setObjectPointCloud(mPointCloudPtr);
        mMapSpaceSampleFilterPtr->setObjectPointCloud(mPointCloudPtr);
    }

    void NextBestViewCalculator::loadCropBoxListFromFile(const std::string mCropBoxListFilePath) {
        this->mCropBoxFilterPtr = CropBoxFilterPtr(new CropBoxFilter(mCropBoxListFilePath));
    }

    void NextBestViewCalculator::setEnableCropBoxFiltering(const bool mEnableCropBoxFiltering) {
        this->mEnableCropBoxFiltering = mEnableCropBoxFiltering;
    }

    void NextBestViewCalculator::setEnableIntermediateObjectWeighting(const bool mEnableIntermediateObjectWeighting) {
        this->mEnableIntermediateObjectWeighting = mEnableIntermediateObjectWeighting;
    }

    ObjectPointCloudPtr NextBestViewCalculator::getPointCloudPtr() {
        return mPointCloudPtr;
    }

    void NextBestViewCalculator::setActiveIndices(const IndicesPtr &activeIndicesPtr) {
        mActiveIndicesPtr = activeIndicesPtr;
        mRatingModulePtr->setObjectPointIndices(mActiveIndicesPtr);
        mCameraModelFilterPtr->setObjectPointIndices(mActiveIndicesPtr);
        for (int i : boost::irange(0, mNumberOfThreads)) {
            mThreadRatingModules[i]->setObjectPointIndices(mActiveIndicesPtr);
            mThreadCameraModels[i]->setObjectPointIndices(mActiveIndicesPtr);
        }
        mSpaceSamplerPtr->setObjectPointIndices(mActiveIndicesPtr);
        if (mEnableClustering) {
            mClusterExtractionPtr->setObjectPointIndices(mActiveIndicesPtr);
        }
        mHypothesisClusterSpaceSampleFilterPtr->setObjectPointIndices(mActiveIndicesPtr);
        mHypothesisKDTreeSpaceSampleFilterPtr->setObjectPointIndices(mActiveIndicesPtr);
        mMapSpaceSampleFilterPtr->setObjectPointIndices(mActiveIndicesPtr);
    }

    IndicesPtr NextBestViewCalculator::getActiveIndices() {
        return mActiveIndicesPtr;
    }

    int NextBestViewCalculator::getNumberActiveNormals() {
        int result = 0;

        ObjectPointCloud objectPointCloud = ObjectPointCloud(*getPointCloudPtr(), *getActiveIndices());
        for (ObjectPoint &objPoint : objectPointCloud) {
            result += objPoint.active_normal_vectors->size();
        }

        return result;
    }

    unsigned int NextBestViewCalculator::getNumberTotalNormals(std::string type, std::string identifier) {
        int result = 0;

        ObjectPointCloud objectPointCloud = ObjectPointCloud(*getPointCloudPtr(), *getActiveIndices());
        for (ObjectPoint &objPoint : objectPointCloud) {
            if (objPoint.identifier.compare(identifier) != 0 || objPoint.type.compare(type) != 0)
                continue;
            result += objPoint.normal_vectors->size();
        }

        return result;
    }

    unsigned int NextBestViewCalculator::getNumberActiveNormals(std::string type, std::string identifier) {
        int result = 0;

        ObjectPointCloud objectPointCloud = ObjectPointCloud(*getPointCloudPtr(), *getActiveIndices());
        for (ObjectPoint &objPoint : objectPointCloud) {
            if (objPoint.identifier.compare(identifier) != 0 || objPoint.type.compare(type) != 0)
                continue;
            result += objPoint.active_normal_vectors->size();
        }

        return result;
    }

    std::vector<std::pair<std::string, std::string>> NextBestViewCalculator::getTypeAndIds() {
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

    bool NextBestViewCalculator::removeObjects(std::string type, std::string identifier) {
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
        setActiveIndices(activeIndicesPtr);
        setPointCloudPtr(mPointCloudPtr);
        return true;
    }

    KdTreePtr NextBestViewCalculator::getKdTreePtr() {
        return mKdTreePtr;
    }

    void NextBestViewCalculator::setUnitSphereSampler(const UnitSphereSamplerPtr &unitSphereSamplerPtr) {
        mUnitSphereSamplerPtr = unitSphereSamplerPtr;
    }

    UnitSphereSamplerPtr NextBestViewCalculator::getUnitSphereSampler() {
        return mUnitSphereSamplerPtr;
    }

    void NextBestViewCalculator::setSpaceSampler(const SpaceSamplerPtr &spaceSamplerPtr) {
        mSpaceSamplerPtr = spaceSamplerPtr;
    }

    SpaceSamplerPtr NextBestViewCalculator::getSpaceSampler() {
        return mSpaceSamplerPtr;
    }

    void NextBestViewCalculator::setRobotModel(const RobotModelPtr &robotModelPtr) {
        mRobotModelPtr = robotModelPtr;
    }

    RobotModelPtr NextBestViewCalculator::getRobotModel() {
        return mRobotModelPtr;
    }

    void NextBestViewCalculator::setCameraModelFilter(const CameraModelFilterPtr &cameraModelFilterPtr) {
        mCameraModelFilterPtr = cameraModelFilterPtr;
    }

    CameraModelFilterPtr NextBestViewCalculator::getCameraModelFilter() {
        return mCameraModelFilterPtr;
    }

    void NextBestViewCalculator::setRatingModule(const RatingModulePtr &ratingModulePtr) {
        mRatingModulePtr = ratingModulePtr;
    }

    RatingModulePtr NextBestViewCalculator::getRatingModule() {
        return mRatingModulePtr;
    }

    void NextBestViewCalculator::setMapHelper(const MapHelperPtr &mapHelperPtr) {
        mMapHelperPtr = mapHelperPtr;
        mVisHelperPtr = VisualizationHelperPtr(new VisualizationHelper(mMapHelperPtr));
        NextBestViewCalculatorPtr thisPtr = shared_from_this();
        mNBVPredictionPtr = NextBestViewPredictionPtr(new NextBestViewPrediction(thisPtr, mVisHelperPtr));
    }

    MapHelperPtr NextBestViewCalculator::getMapHelper() {
        return mMapHelperPtr;
    }

    void NextBestViewCalculator::setHypothesisUpdater(const HypothesisUpdaterPtr &hypothesisUpdaterPtr) {
        mHypothesisUpdaterPtr = hypothesisUpdaterPtr;
    }

    HypothesisUpdaterPtr NextBestViewCalculator::getHypothesisUpdater() {
        return mHypothesisUpdaterPtr;
    }

    ClusterExtractionPtr NextBestViewCalculator::getClusterExtractionPtr() const {
        return mClusterExtractionPtr;
    }

    void NextBestViewCalculator::setClusterExtractionPtr(const ClusterExtractionPtr &clusterExtractionPtr) {
        mClusterExtractionPtr = clusterExtractionPtr;
    }

    void NextBestViewCalculator::setEpsilon(float epsilon) {
        mEpsilon = epsilon;
    }

    float NextBestViewCalculator::getEpsilon() {
        return mEpsilon;
    }

    void NextBestViewCalculator::setMaxIterationSteps(int maxIterationSteps)
    {
        mMaxIterationSteps  = maxIterationSteps;
    }

    void NextBestViewCalculator::setMinIterationSteps(int minIterationSteps) {
        mMinIterationSteps = minIterationSteps;
    }

    void NextBestViewCalculator::setRatingModuleAbstractFactoryPtr(const RatingModuleAbstractFactoryPtr &ratingModuleAbstractFactoryPtr) {
        mRatingModuleAbstractFactoryPtr = ratingModuleAbstractFactoryPtr;
        for (int i : boost::irange(0, mNumberOfThreads)) {
            if (mThreadRatingModules[i]) {
                mThreadRatingModules[i].reset();
            }
            mThreadRatingModules[i] = mRatingModuleAbstractFactoryPtr->createRatingModule();
        }
    }

    RatingModuleAbstractFactoryPtr NextBestViewCalculator::getRatingModuleAbstractFactoryPtr() {
        return mRatingModuleAbstractFactoryPtr;
    }

    void NextBestViewCalculator::setCameraModelFilterAbstractFactoryPtr(const CameraModelFilterAbstractFactoryPtr &cameraModelFilterAbstractFactoryPtr) {
        mCameraModelFilterAbstractFactoryPtr = cameraModelFilterAbstractFactoryPtr;
        for (int i : boost::irange(0, mNumberOfThreads)) {
            if (mThreadCameraModels[i]) {
                mThreadCameraModels[i].reset();
            }
            mThreadCameraModels[i] = mCameraModelFilterAbstractFactoryPtr->createCameraModelFilter();
        }
    }

    CameraModelFilterAbstractFactoryPtr NextBestViewCalculator::getCameraModelFilterAbstractFactoryPtr() {
        return mCameraModelFilterAbstractFactoryPtr;
    }

    int NextBestViewCalculator::getNumberOfThreads() const {
        return mNumberOfThreads;
    }

    void NextBestViewCalculator::setNumberOfThreads(int value) {
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
            mThreadRatingModules[i]->setObjectPointCloud(mPointCloudPtr);

            // set CameraModule per thread
            if (mThreadCameraModels[i]) {
                mThreadCameraModels[i].reset();
            }
            mThreadCameraModels[i] = mCameraModelFilterAbstractFactoryPtr->createCameraModelFilter();
            mThreadCameraModels[i]->setObjectPointCloud(mPointCloudPtr);
            mThreadCameraModels[i]->setInputPointCloud(mPointCloudPtr);
        }
    }

    double NextBestViewCalculator::getMinUtility() const {
        return mMinUtility;
    }

    void NextBestViewCalculator::setMinUtility(double minUtility) {
        mMinUtility = minUtility;
    }

    bool NextBestViewCalculator::getRemoveInvalidNormals() const {
        return mRemoveInvalidNormals;
    }

    void NextBestViewCalculator::setRemoveInvalidNormals(bool removeInvalidNormals) {
        mRemoveInvalidNormals = removeInvalidNormals;
    }

    bool NextBestViewCalculator::getCacheResults() const {
        return mCacheResults;
    }

    void NextBestViewCalculator::setCacheResults(bool cacheResults) {
        mCacheResults = cacheResults;
    }

    bool NextBestViewCalculator::getRequireMinUtility() const {
        return mRequireMinUtility;
    }

    void NextBestViewCalculator::setRequireMinUtility(bool requireMinUtility) {
        mRequireMinUtility = requireMinUtility;
    }

    bool NextBestViewCalculator::getEnableClustering() const {
        return mEnableClustering;
    }

    bool NextBestViewCalculator::getEnablePrediction() const {
        return mEnablePrediction;
    }

    void NextBestViewCalculator::setEnablePrediction(bool enablePrediction) {
        mEnablePrediction = enablePrediction;
    }

    void NextBestViewCalculator::setEnableClustering(bool enableClustering) {
        mEnableClustering = enableClustering;
    }

    // filters
    bool NextBestViewCalculator::getEnableClusterFilter() const {
        return mEnableClusterFilter;
    }

    void NextBestViewCalculator::setEnableClusterFilter(bool enableClusterFilter) {
        mEnableClusterFilter = enableClusterFilter;
        if (mEnableClusterFilter) {
            mHypothesisClusterSpaceSampleFilterPtr->enable();
        } else {
            mHypothesisClusterSpaceSampleFilterPtr->disable();
        }
    }

    bool NextBestViewCalculator::getEnableMapFilter() const {
        return mEnableMapFilter;
    }

    void NextBestViewCalculator::setEnableMapFilter(bool enableMapFilter) {
        mEnableMapFilter = enableMapFilter;
        if (mEnableMapFilter) {
            mMapSpaceSampleFilterPtr->enable();
        } else {
            mMapSpaceSampleFilterPtr->disable();
        }
    }

    bool NextBestViewCalculator::getEnableKDTreeFilter() const {
        return mEnableKDTreeFilter;
    }

    void NextBestViewCalculator::setEnableKDTreeFilter(bool enableKDTreeFilter) {
        mEnableKDTreeFilter = enableKDTreeFilter;
        if (mEnableKDTreeFilter) {
            mHypothesisKDTreeSpaceSampleFilterPtr->enable();
        } else {
            mHypothesisKDTreeSpaceSampleFilterPtr->disable();
        }
    }

    GeneralFilterPtr<SamplePoint> NextBestViewCalculator::getMapSpaceSampleFilterPtr() const {
        return mMapSpaceSampleFilterPtr;
    }

    void NextBestViewCalculator::setMapSpaceSampleFilterPtr(const GeneralFilterPtr<SamplePoint> &mapSpaceSampleFilterPtr) {
        mMapSpaceSampleFilterPtr = mapSpaceSampleFilterPtr;
    }

    GeneralFilterPtr<SamplePoint> NextBestViewCalculator::getHypothesisKDTreeSpaceSampleFilterPtr() const {
        return mHypothesisKDTreeSpaceSampleFilterPtr;
    }

    void NextBestViewCalculator::setHypothesisKDTreeSpaceSampleFilterPtr(const GeneralFilterPtr<SamplePoint> &hypothesisKDTreeSpaceSampleFilterPtr) {
        mHypothesisKDTreeSpaceSampleFilterPtr = hypothesisKDTreeSpaceSampleFilterPtr;
    }

    GeneralFilterPtr<SamplePoint> NextBestViewCalculator::getHypothesisClusterSpaceSampleFilterPtr() const {
        return mHypothesisClusterSpaceSampleFilterPtr;
    }

    void NextBestViewCalculator::setHypothesisClusterSpaceSampleFilterPtr(const GeneralFilterPtr<SamplePoint> &hypothesisClusterSpaceSampleFilterPtr) {
        mHypothesisClusterSpaceSampleFilterPtr = hypothesisClusterSpaceSampleFilterPtr;
    }

    GeneralFilterPtr<SamplePoint> NextBestViewCalculator::getSpaceSamlpingFilterChainPtr() const {
        return mSpaceSamplingFilterChainPtr;
    }

    void NextBestViewCalculator::setSpaceSamplingFilterChainPtr() {
        mSpaceSamplingFilterChainPtr = mHypothesisClusterSpaceSampleFilterPtr;
        mSpaceSamplingFilterChainPtr->setPostFilter(mMapSpaceSampleFilterPtr);
        mMapSpaceSampleFilterPtr->setPostFilter(mHypothesisKDTreeSpaceSampleFilterPtr);
    }

    int NextBestViewCalculator::getMinIterationGA() const {
        return mMinIterationGA;
    }

    void NextBestViewCalculator::setMinIterationGA(int minIterationGA) {
        mMinIterationGA = minIterationGA;
    }

    ViewMutationPtr NextBestViewCalculator::getViewMutationPtr() const {
        return mViewMutationPtr;
    }

    void NextBestViewCalculator::setViewMutationPtr(const ViewMutationPtr &viewMutationPtr) {
        mViewMutationPtr = viewMutationPtr;
    }

    bool NextBestViewCalculator::getEnableGA() const {
        return mEnableGA;
    }

    void NextBestViewCalculator::setEnableGA(bool enableGA) {
        mEnableGA = enableGA;
    }
}
