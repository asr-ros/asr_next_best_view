/*
 * NextBestViewCalculator.hpp
 *
 *  Created on: Sep 25, 2014
 *      Author: ralfschleicher
 */

#pragma once

#include "typedef.hpp"
#include <vector>
#include <map>

#include <boost/foreach.hpp>
#include <boost/range/algorithm_ext/iota.hpp>
#include <boost/lexical_cast.hpp>

#include "next_best_view/hypothesis_updater/HypothesisUpdater.hpp"
#include "next_best_view/robot_model/RobotModel.hpp"
#include "next_best_view/crop_box/CropBoxFilter.hpp"
#include "next_best_view/camera_model_filter/CameraModelFilter.hpp"
#include "next_best_view/unit_sphere_sampler/UnitSphereSampler.hpp"
#include "next_best_view/space_sampler/SpaceSampler.hpp"
#include "next_best_view/rating/RatingModule.hpp"
#include "next_best_view/rating/impl/DefaultScoreContainer.hpp"
#include "pbd_msgs/PbdAttributedPointCloud.h"
#include "pbd_msgs/PbdAttributedPoint.h"
#include "next_best_view/helper/DebugHelper.hpp"
#include "next_best_view/helper/MapHelper.hpp"
#include "next_best_view/helper/ObjectHelper.h"
#include "next_best_view/helper/VisualizationsHelper.hpp"

namespace next_best_view {
class NextBestViewCalculator {
private:
    ObjectPointCloudPtr mPointCloudPtr;
    IndicesPtr mActiveIndicesPtr;
    KdTreePtr mKdTreePtr;
    std::map<std::string, std::string> objectsResources;
    UnitSphereSamplerPtr mUnitSphereSamplerPtr;
    SpaceSamplerPtr mSpaceSamplerPtr;
    RobotModelPtr mRobotModelPtr;
    CropBoxFilterPtr mCropBoxFilterPtr;
    CameraModelFilterPtr mCameraModelFilterPtr;
    RatingModulePtr mRatingModulePtr;
    HypothesisUpdaterPtr mHypothesisUpdaterPtr;
    float mEpsilon;
    ObjectTypeSetPtr mObjectTypeSetPtr;
    DebugHelperPtr mDebugHelperPtr;
    VisualizationHelper mVisHelper;
    bool mEnableCropBoxFiltering;
    bool mEnableIntermediateObjectWeighting;

    int mMaxIterationSteps;

public:

    NextBestViewCalculator(const UnitSphereSamplerPtr & unitSphereSamplerPtr = UnitSphereSamplerPtr(),
                           const SpaceSamplerPtr &spaceSamplerPtr = SpaceSamplerPtr(),
                           const RobotModelPtr &robotModelPtr = RobotModelPtr(),
                           const CameraModelFilterPtr &cameraModelFilterPtr = CameraModelFilterPtr(),
                           const RatingModulePtr &ratingModulePtr = RatingModulePtr())
        : objectsResources(),
          mUnitSphereSamplerPtr(unitSphereSamplerPtr),
          mSpaceSamplerPtr(spaceSamplerPtr),
          mRobotModelPtr(robotModelPtr),
          mCameraModelFilterPtr(cameraModelFilterPtr),
          mRatingModulePtr(ratingModulePtr),
          mEpsilon(10E-3),
          mVisHelper() {

        mDebugHelperPtr = DebugHelper::getInstance();
    }

    /**
     * Calculates the next best view. Starting point of iterative calculations for getNextBestView() service call.
     */
    bool calculateNextBestView(const ViewportPoint &currentCameraViewport, ViewportPoint &resultViewport) {
        std::clock_t begin = std::clock();
        mDebugHelperPtr->writeNoticeably("STARTING CALCULATE-NEXT-BEST-VIEW METHOD", DebugHelper::CALCULATION);

        //Calculate robot configuration corresponding to current camera viewport of robot.
        RobotStatePtr currentState = mRobotModelPtr->calculateRobotState(currentCameraViewport.getPosition(), currentCameraViewport.getSimpleQuaternion());
        //Save it.
        mRobotModelPtr->setCurrentRobotState(currentState);

        mDebugHelperPtr->write("Calculate discrete set of view orientations on unit sphere",
                        DebugHelper::CALCULATION);
        //Get discretized set of camera orientations (pan, tilt) that are to be considered at each robot position considered during iterative optimization.
        SimpleQuaternionCollectionPtr sampledOrientationsPtr = mUnitSphereSamplerPtr->getSampledUnitSphere();
        SimpleQuaternionCollectionPtr feasibleOrientationsCollectionPtr(new SimpleQuaternionCollection());

			BOOST_FOREACH(SimpleQuaternion q, *sampledOrientationsPtr) {
				if (mRobotModelPtr->isPoseReachable(SimpleVector3(0, 0, 0), q)) {
					feasibleOrientationsCollectionPtr->push_back(q);
				}
			}
			// create the next best view point cloud
            bool success = this->doIteration(currentCameraViewport, feasibleOrientationsCollectionPtr, resultViewport);

            std::clock_t end = std::clock();
            double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
            ROS_INFO_STREAM("Iteration took " << elapsed_secs << " seconds.");
            mDebugHelperPtr->writeNoticeably("ENDING CALCULATE-NEXT-BEST-VIEW METHOD", DebugHelper::CALCULATION);
            return success;
		}

		void getFeasibleSamplePoints(const SamplePointCloudPtr &sampledSpacePointCloudPtr, IndicesPtr &resultIndicesPtr) {
			resultIndicesPtr = IndicesPtr(new Indices());
			// get max radius by far clipping plane of camera. this marks the limit for the visible object distance.
			double radius = mCameraModelFilterPtr->getFarClippingPlane();
			//Go through all 
			for(std::size_t index = 0; index < sampledSpacePointCloudPtr->size(); index++) {
				// get the point
				SamplePoint &spaceSamplePoint = sampledSpacePointCloudPtr->at(index);
				ObjectPoint comparablePoint(spaceSamplePoint.getSimpleVector3());

            // the resulting child indices will be written in here
            IndicesPtr childIndicesPtr(new Indices());
            // we don't need distances, but distances are written in here
            SquaredDistances dismissDistances;

            // this is a radius search - which reduces the complexity level for frustum culling.
            int k = mKdTreePtr->radiusSearch(comparablePoint, radius, *childIndicesPtr, dismissDistances);

            // if there is no result of neighboured points, no need to add this point.
            if (k == 0) {
                continue;
            }

            // set the indices
            spaceSamplePoint.child_indices = childIndicesPtr;

            // add the index to active indices.
            resultIndicesPtr->push_back(index);
        }
    }

private:

	bool doIteration(const ViewportPoint &currentCameraViewport, const SimpleQuaternionCollectionPtr &sampledOrientationsPtr, ViewportPoint &resultViewport) {
        mDebugHelperPtr->writeNoticeably("STARTING DO-ITERATION METHOD", DebugHelper::CALCULATION);

        int iterationStep = 0;
        //Best viewport at the end of each iteration step and starting point for optimization (grid alignment) for each following step.
        ViewportPoint currentBestViewport = currentCameraViewport;
        
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
            if (currentCameraViewport.getPosition() == intermediateResultViewport.getPosition() ||
                    (intermediateResultViewport.getPosition() - currentBestViewport.getPosition()).lpNorm<2>() <= this->getEpsilon() || iterationStep >= mMaxIterationSteps) {
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
        SamplePointCloudPtr sampledSpacePointCloudPtr = mSpaceSamplerPtr->getSampledSpacePointCloud(currentBestPosition, contractor);

        //Set height of sample points as it is set to zero by space sampler
        this->setHeight(sampledSpacePointCloudPtr, currentBestPosition[2]);

        IndicesPtr feasibleIndicesPtr;
        //Prune space sample points in that iteration step by checking whether there are any surrounding object points (within constant far-clipping plane).
        this->getFeasibleSamplePoints(sampledSpacePointCloudPtr, feasibleIndicesPtr);

        //Skip rating all orientations (further code here) if we can only consider our current best robot position and increase sampling resolution
        if (feasibleIndicesPtr->size() == 1 && this->getEpsilon() < contractor) {
            mDebugHelperPtr->write("No RViz visualization for this iteration step, since no new next-best-view found for that resolution.",
                            DebugHelper::VISUALIZATION);
            bool success = doIterationStep(currentCameraViewport, currentBestViewport, sampledOrientationsPtr, contractor * .5, resultViewport, iterationStep);
            mDebugHelperPtr->writeNoticeably("ENDING DO-ITERATION-STEP METHOD", DebugHelper::CALCULATION);
            return success;
        }

        //Create list of all view ports that are checked during this iteration step.
        ViewportPointCloudPtr nextBestViewports = ViewportPointCloudPtr(new ViewportPointCloud());

        //Go through all interesting space sample points for one iteration step to create candidate viewports.
        BOOST_FOREACH(int activeIndex, *feasibleIndicesPtr) {
            SamplePoint &samplePoint = sampledSpacePointCloudPtr->at(activeIndex);
            SimpleVector3 samplePointCoords = samplePoint.getSimpleVector3();
            IndicesPtr samplePointChildIndices = samplePoint.child_indices;

            //For each space sample point: Go through all interesting orientations.
            mDebugHelperPtr->write("Iterating over all orientations for a given robot position.",
                            DebugHelper::CALCULATION);
            BOOST_FOREACH(SimpleQuaternion orientation, *sampledOrientationsPtr) {
                // get the corresponding viewport
                ViewportPoint fullViewportPoint;
                //Calculate which object points lie within frustum for given viewport.
                if (!this->doFrustumCulling(samplePointCoords, orientation, samplePointChildIndices, fullViewportPoint)) {
                    //Skip viewport if no object point is within frustum.
                    continue;
                }
                //For given viewport(combination of robot position and camera viewing direction)
                // get combination of objects (all present in frustum) to search for and the corresponding score for viewport, given that combination.
                mDebugHelperPtr->write("Getting viewport with optimal object constellation for given position & orientation combination.",
                            DebugHelper::RATING);
                if (!mRatingModulePtr->setBestScoreContainer(currentCameraViewport, fullViewportPoint)) {
                    continue;
                }
                //Keep viewport with optimal subset of objects within frustum to search for.
                nextBestViewports->push_back(fullViewportPoint);
            }
        }
        mDebugHelperPtr->write("Sorted list of all viewports (each best for pos & orient combi) in this iteration step.",
                    DebugHelper::RATING);
        if (!mRatingModulePtr->getBestViewport(nextBestViewports, resultViewport)) {
            mDebugHelperPtr->writeNoticeably("ENDING DO-ITERATION-STEP METHOD", DebugHelper::CALCULATION);
            return false;
        }

        //Visualize iteration step and its result.
        mVisHelper.triggerIterationVisualization(iterationStep, sampledOrientationsPtr, resultViewport,
                                                    feasibleIndicesPtr, sampledSpacePointCloudPtr, mSpaceSamplerPtr);

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
         * \return
         */
    bool doFrustumCulling(const SimpleVector3 &position, const SimpleQuaternion &orientation, const IndicesPtr &indices, ViewportPoint &viewportPoint) {
        mCameraModelFilterPtr->setIndices(indices);
        mCameraModelFilterPtr->setPivotPointPose(position, orientation);

        // do the frustum culling
        IndicesPtr frustumIndicesPtr;
        //Call wrapper (with next-best-view data structures) for PCL frustum culling call.
        mCameraModelFilterPtr->filter(frustumIndicesPtr);

        if (frustumIndicesPtr->size() == 0) {
            return false;
        }

        viewportPoint = ViewportPoint(position, orientation);
        viewportPoint.child_indices = frustumIndicesPtr;
        viewportPoint.child_point_cloud = mCameraModelFilterPtr->getInputCloud();
        viewportPoint.point_cloud = mPointCloudPtr;
        viewportPoint.object_type_set = mObjectTypeSetPtr;

        return true;
    }

    void updateFromExternalObjectPointList(const std::vector<ViewportPoint> &viewportPointList) {
        BOOST_FOREACH(ViewportPoint viewportPoint, viewportPointList) {
            ViewportPoint culledViewportPoint;
            if (!this->doFrustumCulling(viewportPoint.getPosition(), viewportPoint.getSimpleQuaternion(), this->getActiveIndices(), culledViewportPoint)) {
                mDebugHelperPtr->write(std::stringstream() << "Viewpoint SKIPPED by Culling: " << viewportPoint.getPosition(),
                                DebugHelper::CALCULATION);
                continue;
            }

            ViewportPoint resultingViewportPoint;
            if (!culledViewportPoint.filterObjectTypes(viewportPoint.object_type_set, resultingViewportPoint)) {
                mDebugHelperPtr->write(std::stringstream() << "Viewpoint SKIPPED by NameFiltering: " << viewportPoint.getPosition(),
                                DebugHelper::CALCULATION);
                continue;
            }

            mDebugHelperPtr->write(std::stringstream() << "Viewpoint TAKEN: " << resultingViewportPoint.getPosition(),
                                DebugHelper::CALCULATION);
            for (std::set<std::string>::iterator it=resultingViewportPoint.object_type_set->begin(); it!=resultingViewportPoint.object_type_set->end(); ++it)
            {
                mDebugHelperPtr->write(std::stringstream() << "Object: " << *it, DebugHelper::CALCULATION);
            }
            this->updateObjectPointCloud(mObjectTypeSetPtr, resultingViewportPoint);
            break;
        }
    }

    /*!
         * \brief Updates the point cloud under the assumption that the given viewport was chosen.
         * \param objectTypeSetPtr the object type names that shall be updated.
         * \param viewportPoint the viewport that was chosen
         * \return the number of deactivated normals
         */
    unsigned int updateObjectPointCloud(const ObjectTypeSetPtr &objectTypeSetPtr, const ViewportPoint &viewportPoint) {
        return mHypothesisUpdaterPtr->update(objectTypeSetPtr, viewportPoint);
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
    bool setPointCloudFromMessage(const pbd_msgs::PbdAttributedPointCloud &msg) {
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

            // add type name to list if not already inserted
            if (mObjectTypeSetPtr->find(element.type) == mObjectTypeSetPtr->end())
                mObjectTypeSetPtr->insert(element.type);

            // Get the rotation matrix to translate the normal vectors of the object.
            SimpleMatrix3 rotationMatrix = pointCloudPoint.getSimpleQuaternion().toRotationMatrix();

            // get object type information
            ObjectMetaDataResponsePtr responsePtr_ObjectData = objectHelper.getObjectMetaData(pointCloudPoint.type);

            if (responsePtr_ObjectData) {
                // translating from std::vector<geometry_msgs::Point> to std::vector<SimpleVector3>
                int normalVectorCount = 0;
                // in cropbox filtering we don't use the normals of the point clouds. Instead we use the ones defined in the xml
                if (!mEnableCropBoxFiltering) {
                    for (geometry_msgs::Point point : responsePtr_ObjectData->normal_vectors) {
                        SimpleVector3 normal(point.x, point.y, point.z);
                        normal = rotationMatrix * normal;
                        pointCloudPoint.active_normal_vectors->push_back(normalVectorCount);
                        pointCloudPoint.normal_vectors->push_back(normal);
                        ++normalVectorCount;
                    }
                }
            } else {
                ROS_ERROR("Invalid object name '%s' in point cloud or object_database node not started. Point Cloud not set!", pointCloudPoint.type.c_str());
                return false;
            }

            //Insert the meshpath
            objectsResources[pointCloudPoint.type] = responsePtr_ObjectData->object_mesh_resource;

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

            mVisHelper.triggerCropBoxVisualization(mCropBoxFilterPtr->getCropBoxWrapperPtrList());

            outputPointCloudPtr = ObjectPointCloudPtr(new ObjectPointCloud(*originalPointCloudPtr, *filteredObjectIndices));

            // we have to set now the object hypothesis normals
            for (ObjectPoint& pointCloudPoint : *outputPointCloudPtr) {
                auto cropBoxPtrList = mCropBoxFilterPtr->getCropBoxWrapperPtrList();
                SimpleMatrix3 rotationMatrix = pointCloudPoint.getSimpleQuaternion().toRotationMatrix();
                int normalVectorCount = 0;
                for (CropBoxWrapperPtr cropBoxWrapper : *cropBoxPtrList) {
                    CropBoxPtr cropBoxPtr = cropBoxWrapper->getCropBox();
                    Eigen::Vector4f max = cropBoxPtr->getMax();
                    Eigen::Vector3f translation = cropBoxPtr->getTranslation();
                    // check if pointCloudPoint is in the cropbox
                    if (isPointInCropbox(pointCloudPoint.getPosition(), translation, max)) {
                        for (SimpleVector3 normal : *cropBoxWrapper->getCropBoxNormalsList()) {
                            pointCloudPoint.active_normal_vectors->push_back(normalVectorCount);
                            SimpleVector3 rotatedNormal = rotationMatrix * normal;
                            pointCloudPoint.normal_vectors->push_back(rotatedNormal);
                            ++normalVectorCount;
                        }
                    }
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

    bool isPointInCropbox(SimpleVector3 position, Eigen::Vector3f translation, Eigen::Vector4f max) {
        return isPointInRange(position(0,0), translation(0,0), max(0,0)) &&
                isPointInRange(position(1,0), translation(1,0), max(1,0)) &&
                isPointInRange(position(2,0), translation(2,0), max(2,0));
    }

    bool isPointInRange(float point, float min, float lenght) {
        return point >= min && point <= (min + lenght);
    }

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
    void setRobotModel(const RobotModelPtr &robotModelPtr) {
        mRobotModelPtr = robotModelPtr;
    }

    /**
         *@return the robot ptr
         */
    RobotModelPtr getRobotModel() {
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
};
}
