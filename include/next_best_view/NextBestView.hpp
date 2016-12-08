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

// Global Includes
#include <algorithm>
#include <boost/foreach.hpp>
#include <eigen3/Eigen/Dense>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>

// ROS Main Include
#include <ros/ros.h>

// ROS Includes
#include <object_database/ObjectManager.h>
#include <dynamic_reconfigure/server.h>
#include <next_best_view/DynamicParametersConfig.h>

// Local Includes
#include "typedef.hpp"
#include "next_best_view/RatedViewport.h"
#include "next_best_view/RateViewports.h"
#include "next_best_view/RemoveObjects.h"
#include "next_best_view/NormalsInfo.h"
#include "next_best_view/TriggerFrustumVisualization.h"
#include "next_best_view/TriggerFrustumsAndPointCloudVisualization.h"
#include "next_best_view/GetAttributedPointCloud.h"
#include "next_best_view/GetNextBestView.h"
#include "next_best_view/SetAttributedPointCloud.h"
#include "next_best_view/SetInitRobotState.h"
#include "next_best_view/ResetCalculator.h"
#include "next_best_view/UpdatePointCloud.h"
#include "next_best_view/GetActiveNormals.h"
#include "next_best_view/helper/MapHelper.hpp"
#include "next_best_view/helper/WorldHelper.hpp"
#include "next_best_view/NextBestViewCalculator.hpp"
#include "next_best_view/helper/VisualizationsHelper.hpp"
#include "pbd_msgs/PbdAttributedPointCloud.h"
#include "pbd_msgs/PbdAttributedPoint.h"
#include "pbd_msgs/PbdViewport.h"
#include "next_best_view/camera_model_filter/impl/Raytracing3DBasedSingleCameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/impl/SingleCameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/impl/Raytracing3DBasedStereoCameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/impl/StereoCameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/impl/Raytracing3DBasedSingleCameraModelFilterFactory.hpp"
#include "next_best_view/camera_model_filter/impl/SingleCameraModelFilterFactory.hpp"
#include "next_best_view/camera_model_filter/impl/Raytracing3DBasedStereoCameraModelFilterFactory.hpp"
#include "next_best_view/camera_model_filter/impl/StereoCameraModelFilterFactory.hpp"
#include "next_best_view/helper/DebugHelper.hpp"
#include "next_best_view/helper/VisualizationsHelper.hpp"
#include "next_best_view/hypothesis_updater/impl/PerspectiveHypothesisUpdater.hpp"
#include "next_best_view/hypothesis_updater/impl/DefaultHypothesisUpdater.hpp"
#include "next_best_view/hypothesis_updater/impl/PerspectiveHypothesisUpdaterFactory.hpp"
#include "next_best_view/hypothesis_updater/impl/DefaultHypothesisUpdaterFactory.hpp"
#include "next_best_view/robot_model/impl/MILDRobotModelWithExactIK.hpp"
#include "next_best_view/robot_model/impl/MILDRobotModelWithApproximatedIK.hpp"
#include "next_best_view/robot_model/impl/MILDRobotModelWithExactIKFactory.hpp"
#include "next_best_view/robot_model/impl/MILDRobotModelWithApproximatedIKFactory.hpp"
#include "next_best_view/robot_model/impl/MILDRobotState.hpp"
#include "next_best_view/unit_sphere_sampler/impl/SpiralApproxUnitSphereSampler.hpp"
#include "next_best_view/unit_sphere_sampler/impl/SprialApproxUnitSphereSamplerFactory.hpp"
#include "next_best_view/space_sampler/impl/PlaneSubSpaceSampler.hpp"
#include "next_best_view/space_sampler/impl/Raytracing2DBasedSpaceSampler.hpp"
#include "next_best_view/space_sampler/impl/MapBasedHexagonSpaceSampler.hpp"
#include "next_best_view/space_sampler/impl/MapBasedRandomSpaceSampler.hpp"
#include "next_best_view/space_sampler/impl/HypothesisSpaceSampler.hpp"
#include "next_best_view/space_sampler/impl/PlaneSubSpaceSamplerFactory.hpp"
#include "next_best_view/space_sampler/impl/Raytracing2DBasedSpaceSamplerFactory.hpp"
#include "next_best_view/space_sampler/impl/MapBasedHexagonSpaceSamplerFactory.hpp"
#include "next_best_view/space_sampler/impl/MapBasedRandomSpaceSamplerFactory.hpp"
#include "next_best_view/space_sampler/impl/HypothesisSpaceSamplerFactory.hpp"
#include "next_best_view/space_sampler/impl/HexagonSpaceSamplePattern.hpp"
#include "next_best_view/filter/sample_point/HypothesisClusterSpaceSampleFilter.hpp"
#include "next_best_view/filter/sample_point/HypothesisKDTreeSpaceSampleFilter.hpp"
#include "next_best_view/filter/sample_point/MapSpaceSampleFilter.hpp"
#include "next_best_view/rating/impl/DefaultRatingModule.hpp"
#include "next_best_view/rating/impl/DefaultRatingModuleFactory.hpp"
#include "next_best_view/cluster/impl/EuclideanPCLClusterExtraction.hpp"

namespace next_best_view {
// Defining namespace shorthandles
namespace viz = visualization_msgs;
namespace odb = object_database;

/*!
     * \brief NextBestView is a configuration class of the related node.
     *
     * NextBestView provides services and publishers for the next_best_view Node and also
     * saves data structures which are needed for node-wide purposes.
     */
class NextBestView {
private:
    // The Calculator Instance
    NextBestViewCalculatorPtr mCalculatorPtr;

    // Node Handles
    ros::NodeHandle mGlobalNodeHandle;
    ros::NodeHandle mNodeHandle;

    // ServiceServer and Publisher
    ros::ServiceServer mGetPointCloudServiceServer;
    ros::ServiceServer mSetPointCloudServiceServer;
    ros::ServiceServer mSetInitRobotStateServiceServer;
    ros::ServiceServer mGetNextBestViewServiceServer;
    ros::ServiceServer mUpdatePointCloudServiceServer;
    ros::ServiceServer mTriggerFrustumVisualizationServer;
    ros::ServiceServer mTriggerOldFrustumVisualizationServer;
    ros::ServiceServer mTriggerFrustmsAndPointCloudVisualizationServer;
    ros::ServiceServer mResetCalculatorServer;
    ros::ServiceServer mRateViewportsServer;
    ros::ServiceServer mRemoveObjectsServer;


    // Etcetera
    ViewportPoint mCurrentCameraViewport;
    DebugHelperPtr mDebugHelperPtr;
    VisualizationHelperPtr mVisHelperPtr;
    /*!
     * visualization settings
     */
    bool mShowSpaceSampling, mShowPointcloud, mShowFrustumPointCloud, mShowFrustumMarkerArray, mShowNormals;
    bool mCurrentlyPublishingVisualization;

    // dynconfig
    dynamic_reconfigure::Server<DynamicParametersConfig> mDynamicReconfigServer;
    DynamicParametersConfig mConfig;
    uint32_t mConfigLevel;
    bool mFirstRun;
    enum ConfigLevelType {
        parameterConfig            = 0b1 << 0,
        sphereSamplingConfig       = 0b1 << 1,
        mapHelperConfig            = 0b1 << 2,
        spaceSamplingConfig        = (0b1 << 3) | mapHelperConfig, // these 3 might require mapHelper (depending on chosen moduleId),
        cameraModelConfig          = (0b1 << 4) | mapHelperConfig, // so they should be updated if mapHelper params are changed
        robotModelConfig           = (0b1 << 5) | mapHelperConfig,
        ratingConfig               = (0b1 << 6) | robotModelConfig | cameraModelConfig, // rating requires cameraModel and robotModel
        hypothesisUpdaterConfig    = (0b1 << 7) | ratingConfig, // requires ratingModule
        cropboxFileConfig          = 0b1 << 8,
		worldHelperConfig          = mapHelperConfig
    };

    boost::shared_ptr<boost::thread> mVisualizationThread;

    // module factory classes
    UnitSphereSamplerAbstractFactoryPtr mSphereSamplerFactoryPtr;
    MapHelperPtr mMapHelperPtr;
    SpaceSamplerAbstractFactoryPtr mSpaceSampleFactoryPtr;
    CameraModelFilterAbstractFactoryPtr mCameraModelFactoryPtr;
    RobotModelAbstractFactoryPtr mRobotModelFactoryPtr;
    RatingModuleAbstractFactoryPtr mRatingModuleFactoryPtr;
    HypothesisUpdaterAbstractFactoryPtr mHypothesisUpdaterFactoryPtr;
    ClusterExtractionPtr mClusterExtractionPtr;
    GeneticAlgorithmPtr mGeneticAlgorithmPtr;

public:
    /*!
         * \brief Creates an instance of the NextBestView class.
         */
    NextBestView()
    {
        mDebugHelperPtr = DebugHelper::getInstance();

        mGlobalNodeHandle = ros::NodeHandle();
        mNodeHandle = ros::NodeHandle(ros::this_node::getName());

        mCalculatorPtr = NextBestViewCalculatorPtr(new NextBestViewCalculator());

        mFirstRun = true;
        dynamic_reconfigure::Server<next_best_view::DynamicParametersConfig>::CallbackType f = boost::bind(&NextBestView::dynamicReconfigureCallback, this, _1, _2);
        mDynamicReconfigServer.setCallback(f);

        // TODO is this necessary or does dynReconfig always call callback once at the beginning?
        // DynamicParametersConfig config;
        // mDynamicReconfigServer.getConfigDefault(config);
        // dynamicReconfigureCallback(config, UINT_MAX);
    }

    //dtor
    virtual ~NextBestView() {
        this->mVisualizationThread->join();
    }

    void initialize()
    {
        mDebugHelperPtr->writeNoticeably("STARTING NBV PARAMETER OUTPUT", DebugHelper::PARAMETERS);

        mDebugHelperPtr->write(std::stringstream() << "debugLevels: " << mDebugHelperPtr->getLevelString(), DebugHelper::PARAMETERS);

        mCurrentlyPublishingVisualization = false;

        /* These are the parameters for the CameraModelFilter. By now they will be kept in here, but for the future they'd better
         * be defined in the CameraModelFilter specialization class.
         * TODO: Export these parameters to the specialization of the CameraModelFilter class. This makes sense, because you have to
         * keep in mind that there are stereo cameras which might have slightly different settings of the frustums. So we will be
         * able to adjust the parameters for each camera separateley.
         */
        mDebugHelperPtr->write(std::stringstream() << "fovx: " << mConfig.fovx, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "fovy: " << mConfig.fovy, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "ncp: " << mConfig.ncp, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "fcp: " << mConfig.fcp, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "radius: " << mConfig.radius, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "colThresh: " << mConfig.colThresh, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "samples: " << mConfig.sampleSizeUnitSphereSampler, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "speedFactorRecognizer: " << mConfig.speedFactorRecognizer, DebugHelper::PARAMETERS);

        //////////////////////////////////////////////////////////////////
        // HERE STARTS THE CONFIGURATION OF THE NEXTBESTVIEW CALCULATOR //
        //////////////////////////////////////////////////////////////////
        /* The NextBestViewCalculator consists of multiple modules which interact w/ each other. By this
         * interaction we get a result for the next best view which suites our constraints best.
         *
         * The modules namely are:
         * - UnitSphereSampler (unit_sphere_sampler/UnitSphereSampler.hpp)
         * - SpaceSampler (space_sampler/SpaceSampler.hpp)
         * - CameraModelFilter (camera_model_filter/CameraModelFilter.hpp)
         * - RobotModel (robot_model/RobotModel.hpp)
         * - RatingModule (rating_module/RatingModule.hpp)
         * - HypothesisUpdater (hypothesis_updater/HypothesisUpdater.hpp)
         *
         * Every of these modules is an abstract class which provides an interface to interact with.
         * These interfaces are, right now, far from final and can therefore be change as you want to change them.
         * Side-effects are expected just in the next_best_view node.
         */

        /* SpiralApproxUnitSphereSampler is a specialization of the abstract UnitSphereSampler class.
         * It picks a given number of samples out of the unit sphere. These samples should be uniform
         * distributed on the sphere's surface which is - by the way - no easy problem to solve.
         * Therefore we used an approximation algorithm which approximates the surface with a
         * projection of a spiral on the sphere's surface. Resulting in this wonderful sounding name.
         * - sphereSamplerId == 1 => SpiralApproxUnitSphereSampler
         */
        if (mConfigLevel & sphereSamplingConfig) {
            if (mSphereSamplerFactoryPtr) {
                mSphereSamplerFactoryPtr.reset();
            }
            mSphereSamplerFactoryPtr = createSphereSamplerFactoryFromConfig(mConfig.sphereSamplerId);
            mCalculatorPtr->setUnitSphereSampler(mSphereSamplerFactoryPtr->createUnitSphereSampler());
        }

        /* MapHelper does get the maps on which we are working on and modifies them for use with applications like raytracing and others.
         * TODO: The maps may have areas which are marked feasible but in fact are not, because of different reasons. The main
         * reason may be that the map may contain areas where the robot cannot fit through and therefore cannot move to the
         * wanted position (islands/holes). You have to consider if there is any possibility to mark these areas as non-feasible.
         */
        if (mConfigLevel & mapHelperConfig) {
            mMapHelperPtr = MapHelperPtr(new MapHelper());
            mMapHelperPtr->setCollisionThreshold(mConfig.colThresh);
            mCalculatorPtr->setMapHelper(mMapHelperPtr);

            mVisHelperPtr = VisualizationHelperPtr(new VisualizationHelper(mCalculatorPtr->getMapHelper()));
        }

        /* Cluster Extraction/filter
         */
        if (mFirstRun) {
            mClusterExtractionPtr = EuclideanPCLClusterExtractionPtr(new EuclideanPCLClusterExtraction());
            mCalculatorPtr->setClusterExtractionPtr(mClusterExtractionPtr);
            mCalculatorPtr->setHypothesisKDTreeSpaceSampleFilterPtr(boost::make_shared<HypothesisKDTreeSpaceSampleFilter>(mConfig.fcp));
            mCalculatorPtr->setHypothesisClusterSpaceSampleFilterPtr(boost::make_shared<HypothesisClusterSpaceSampleFilter>(mCalculatorPtr->getClusterExtractionPtr(), mConfig.fcp));
            mCalculatorPtr->setMapSpaceSampleFilterPtr(boost::make_shared<MapSpaceSampleFilter>(mMapHelperPtr));
            mCalculatorPtr->setSpaceSamplingFilterChainPtr();
        }


        /* Intializes spaceSampler with a SpaceSampler subclass specified by the parameter samplerId :
         * - samplerId == 1 => MapBasedHexagonSpaceSampler
         * - samplerId == 2 => MapBasedRandomSpaceSampler
         * - samplerId == 3 => PlaneSubSpaceSampler
         * - samplerId == 4 => Raytracing2DBasedSpaceSampler
         */
        mDebugHelperPtr->write(std::stringstream() << "sampleSizeMapBasedRandomSpaceSampler: " << mConfig.sampleSizeMapBasedRandomSpaceSampler, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "spaceSamplerId: " << mConfig.spaceSamplerId, DebugHelper::PARAMETERS);

        if (mConfigLevel & spaceSamplingConfig) {
            if (mSpaceSampleFactoryPtr) {
                mSpaceSampleFactoryPtr.reset();
            }
            mSpaceSampleFactoryPtr = createSpaceSamplerFactoryFromConfig(mConfig.spaceSamplerId);
            mCalculatorPtr->setSpaceSampler(mSpaceSampleFactoryPtr->createSpaceSampler());
        }

        mDebugHelperPtr->write(std::stringstream() << "cameraFilterId: " << mConfig.cameraFilterId, DebugHelper::PARAMETERS);

        /*
         * Intializes cameraModelFilter with a CameraModelFilter subclass specified by the parameter cameraFilterId :
         * - cameraFilterId == 1 => Raytracing3DBasedStereoCameraModelFilter
         * - cameraFilterId == 2 => StereoCameraModelFilter
         * - cameraFilterId == 3 => Raytracing3DBasedSingleCameraModelFilter
         * - cameraFilterId == 4 => SingleCameraModelFilter
         * Note that the first 2 ids are stereo based filters, while the last 2 are based on a single camera.
         * Furthermore even ids don't use ray tracing, while odd ids use ray tracing.
         */
        if (mConfigLevel & cameraModelConfig) {
            if (mCameraModelFactoryPtr) {
                mCameraModelFactoryPtr.reset();
            }
            mCameraModelFactoryPtr = createCameraModelFactoryFromConfig(mConfig.cameraFilterId);
            mCalculatorPtr->setCameraModelFilter(mCameraModelFactoryPtr->createCameraModelFilter());
            mCalculatorPtr->setCameraModelFilterAbstractFactoryPtr(mCameraModelFactoryPtr);
        }

        /* MILDRobotModel is a specialization of the abstract RobotModel class.
         * The robot model maps takes the limitations of the used robot into account and by this it is possible to filter out
         * non-reachable configurations of the robot which can therefore be ignored during calculation.
         * - robotModelId == 1 => MILDRobotModelWithExactIK
         * - robotModelId == 2 => MILDRobotModelWithApproximatedIK
         */
        mDebugHelperPtr->write(std::stringstream() << "panMin: " << mConfig.panMin, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "panMax: " << mConfig.panMax, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "tiltMin: " << mConfig.tiltMin, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "tiltMax: " << mConfig.tiltMax, DebugHelper::PARAMETERS);

        // TODO: some robot model params are not taken from dyn reconfigure
        if (mConfigLevel & robotModelConfig) {
            if (mRobotModelFactoryPtr) {
                mRobotModelFactoryPtr.reset();
            }
            mRobotModelFactoryPtr = createRobotModelFactoryFromConfig(mConfig.robotModelId);
            mCalculatorPtr->setRobotModel(mRobotModelFactoryPtr->createRobotModel());
        }

        /* DefaultRatingModule is a specialization of the abstract RatingModule class.
         * The rating module calculates the use and the costs of an operation.
         * - ratingModuleId == 1 => DefaultRatingModule
         */
        if (mConfigLevel & ratingConfig) {
            if (mRatingModuleFactoryPtr) {
                mRatingModuleFactoryPtr.reset();
            }
            mRatingModuleFactoryPtr = createRatingModuleFactoryFromConfig(mConfig.ratingModuleId);
            mCalculatorPtr->setRatingModule(mRatingModuleFactoryPtr->createRatingModule());
            mCalculatorPtr->setRatingModuleAbstractFactoryPtr(mRatingModuleFactoryPtr);
            NextBestViewCache::setRatingModulePtr(mRatingModuleFactoryPtr->createRatingModule());
        }

        /* PerspectiveHypothesisUpdater is a specialization of the abstract HypothesisUpdater.
         * - hypothesisUpdaterId == 1 => PerspectiveHypothesisUpdater
         * - hypothesisUpdaterId == 2 => DefaultHypothesisUpdater
         */
        if (mConfigLevel & hypothesisUpdaterConfig) {
            if (mHypothesisUpdaterFactoryPtr) {
                mHypothesisUpdaterFactoryPtr.reset();
            }
            mHypothesisUpdaterFactoryPtr = createHypothesisUpdaterFactoryFromConfig(mConfig.hypothesisUpdaterId);
            mCalculatorPtr->setHypothesisUpdater(mHypothesisUpdaterFactoryPtr->createHypothesisUpdater());
        }

        // The settings of the modules.
        if (mConfigLevel & cropboxFileConfig) {
            mCalculatorPtr->loadCropBoxListFromFile(mConfig.mCropBoxListFilePath);
        }
        if (mConfigLevel & parameterConfig) {
            mDebugHelperPtr->setLevels(mConfig.debugLevels);

            //Set the max/min amout of iterations
            mDebugHelperPtr->write(std::stringstream() << "maxIterationSteps: " << mConfig.maxIterationSteps, DebugHelper::PARAMETERS);
            mCalculatorPtr->setMaxIterationSteps(mConfig.maxIterationSteps);
            mDebugHelperPtr->write(std::stringstream() << "minIterationSteps: " << mConfig.minIterationSteps, DebugHelper::PARAMETERS);
            mCalculatorPtr->setMinIterationSteps(mConfig.minIterationSteps);

            mCalculatorPtr->setNumberOfThreads(mConfig.nRatingThreads);
            mCalculatorPtr->setEpsilon(mConfig.epsilon);

            // optional features
            mCalculatorPtr->setEnableIntermediateObjectWeighting(mConfig.enableIntermediateObjectWeighting);
            mCalculatorPtr->setEnableCropBoxFiltering(mConfig.enableCropBoxFiltering);
            mCalculatorPtr->setRemoveInvalidNormals(mConfig.removeInvalidNormals);
            mCalculatorPtr->setCacheResults(mConfig.cacheResults);
            mCalculatorPtr->setEnablePrediction(mConfig.enablePrediction);
            mCalculatorPtr->setEnableGA(mConfig.enableGA);
            mCalculatorPtr->setEnableClustering(mConfig.enableClustering);

            // filter
            mCalculatorPtr->setEnableClusterFilter(mConfig.enableClusterFilter);
            mCalculatorPtr->setEnableKDTreeFilter(mConfig.enableKDTreeFilter);
            mCalculatorPtr->setEnableMapFilter(mConfig.enableMapFilter);

            // ga
            mGeneticAlgorithmPtr = boost::make_shared<GeneticAlgorithm>(mCalculatorPtr, mMapHelperPtr, mClusterExtractionPtr, mConfig.improvementIterations, mConfig.improvementAngle, mConfig.radius, mConfig.minIterationGA);
            mCalculatorPtr->setMinIterationGA(mConfig.minIterationGA);
            mCalculatorPtr->setGeneticAlgorithmPtr(mGeneticAlgorithmPtr);

            // min utility
            float minUtility;
            mNodeHandle.getParam("/scene_exploration_sm/min_utility_for_moving", minUtility);
            mCalculatorPtr->setMinUtility(minUtility);

            mShowSpaceSampling = mConfig.show_space_sampling;
            mShowPointcloud = mConfig.show_point_cloud;
            mShowFrustumPointCloud = mConfig.show_frustum_point_cloud;
            mShowFrustumMarkerArray = mConfig.show_frustum_marker_array;
            mShowNormals = mConfig.show_normals;
        }

        mDebugHelperPtr->write(std::stringstream() << "boolClearBetweenIterations: " << mVisHelperPtr->getBoolClearBetweenIterations(), DebugHelper::PARAMETERS);

        mDebugHelperPtr->writeNoticeably("ENDING NBV PARAMETER OUTPUT", DebugHelper::PARAMETERS);

        if (mFirstRun) {
            // at our first run we have to advertise services.
            mFirstRun = false;
            mGetPointCloudServiceServer = mNodeHandle.advertiseService("get_point_cloud", &NextBestView::processGetPointCloudServiceCall, this);
            mGetNextBestViewServiceServer = mNodeHandle.advertiseService("next_best_view", &NextBestView::processGetNextBestViewServiceCall, this);
            mSetPointCloudServiceServer = mNodeHandle.advertiseService("set_point_cloud", &NextBestView::processSetPointCloudServiceCall, this);
            mSetInitRobotStateServiceServer = mNodeHandle.advertiseService("set_init_robot_state", &NextBestView::processSetInitRobotStateServiceCall, this);
            mUpdatePointCloudServiceServer = mNodeHandle.advertiseService("update_point_cloud", &NextBestView::processUpdatePointCloudServiceCall, this);
            mTriggerFrustumVisualizationServer = mNodeHandle.advertiseService("trigger_frustum_visualization", &NextBestView::processTriggerFrustumVisualization, this);
            mTriggerOldFrustumVisualizationServer = mNodeHandle.advertiseService("trigger_old_frustum_visualization", &NextBestView::processTriggerOldFrustumVisualization, this);
            mTriggerFrustmsAndPointCloudVisualizationServer = mNodeHandle.advertiseService("trigger_frustums_and_point_cloud_visualization", &NextBestView::processTriggerFrustumsAndPointCloudVisualization, this);
            mResetCalculatorServer = mNodeHandle.advertiseService("reset_nbv_calculator", &NextBestView::processResetCalculatorServiceCall, this);
            mRateViewportsServer = mNodeHandle.advertiseService("rate_viewports", &NextBestView::processRateViewports, this);
            mRemoveObjectsServer = mNodeHandle.advertiseService("remove_objects", &NextBestView::processRemoveObjects, this);
        }
    }

	WorldHelperPtr initializeWorldHelper() {
        mDebugHelperPtr->write(std::stringstream() << "worldFilePath: " << mConfig.worldFilePath, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "voxelSize: " << mConfig.voxelSize, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "worldHeight: " << mConfig.worldHeight, DebugHelper::PARAMETERS);

        MapHelperPtr mapHelperPtr = mCalculatorPtr->getMapHelper();

        return WorldHelperPtr(new WorldHelper(mapHelperPtr, mConfig.worldFilePath, mConfig.voxelSize, mConfig.worldHeight, mConfig.visualizeRaytracing));
    }

    UnitSphereSamplerAbstractFactoryPtr createSphereSamplerFactoryFromConfig(int moduleId) {
        switch (moduleId) {
        case 1:
            return UnitSphereSamplerAbstractFactoryPtr(new SpiralApproxUnitSphereSamplerFactory(mConfig.sampleSizeUnitSphereSampler));
        default:
            std::stringstream ss;
            ss << mConfig.sphereSamplerId << " is not a valid sphere sampler ID";
            ROS_ERROR_STREAM(ss.str());
            throw std::runtime_error(ss.str());
        }
    }

    SpaceSamplerAbstractFactoryPtr createSpaceSamplerFactoryFromConfig(int moduleId) {
        switch (moduleId)
        {
        case 1:
            /* MapBasedHexagonSpaceSampler is a specialization of the abstract SpaceSampler class.
             * By space we denote the area in which the robot is moving. In our case there are just two degrees of freedom
             * in which the robot can move, namely the xy-plane. But we do also have a map on which we can base our sampling on.
             * There are a lot of ways to sample the xy-plane into points but we decided to use a hexagonal grid which we lay over
             * the map and calculate the points which are contained in the feasible map space.
             */
            return SpaceSamplerAbstractFactoryPtr(new MapBasedHexagonSpaceSamplerFactory(mMapHelperPtr, mConfig.radius));
        case 2:
            return SpaceSamplerAbstractFactoryPtr(new MapBasedRandomSpaceSamplerFactory(mMapHelperPtr, mConfig.sampleSizeMapBasedRandomSpaceSampler));
        case 3:
            return SpaceSamplerAbstractFactoryPtr(new PlaneSubSpaceSamplerFactory());
        case 4:
            return SpaceSamplerAbstractFactoryPtr(new Raytracing2DBasedSpaceSamplerFactory(mMapHelperPtr));
        case 5:
            return SpaceSamplerAbstractFactoryPtr(new HypothesisSpaceSamplerFactory(mCalculatorPtr->getClusterExtractionPtr(), createSpaceSamplePatternFactoryFromConfig(mConfig.spaceSamplePatternId), mConfig.fcp));
        default:
            std::stringstream ss;
            ss << mConfig.spaceSamplerId << " is not a valid space sampler ID";
            ROS_ERROR_STREAM(ss.str());
            throw std::runtime_error(ss.str());
        }
    }

    SpaceSamplePatternPtr createSpaceSamplePatternFactoryFromConfig(int moduleId) {
        HexagonSpaceSamplePatternPtr hexagonSpaceSamplePatternPtr;
        switch (moduleId)
        {
        case 1:
            hexagonSpaceSamplePatternPtr = HexagonSpaceSamplePatternPtr(new HexagonSpaceSamplePattern());
            hexagonSpaceSamplePatternPtr->setRadius(mConfig.radius);
            return hexagonSpaceSamplePatternPtr;
        default:
            std::stringstream ss;
            ss << mConfig.spaceSamplerId << " is not a valid space sample pattern ID";
            ROS_ERROR_STREAM(ss.str());
            throw std::runtime_error(ss.str());
        }
    }

    CameraModelFilterAbstractFactoryPtr createCameraModelFactoryFromConfig(int moduleId) {
        SimpleVector3 leftCameraPivotPointOffset = SimpleVector3(0.0, -0.067, 0.04);
        SimpleVector3 rightCameraPivotPointOffset = SimpleVector3(0.0, 0.086, 0.04);
        SimpleVector3 oneCameraPivotPointOffset = (leftCameraPivotPointOffset + rightCameraPivotPointOffset) * 0.5;
   	    // data for 3D raytracing
        WorldHelperPtr worldHelperPtr = initializeWorldHelper();

        switch (moduleId) {
        case 1:
            return CameraModelFilterAbstractFactoryPtr(
                        new Raytracing3DBasedStereoCameraModelFilterFactory(worldHelperPtr,
                                                                            leftCameraPivotPointOffset, rightCameraPivotPointOffset,
                                                                            mConfig.fovx, mConfig.fovy,
                                                                            mConfig.fcp, mConfig.ncp,
                                                                            mConfig.speedFactorRecognizer));
        case 2:
            return CameraModelFilterAbstractFactoryPtr(
                        new StereoCameraModelFilterFactory(leftCameraPivotPointOffset, rightCameraPivotPointOffset,
                                                           mConfig.fovx, mConfig.fovy,
                                                           mConfig.fcp, mConfig.ncp,
                                                           mConfig.speedFactorRecognizer));
        case 3:
            /* MapBasedSingleCameraModelFilterPtr is a specialization of the abstract CameraModelFilter class.
             * The camera model filter takes account for the fact, that there are different cameras around in the real world.
             * In respect of the parameters of these cameras the point cloud gets filtered by theses camera filters. Plus
             * the map-based version of the camera filter also uses the knowledge of obstacles or walls between the camera and
             * the object to be observed. So, map-based in this context means that the way from the lens to the object is ray-traced.
             */
            return CameraModelFilterAbstractFactoryPtr(
                        new Raytracing3DBasedSingleCameraModelFilterFactory(worldHelperPtr,
                                                                            oneCameraPivotPointOffset,
                                                                            mConfig.fovx, mConfig.fovy,
                                                                            mConfig.fcp, mConfig.ncp,
                                                                            mConfig.speedFactorRecognizer));
        case 4:
            return CameraModelFilterAbstractFactoryPtr(
                        new SingleCameraModelFilterFactory(oneCameraPivotPointOffset,
                                                           mConfig.fovx, mConfig.fovy,
                                                           mConfig.fcp, mConfig.ncp,
                                                           mConfig.speedFactorRecognizer));
        default:
            std::stringstream ss;
            ss << mConfig.cameraFilterId << " is not a valid camera filter ID";
            ROS_ERROR_STREAM(ss.str());
            throw std::runtime_error(ss.str());
        }
    }

    RobotModelAbstractFactoryPtr createRobotModelFactoryFromConfig(int moduleId) {
        switch (moduleId) {
        case 1:
            mDebugHelperPtr->write("NBV: Using new IK model", DebugHelper::PARAMETERS);
            return RobotModelAbstractFactoryPtr(new MILDRobotModelWithExactIKFactory(mConfig.tiltMin, mConfig.tiltMax, mConfig.panMin, mConfig.panMax, mMapHelperPtr));
        case 2:
            mDebugHelperPtr->write("NBV: Using old IK model", DebugHelper::PARAMETERS);
            return RobotModelAbstractFactoryPtr(new MILDRobotModelWithApproximatedIKFactory(mConfig.tiltMin, mConfig.tiltMax, mConfig.panMin, mConfig.panMax, mMapHelperPtr));
        default:
            std::stringstream ss;
            ss << mConfig.robotModelId << " is not a valid robot model ID";
            ROS_ERROR_STREAM(ss.str());
            throw std::runtime_error(ss.str());
        }
    }

    RatingModuleAbstractFactoryPtr createRatingModuleFactoryFromConfig(int moduleId) {
        RobotModelAbstractFactoryPtr robotModelFactoryPtr;
        CameraModelFilterAbstractFactoryPtr cameraModelFactoryPtr;
        switch (moduleId) {
        case 1:
            robotModelFactoryPtr = createRobotModelFactoryFromConfig(mConfig.robotModelId);
            cameraModelFactoryPtr = createCameraModelFactoryFromConfig(mConfig.cameraFilterId);
            return RatingModuleAbstractFactoryPtr(
                        new DefaultRatingModuleFactory(mConfig.fovx, mConfig.fovy,
                                                       mConfig.fcp, mConfig.ncp,
                                                       robotModelFactoryPtr, cameraModelFactoryPtr, mMapHelperPtr,
                                                       mConfig.mRatingNormalAngleThreshold / 180.0 * M_PI,
                                                       mConfig.mOmegaUtility, mConfig.mOmegaPan, mConfig.mOmegaTilt,
                                                       mConfig.mOmegaRot, mConfig.mOmegaBase, mConfig.mOmegaRecognizer,
                                                       mConfig.useOrientationUtility, mConfig.useProximityUtility, mConfig.useSideUtility));
        default:
            std::stringstream ss;
            ss << mConfig.ratingModuleId << " is not a valid rating module ID";
            ROS_ERROR_STREAM(ss.str());
            throw std::runtime_error(ss.str());
        }
    }

    HypothesisUpdaterAbstractFactoryPtr createHypothesisUpdaterFactoryFromConfig(int moduleId) {
        DefaultRatingModuleFactoryPtr ratingModuleFactoryPtr;
        switch (moduleId) {
        case 1:
            // create a DefaultRatingModule from config
            ratingModuleFactoryPtr = boost::static_pointer_cast<DefaultRatingModuleFactory>(createRatingModuleFactoryFromConfig(1));
            return HypothesisUpdaterAbstractFactoryPtr(new PerspectiveHypothesisUpdaterFactory(ratingModuleFactoryPtr,
                                                                                               mConfig.mHypothesisUpdaterAngleThreshold / 180.0 * M_PI));
        case 2:
            return HypothesisUpdaterAbstractFactoryPtr(new DefaultHypothesisUpdaterFactory());
        default:
            std::stringstream ss;
            ss << mConfig.hypothesisUpdaterId << " is not a valid hypothesis module ID";
            ROS_ERROR_STREAM(ss.str());
            throw std::runtime_error(ss.str());
        }
    }

    bool processResetCalculatorServiceCall(ResetCalculator::Request &request, ResetCalculator::Response &response) {
        mDebugHelperPtr->writeNoticeably("STARTING NBV RESET-CALCULATOR SERVICE CALL", DebugHelper::SERVICE_CALLS);
        mConfigLevel = numeric_limits<uint32_t>::max();
        initialize();

        response.succeeded = true;
        mDebugHelperPtr->writeNoticeably("ENDING NBV RESET-CALCULATOR SERVICE CALL", DebugHelper::SERVICE_CALLS);
        return true;
    }

    bool processRateViewports(RateViewports::Request &request, RateViewports::Response &response) {
        mDebugHelperPtr->writeNoticeably("STARTING NBV RATE-VIEWPORTS SERVICE CALL", DebugHelper::SERVICE_CALLS);

        if (request.viewports.empty()) {
            return true;
        }

        // Current camera view (frame of camera) of the robot.
        ViewportPoint currentCameraViewport(request.current_pose);

        // set robotstate
        mCalculatorPtr->initializeRobotState(currentCameraViewport);

        // convert geometry_msgs::Poses to ViewportPoints
        ViewportPointCloudPtr sampleViewportsPtr = ViewportPointCloudPtr(new ViewportPointCloud());
        unsigned int i = 0;
        for (geometry_msgs::Pose viewport : request.viewports) {
            ViewportPoint sampleViewport(viewport);
            sampleViewport.object_type_set = ObjectTypeSetPtr(new ObjectTypeSet(request.object_type_name_list.begin(), request.object_type_name_list.end()));
            sampleViewport.oldIdx = i;
            sampleViewportsPtr->push_back(sampleViewport);
            i++;
        }

        // find nearby object hypothesis per sampleViewport
        IndicesPtr feasibleViewportsPtr;
        mCalculatorPtr->getFeasibleViewports(sampleViewportsPtr, feasibleViewportsPtr);

        // remove sampleViewports without nearby object hypothesis
        ViewportPointCloudPtr feasibleSampleViewportsPtr = ViewportPointCloudPtr(new ViewportPointCloud(*sampleViewportsPtr, *feasibleViewportsPtr));

        // rate
        ViewportPointCloudPtr ratedSampleViewportsPtr;
        mCalculatorPtr->rateViewports(feasibleSampleViewportsPtr, currentCameraViewport, ratedSampleViewportsPtr, request.use_object_type_to_rate);

        mDebugHelperPtr->write(std::stringstream() << "number of rated viewports: " << ratedSampleViewportsPtr->size(), DebugHelper::SERVICE_CALLS);

        // threads mix up oldIdx
        std::sort(ratedSampleViewportsPtr->begin(), ratedSampleViewportsPtr->end(), [](ViewportPoint a, ViewportPoint b)
        {
            return a.oldIdx < b.oldIdx;
        });

        // convert to response
        unsigned int nextRatedViewportIdx = 0; // to iterate through ratedSampleViewportsPtr
        unsigned int nextRatedViewportOldIdx = 0; // oldIdx attribute of next ratedSampleViewport/nextRatedViewportIdx
        if (ratedSampleViewportsPtr->size() == 0) {
            nextRatedViewportOldIdx = -1;
        } else {
            nextRatedViewportOldIdx = ratedSampleViewportsPtr->at(nextRatedViewportIdx).oldIdx;
        }
        for (i = 0; i < sampleViewportsPtr->size(); i++) {
            RatedViewport responseViewport;
            if (i != nextRatedViewportOldIdx) {
                ViewportPoint unratedViewport = sampleViewportsPtr->at(i);
                responseViewport.pose = unratedViewport.getPose();
                responseViewport.oldIdx = i;
                if (unratedViewport.object_type_set->size() > 0) {
                    responseViewport.object_type_name_list = std::vector<string>(unratedViewport.object_type_set->size());
                    std::copy(unratedViewport.object_type_set->begin(),
                              unratedViewport.object_type_set->end(),
                              responseViewport.object_type_name_list.begin());
                }
            } else {
                // assert(i == nextRatedViewportOldIdx) ^= ratedSampleViewportsPtr->at(nextRatedViewportIdx).oldIdx
                ViewportPoint& ratedViewport = ratedSampleViewportsPtr->at(nextRatedViewportIdx);
                responseViewport.pose = ratedViewport.getPose();
                responseViewport.oldIdx = i;
                if (ratedViewport.object_type_set->size() > 0) {
                    responseViewport.object_type_name_list = std::vector<string>(ratedViewport.object_type_set->size());
                    std::copy(ratedViewport.object_type_set->begin(),
                              ratedViewport.object_type_set->end(),
                              responseViewport.object_type_name_list.begin());
                }
                responseViewport.rating = mCalculatorPtr->getRatingModule()->getRating(ratedViewport.score);
                // set utility and inverse cost terms in rating for the given next best view.
                responseViewport.utility = ratedViewport.score->getUnweightedUnnormalizedUtility();
                responseViewport.inverse_costs = ratedViewport.score->getWeightedInverseCosts();
                responseViewport.base_translation_inverse_costs = ratedViewport.score->getUnweightedInverseMovementCostsBaseTranslation();
                responseViewport.base_rotation_inverse_costs = ratedViewport.score->getUnweightedInverseMovementCostsBaseRotation();
                responseViewport.ptu_movement_inverse_costs = ratedViewport.score->getUnweightedInverseMovementCostsPTU();
                responseViewport.recognition_inverse_costs = ratedViewport.score->getUnweightedInverseRecognitionCosts();
                nextRatedViewportIdx++;
                if (nextRatedViewportIdx < ratedSampleViewportsPtr->size()) {
                    nextRatedViewportOldIdx = ratedSampleViewportsPtr->at(nextRatedViewportIdx).oldIdx;
                }
            }
            response.sortedRatedViewports.push_back(responseViewport);
        }

        // sort
        std::stable_sort(response.sortedRatedViewports.begin(), response.sortedRatedViewports.end(), [](RatedViewport a, RatedViewport b)
        {
            return a.rating > b.rating;
        });

        mDebugHelperPtr->writeNoticeably("ENDING NBV RATE-VIEWPORTS SERVICE CALL", DebugHelper::SERVICE_CALLS);
        return true;
    }

    bool processRemoveObjects(RemoveObjects::Request &request, RemoveObjects::Response &response) {

        mDebugHelperPtr->writeNoticeably("STARTING NBV REMOVE-OBJECTS SERVICE CALL", DebugHelper::SERVICE_CALLS);

        bool is_valid = mCalculatorPtr->removeObjects(request.type, request.identifier);
        // if pointcloud is empty after this call
        response.is_valid = is_valid;

        mDebugHelperPtr->writeNoticeably("ENDING NBV REMOVE-OBJECTS SERVICE CALL", DebugHelper::SERVICE_CALLS);

        return true;
    }

    static void convertObjectPointCloudToAttributedPointCloud(const ObjectPointCloud &pointCloud, pbd_msgs::PbdAttributedPointCloud &pointCloudMessage, uint minNumberNormals) {
        pointCloudMessage.elements.clear();

        BOOST_FOREACH(ObjectPoint point, pointCloud) {

            // skip objects with too few normals
            if (point.active_normal_vectors->size() < minNumberNormals)
//            if (point.active_normal_vectors->size() < point.normal_vectors->size())
                continue;

            pbd_msgs::PbdAttributedPoint aPoint;

            aPoint.pose = point.getPose();
            aPoint.type = point.type;
            aPoint.identifier = point.identifier;

            pointCloudMessage.elements.push_back(aPoint);
        }
    }

    bool processGetPointCloudServiceCall(GetAttributedPointCloud::Request &request, GetAttributedPointCloud::Response &response) {
        mDebugHelperPtr->writeNoticeably("STARTING NBV GET-POINT-CLOUD SERVICE CALL", DebugHelper::SERVICE_CALLS);

        // minNumberNormals defaults to 1 if not set
        if (request.minNumberNormals < 1)
            request.minNumberNormals = 1;
        convertObjectPointCloudToAttributedPointCloud(*mCalculatorPtr->getPointCloudPtr(), response.point_cloud, request.minNumberNormals);

        mDebugHelperPtr->writeNoticeably("ENDING NBV GET-POINT-CLOUD SERVICE CALL", DebugHelper::SERVICE_CALLS);
        return true;
    }

    bool processSetPointCloudServiceCall(SetAttributedPointCloud::Request &request, SetAttributedPointCloud::Response &response) {

        mDebugHelperPtr->writeNoticeably("STARTING NBV SET-POINT-CLOUD SERVICE CALL", DebugHelper::SERVICE_CALLS);

        if (!mCalculatorPtr->setPointCloudFromMessage(request.point_cloud)) {
            ROS_ERROR("Could not set point cloud from message.");
            mDebugHelperPtr->writeNoticeably("ENDING NBV SET-POINT-CLOUD SERVICE CALL", DebugHelper::SERVICE_CALLS);
            return false;
        }

        if(mCalculatorPtr->getPointCloudPtr()->size() == 0)
        {
            response.is_valid = false;
            mDebugHelperPtr->writeNoticeably("ENDING NBV SET-POINT-CLOUD SERVICE CALL", DebugHelper::SERVICE_CALLS);
            return true;
        }

        // convert to viewportPointCloud
        std::vector<ViewportPoint> viewportPointList;
        BOOST_FOREACH(pbd_msgs::PbdViewport &viewport, request.viewports_to_filter)
        {
            ViewportPoint viewportConversionPoint(viewport.pose);
            viewportConversionPoint.object_type_set = boost::shared_ptr<ObjectTypeSet>(new ObjectTypeSet());
            viewportConversionPoint.object_type_set->insert(viewport.object_type_name_list.begin(), viewport.object_type_name_list.end());
            viewportPointList.push_back(viewportConversionPoint);
        }

        //Filter point cloud with those views.
        mCalculatorPtr->updateFromExternalViewportPointList(viewportPointList);

        response.is_valid = true;

        // object ^= object type + identifier
        std::vector<std::pair<std::string, std::string>> typeAndIds = mCalculatorPtr->getTypeAndIds();
        for (auto &typeAndId : typeAndIds) {
            NormalsInfo curNormalsInfo;
            std::string type = typeAndId.first;
            std::string identifier = typeAndId.second;
            unsigned int activeNormals = mCalculatorPtr->getNumberActiveNormals(type, identifier);
            unsigned int totalNormals = mCalculatorPtr->getNumberTotalNormals(type, identifier);
            unsigned int deactivatedNormals = totalNormals - activeNormals;
            curNormalsInfo.active_normals = totalNormals;
            curNormalsInfo.deactivated_object_normals = deactivatedNormals;
            curNormalsInfo.type = type;
            curNormalsInfo.identifier = identifier;
            response.normals_per_object.push_back(curNormalsInfo);
        }

        // publish the visualization
        this->publishNewPointCloudVisualization();
        mDebugHelperPtr->writeNoticeably("ENDING NBV SET-POINT-CLOUD SERVICE CALL", DebugHelper::SERVICE_CALLS);
        return true;

    }

    bool processSetInitRobotStateServiceCall(SetInitRobotState::Request &request, SetInitRobotState::Response &response) {
        mDebugHelperPtr->writeNoticeably("STARTING NBV SET-INIT-ROBOT-STATE SERVICE CALL", DebugHelper::SERVICE_CALLS);

        MILDRobotStatePtr currentRobotStatePtr(new MILDRobotState());
        currentRobotStatePtr->pan = request.robotState.pan;
        currentRobotStatePtr->tilt = request.robotState.tilt;
        currentRobotStatePtr->rotation = request.robotState.rotation;
        currentRobotStatePtr->x = request.robotState.x;
        currentRobotStatePtr->y = request.robotState.y;

        ROS_INFO_STREAM("Initial pose: pan " << currentRobotStatePtr->pan << ", tilt " << currentRobotStatePtr->tilt
                            << ", rotation " << currentRobotStatePtr->rotation << ", x " << currentRobotStatePtr->x << ", y " << currentRobotStatePtr->y);

        mCalculatorPtr->getRobotModel()->setCurrentRobotState(currentRobotStatePtr);

        mDebugHelperPtr->writeNoticeably("ENDING NBV SET-INIT-ROBOT-STATE SERVICE CALL", DebugHelper::SERVICE_CALLS);
        return true;
    }

    //COMMENT?
    bool processGetNextBestViewServiceCall(GetNextBestView::Request &request, GetNextBestView::Response &response) {
        mDebugHelperPtr->writeNoticeably("STARTING NBV GET-NEXT-BEST-VIEW SERVICE CALL", DebugHelper::SERVICE_CALLS);
        // Current camera view (frame of camera) of the robot.
        ViewportPoint currentCameraViewport(request.current_pose);
        //Contains Next best view.

        // we cannot find a nbv if pointcloud is empty
        if (mCalculatorPtr->getPointCloudPtr()->empty()) {
            response.found = false;
            return true;
        }

        ViewportPoint resultingViewport;
        //Estimate the Next best view.
        if (!mCalculatorPtr->calculateNextBestView(currentCameraViewport, resultingViewport)) {
            //No points from input cloud in any nbv candidate or iterative search aborted (by user).
            mDebugHelperPtr->write("No more next best view found.", DebugHelper::SERVICE_CALLS);
            response.found = false;
            mDebugHelperPtr->writeNoticeably("ENDING NBV GET-NEXT-BEST-VIEW SERVICE CALL", DebugHelper::SERVICE_CALLS);
            return true;
        }
        //Return the optimization result including its parameters.
        response.found = true;
        response.resulting_pose = resultingViewport.getPose();

        // copying the objects to be searched for into a list
        response.object_type_name_list = ObjectTypeList(resultingViewport.object_type_set->size());
        std::copy(resultingViewport.object_type_set->begin(), resultingViewport.object_type_set->end(), response.object_type_name_list.begin());

        //Reconstruct robot configuration for next best camera viewport (once known when estimating its costs) for outpout purposes.
        // TODO: This solution is very dirty because we get the specialization of RobotState and this will break if we change the RobotModel and RobotState type.
        RobotStatePtr state = mCalculatorPtr->getRobotModel()->calculateRobotState(resultingViewport.getPosition(), resultingViewport.getSimpleQuaternion());
        MILDRobotStatePtr mildState = boost::static_pointer_cast<MILDRobotState>(state);

        RobotStateMessage robotStateMsg;
        robotStateMsg.pan = mildState->pan;
        robotStateMsg.tilt = mildState->tilt;
        robotStateMsg.rotation = mildState->rotation;
        robotStateMsg.x = mildState->x;
        robotStateMsg.y = mildState->y;

        // set it to the response
        response.robot_state = robotStateMsg;

        // set utility and inverse cost terms in rating for the given next best view.
        RatingModulePtr ratingModule = mRatingModuleFactoryPtr->createRatingModule();
        std::stringstream scoreSS;
        scoreSS << (*resultingViewport.score);
        response.score_container = scoreSS.str();
        response.rating = ratingModule->getRating(resultingViewport.score);
        response.utility = resultingViewport.score->getWeightedNormalizedUtility();
        response.unnormalized_utility = resultingViewport.score->getUnweightedUnnormalizedUtility();
        response.utility_normalization = resultingViewport.score->getUtilityNormalization();
        response.inverse_costs = resultingViewport.score->getWeightedInverseCosts();
        response.base_translation_inverse_costs = resultingViewport.score->getUnweightedInverseMovementCostsBaseTranslation();
        response.base_rotation_inverse_costs = resultingViewport.score->getUnweightedInverseMovementCostsBaseRotation();
        response.ptu_movement_inverse_costs = resultingViewport.score->getUnweightedInverseMovementCostsPTU();
        response.recognition_inverse_costs = resultingViewport.score->getUnweightedInverseRecognitionCosts();

        mCurrentCameraViewport = resultingViewport;

        mDebugHelperPtr->writeNoticeably("ENDING NBV GET-NEXT-BEST-VIEW SERVICE CALL", DebugHelper::SERVICE_CALLS);
        return true;
    }

    bool processUpdatePointCloudServiceCall(UpdatePointCloud::Request &request, UpdatePointCloud::Response &response) {
        mDebugHelperPtr->writeNoticeably("STARTING NBV UPDATE-POINT-CLOUD SERVICE CALL", DebugHelper::SERVICE_CALLS);

        // output
        std::stringstream pose;
        pose << "x = " << request.pose_for_update.position.x << ", "
             << "y = " << request.pose_for_update.position.y << ", "
             << "z = " << request.pose_for_update.position.z << ", "
             << "qw = " << request.pose_for_update.orientation.w << ", "
             << "qx = " << request.pose_for_update.orientation.x << ", "
             << "qy = " << request.pose_for_update.orientation.y << ", "
             << "qz = " << request.pose_for_update.orientation.z;
        std::string objects = "";
        BOOST_FOREACH(std::string object, request.object_type_name_list) {
            if (objects.length() != 0) {
                objects += ", ";
            }
            objects += object;
        }

        mDebugHelperPtr->write(std::stringstream() << "Updating with pose: " << pose, DebugHelper::SERVICE_CALLS);
        mDebugHelperPtr->write(std::stringstream() << "Updating objects: " << objects, DebugHelper::SERVICE_CALLS);

        // convert data types
        SimpleVector3 point = TypeHelper::getSimpleVector3(request.pose_for_update);
        SimpleQuaternion orientation = TypeHelper::getSimpleQuaternion(request.pose_for_update);
        ViewportPoint viewportPoint;
        mDebugHelperPtr->write(std::stringstream() << "Do frustum culling: ActiveIndices="
                                        << mCalculatorPtr->getActiveIndices()->size(),
                                    DebugHelper::SERVICE_CALLS);
        ViewportPoint updateViewport(point, orientation);
        updateViewport.child_indices = mCalculatorPtr->getActiveIndices();
        if (!mCalculatorPtr->doFrustumCulling(updateViewport)) {
            mDebugHelperPtr->write("no objects in frustum", DebugHelper::SERVICE_CALLS);
            return true;
        }
        mDebugHelperPtr->write("Do update object point cloud", DebugHelper::SERVICE_CALLS);

        // copy objects to be updated from list to set
        ObjectTypeSetPtr objectTypeSetPtr = ObjectTypeSetPtr(new ObjectTypeSet(request.object_type_name_list.begin(), request.object_type_name_list.end()));
        unsigned int deactivatedNormals = mCalculatorPtr->updateObjectPointCloud(objectTypeSetPtr, viewportPoint);

	    response.deactivated_object_normals = deactivatedNormals;

        this->publishPointCloudNormals();

        mDebugHelperPtr->writeNoticeably("ENDING NBV UPDATE-POINT-CLOUD SERVICE CALL", DebugHelper::SERVICE_CALLS);
        return true;
    }

    bool processTriggerFrustumVisualization(TriggerFrustumVisualization::Request &request,
                                            TriggerFrustumVisualization::Response &response)
    {
        mDebugHelperPtr->writeNoticeably("STARTING NBV TRIGGER-FRUSTUM-VISUALIZATION SERVICE CALL", DebugHelper::SERVICE_CALLS);
        geometry_msgs::Pose pose = request.current_pose;
        SimpleVector3 position = TypeHelper::getSimpleVector3(pose);
        SimpleQuaternion orientation = TypeHelper::getSimpleQuaternion(pose);
        mCalculatorPtr->getCameraModelFilter()->setPivotPointPose(position, orientation);

        mVisHelperPtr->triggerNewFrustumVisualization(mCalculatorPtr->getCameraModelFilter());

        mDebugHelperPtr->writeNoticeably("ENDING NBV TRIGGER-FRUSTUM-VISUALIZATION SERVICE CALL", DebugHelper::SERVICE_CALLS);
        return true;
    }

    bool processTriggerOldFrustumVisualization(TriggerFrustumVisualization::Request &request,
                                               TriggerFrustumVisualization::Response &response)
    {
        mDebugHelperPtr->writeNoticeably("STARTING NBV TRIGGER-OLD-FRUSTUM-VISUALIZATION SERVICE CALL", DebugHelper::SERVICE_CALLS);
        geometry_msgs::Pose pose = request.current_pose;
        SimpleVector3 position = TypeHelper::getSimpleVector3(pose);
        SimpleQuaternion orientation = TypeHelper::getSimpleQuaternion(pose);
        mCalculatorPtr->getCameraModelFilter()->setPivotPointPose(position, orientation);

        mVisHelperPtr->triggerOldFrustumVisualization(this->mCalculatorPtr->getCameraModelFilter());

        mDebugHelperPtr->writeNoticeably("ENDING NBV TRIGGER-OLD-FRUSTUM-VISUALIZATION SERVICE CALL", DebugHelper::SERVICE_CALLS);
        return true;
    }

    bool processTriggerFrustumsAndPointCloudVisualization(TriggerFrustumsAndPointCloudVisualization::Request &request,
                                                         TriggerFrustumsAndPointCloudVisualization::Response &response)
    {
        mDebugHelperPtr->writeNoticeably("STARTING NBV TRIGGER-FRUSTUMS-AND-POINT-CLOUD-VISUALIZATION SERVICE CALL", DebugHelper::SERVICE_CALLS);

        // set viewport
        ViewportPoint viewport(request.new_viewport.pose);

        IndicesPtr childIndicesPtr;
        if (!mCalculatorPtr->getFeasibleHypothesis(viewport.getPosition(), childIndicesPtr))
        {
            viewport.child_indices = IndicesPtr(new Indices());
            triggerVisualization(viewport);
            return true;
        }

        viewport.child_indices = childIndicesPtr;

        if (!mCalculatorPtr->doFrustumCulling(viewport))
        {
            triggerVisualization(viewport);
            return true;
        }

        ObjectTypeSetPtr objectTypeSetPtr = ObjectTypeSetPtr(new ObjectTypeSet());

        BOOST_FOREACH(std::string objectType, request.new_viewport.object_type_name_list)
            objectTypeSetPtr->insert(objectType);

        ViewportPoint filteredViewport;
        viewport.filterObjectPointCloudByTypes(objectTypeSetPtr, filteredViewport);

        // visualize viewport
        triggerVisualization(filteredViewport);

        mDebugHelperPtr->writeNoticeably("ENDING NBV TRIGGER-FRUSTUMS-AND-POINT-CLOUD-VISUALIZATION SERVICE CALL", DebugHelper::SERVICE_CALLS);

        return true;
    }

    bool triggerVisualization() {
        return this->triggerVisualization(mCurrentCameraViewport);
    }

    bool triggerVisualization(ViewportPoint viewport) {
        if (mCurrentlyPublishingVisualization) {
            mDebugHelperPtr->write("Already generating visualization data.", DebugHelper::VISUALIZATION);
            return false;
        }

        mCurrentlyPublishingVisualization = true;

        this->mVisualizationThread = boost::shared_ptr<boost::thread>(new boost::thread(&NextBestView::publishVisualization, this, viewport, true));
        return true;
    }

    /*!
     * \brief Publishes the Visualization of the NextBestView
     * \param publishFrustum whether the frustum should be published
     */
    void publishVisualization(ViewportPoint viewport, bool publishFrustum) {
        mDebugHelperPtr->write("Publishing Visualization with viewport", DebugHelper::VISUALIZATION);

        mCalculatorPtr->getCameraModelFilter()->setPivotPointPose(viewport.getPosition(), viewport.getSimpleQuaternion());

        mDebugHelperPtr->write(std::stringstream() << "Frustum Pivot Point : " << mCalculatorPtr->getCameraModelFilter()->getPivotPointPosition()[0]
                                        << " , " <<  mCalculatorPtr->getCameraModelFilter()->getPivotPointPosition()[1]
                                        << " , " << mCalculatorPtr->getCameraModelFilter()->getPivotPointPosition()[2],
                                    DebugHelper::VISUALIZATION);

        mDebugHelperPtr->write(std::stringstream() << "Object points in the viewport: " << viewport.child_indices->size(),
                                    DebugHelper::VISUALIZATION);

        if (mShowPointcloud)
        {
            // publish object point cloud
            Indices pointCloudIndices;
            if (mShowFrustumPointCloud) {
                // if the frustum point cloud is published seperately only publish points outside frustum
                this->getIndicesOutsideFrustum(viewport, pointCloudIndices);
            } else {
                pointCloudIndices = *mCalculatorPtr->getActiveIndices();
            }

            ObjectPointCloud objectPointCloud = ObjectPointCloud(*mCalculatorPtr->getPointCloudPtr(), pointCloudIndices);
            std::map<std::string, std::string> typeToMeshResource = this->getMeshResources(objectPointCloud);

            mVisHelperPtr->triggerObjectPointCloudVisualization(objectPointCloud, typeToMeshResource);
        }
        if (mShowFrustumPointCloud)
        {
            // publish frustum object point cloud
            ObjectPointCloud frustumObjectPointCloud = ObjectPointCloud(*mCalculatorPtr->getPointCloudPtr(), *viewport.child_indices);
            std::map<std::string, std::string> typeToMeshResource = this->getMeshResources(frustumObjectPointCloud);

            mVisHelperPtr->triggerFrustumObjectPointCloudVisualization(frustumObjectPointCloud, typeToMeshResource);
        }
        if (mShowFrustumMarkerArray && publishFrustum)
        {
            // publish furstums visualization
            mVisHelperPtr->triggerFrustumsVisualization(this->mCalculatorPtr->getCameraModelFilter());
        }
        mCurrentlyPublishingVisualization = false;
    }

    void publishNewPointCloudVisualization() {
        mDebugHelperPtr->write("Publishing visualization of new point cloud", DebugHelper::VISUALIZATION);

        ObjectPointCloud objectPointCloud = ObjectPointCloud(*mCalculatorPtr->getPointCloudPtr(), *mCalculatorPtr->getActiveIndices());

        if (mShowPointcloud)
        {
            // clear visualization of old objects in frustum
            mVisHelperPtr->clearFrustumObjectPointCloudVisualization();

            // publish new object point cloud
            std::map<std::string, std::string> typeToMeshResource = this->getMeshResources(objectPointCloud);

            mVisHelperPtr->triggerObjectPointCloudVisualization(objectPointCloud, typeToMeshResource);
        }

        if (mShowNormals) {
            // publish new normals
            mVisHelperPtr->triggerObjectNormalsVisualization(objectPointCloud);
        }
    }

    void publishPointCloudNormals() {
        mDebugHelperPtr->write("Publishing normals", DebugHelper::VISUALIZATION);

        if (mShowNormals) {
            // show normals
            ObjectPointCloud objectPointCloud = ObjectPointCloud(*mCalculatorPtr->getPointCloudPtr(), *mCalculatorPtr->getActiveIndices());
            mVisHelperPtr->triggerObjectNormalsVisualization(objectPointCloud);
        }
    }

    NextBestViewCalculatorPtr& getCalculator() {
        return mCalculatorPtr;
    }

    void getIndicesOutsideFrustum(const ViewportPoint &viewport, Indices &resultIndices) {
        IndicesPtr activeIndices = mCalculatorPtr->getActiveIndices();
        IndicesPtr childIndices = viewport.child_indices;
        std::set_difference(activeIndices->begin(), activeIndices->end(),
                            childIndices->begin(), childIndices->end(),
                            std::back_inserter(resultIndices));
    }

    std::map<std::string, std::string> getMeshResources(ObjectPointCloud objectPointCloud) {
        std::map<std::string, std::string> typeToMeshResource;
        for(ObjectPointCloud::iterator it = objectPointCloud.begin(); it < objectPointCloud.end(); it++) {
            if (typeToMeshResource.count(it->type) == 0) {
                std::string path = mCalculatorPtr->getMeshPathByName(it->type);
                // check if the path is valid
                if (path.compare("-2") != 0) {
                    typeToMeshResource[it->type] = mCalculatorPtr->getMeshPathByName(it->type);
                }
            }
        }
        return typeToMeshResource;
    }

    void dynamicReconfigureCallback(DynamicParametersConfig &config, uint32_t level) {
        // TODO consider that services and this and other stuff is called parallel
        ROS_INFO_STREAM("Parameters updated");
        ROS_INFO_STREAM("level: " << level);
        this->mConfig = config;
        this->mConfigLevel = level;
        initialize();
    }
};
typedef boost::shared_ptr<DynamicParametersConfig> DynamicParametersConfigPtr;
}

