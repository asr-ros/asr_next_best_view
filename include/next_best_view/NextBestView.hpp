/**

Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Braun Kai, Heller Florian, Hutmacher Robin, Karrenbauer Oliver, Marek Felix, Mayr Matthias, Mehlhaus Jonas, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Stroh Daniel, Trautmann Jeremias, Walter Milena
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
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <boost/range/algorithm_ext/iota.hpp>
#include <eigen3/Eigen/Dense>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <set>
#include <vector>

// ROS Main Include
#include <ros/ros.h>

// ROS Includes
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <object_database/ObjectManager.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <set>
#include <std_srvs/Empty.h>
#include <world_model/GetViewportList.h>
#include <world_model/PushViewport.h>
#include <std_msgs/ColorRGBA.h>
#include <dynamic_reconfigure/server.h>
#include <next_best_view/DynamicParametersConfig.h>

// Local Includes
#include "typedef.hpp"
#include "next_best_view/TriggerFrustumVisualization.h"
#include "next_best_view/GetAttributedPointCloud.h"
#include "next_best_view/GetNextBestView.h"
#include "next_best_view/SetAttributedPointCloud.h"
#include "next_best_view/SetInitRobotState.h"
#include "next_best_view/ResetCalculator.h"
#include "next_best_view/UpdatePointCloud.h"
#include "next_best_view/helper/MapHelper.hpp"
#include "next_best_view/helper/MapHelperFactory.hpp"
#include "next_best_view/NextBestViewCalculator.hpp"
#include "next_best_view/helper/VisualizationsHelper.hpp"
#include "pbd_msgs/PbdAttributedPointCloud.h"
#include "pbd_msgs/PbdAttributedPoint.h"
#include "next_best_view/camera_model_filter/impl/Raytracing2DBasedSingleCameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/impl/SingleCameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/impl/Raytracing2DBasedStereoCameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/impl/StereoCameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/impl/Raytracing2DBasedSingleCameraModelFilterFactory.hpp"
#include "next_best_view/camera_model_filter/impl/SingleCameraModelFilterFactory.hpp"
#include "next_best_view/camera_model_filter/impl/Raytracing2DBasedStereoCameraModelFilterFactory.hpp"
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
#include "next_best_view/space_sampler/impl/PlaneSubSpaceSamplerFactory.hpp"
#include "next_best_view/space_sampler/impl/Raytracing2DBasedSpaceSamplerFactory.hpp"
#include "next_best_view/space_sampler/impl/MapBasedHexagonSpaceSamplerFactory.hpp"
#include "next_best_view/space_sampler/impl/MapBasedRandomSpaceSamplerFactory.hpp"
#include "next_best_view/rating/impl/DefaultRatingModule.hpp"
#include "next_best_view/rating/impl/DefaultRatingModuleFactory.hpp"

namespace next_best_view {
// Defining namespace shorthandles
namespace viz = visualization_msgs;
namespace odb = object_database;

// Defining shorthandle for action client.
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseActionClient;
typedef boost::shared_ptr<MoveBaseActionClient> MoveBaseActionClientPtr;

/*!
     * \brief NextBestView is a configuration class of the related node.
     *
     * NextBestView provides services and publishers for the next_best_view Node and also
     * saves data structures which are needed for node-wide purposes.
     */
class NextBestView {
private:
    // The Calculator Instance
    NextBestViewCalculator mCalculator;

    // Node Handles
    ros::NodeHandle mGlobalNodeHandle;
    ros::NodeHandle mNodeHandle;

    // ServiceServer and Publisher
    ros::ServiceServer mGetPointCloud2ServiceServer;
    ros::ServiceServer mGetPointCloudServiceServer;
    ros::ServiceServer mSetPointCloudServiceServer;
    ros::ServiceServer mSetInitRobotStateServiceServer;
    ros::ServiceServer mGetNextBestViewServiceServer;
    ros::ServiceServer mUpdatePointCloudServiceServer;
    ros::ServiceServer mTriggerFrustumVisualizationServer;
    ros::ServiceServer mTriggerOldFrustumVisualizationServer;
    ros::ServiceServer mResetCalculatorServer;

    // Action Clients
    MoveBaseActionClientPtr mMoveBaseActionClient;

    // ServiceClients and Subscriber
    ros::ServiceClient mPushViewportServiceClient;
    ros::ServiceClient mGetViewportListServiceClient;

    // Etcetera
    ViewportPoint mCurrentCameraViewport;
    DebugHelperPtr mDebugHelperPtr;
    VisualizationHelper mVisHelper;
    /*!
     * visualization settings
     */
    bool mShowSpaceSampling, mShowPointcloud, mShowFrustumPointCloud, mShowFrustumMarkerArray;
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
    };

    boost::shared_ptr<boost::thread> mVisualizationThread;

    // module factory classes
    UnitSphereSamplerAbstractFactoryPtr mSphereSamplerFactoryPtr;
    MapHelperFactoryPtr mMapHelperFactoryPtr;
    SpaceSamplerAbstractFactoryPtr mSpaceSampleFactoryPtr;
    CameraModelFilterAbstractFactoryPtr mCameraModelFactoryPtr;
    RobotModelAbstractFactoryPtr mRobotModelFactoryPtr;
    RatingModuleAbstractFactoryPtr mRatingModuleFactoryPtr;
    HypothesisUpdaterAbstractFactoryPtr mHypothesisUpdaterFactoryPtr;

public:
    /*!
         * \brief Creates an instance of the NextBestView class.
         */
    NextBestView()
    {
        mDebugHelperPtr = DebugHelper::getInstance();

        mGlobalNodeHandle = ros::NodeHandle();
        mNodeHandle = ros::NodeHandle(ros::this_node::getName());

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
            mSphereSamplerFactoryPtr = createSphereSamplerFromConfig(mConfig.sphereSamplerId);
            mCalculator.setUnitSphereSampler(mSphereSamplerFactoryPtr->createUnitSphereSampler());
        }

        /* MapHelper does get the maps on which we are working on and modifies them for use with applications like raytracing and others.
         * TODO: The maps may have areas which are marked feasible but in fact are not, because of different reasons. The main
         * reason may be that the map may contain areas where the robot cannot fit through and therefore cannot move to the
         * wanted position. You have to consider if there is any possibility to mark these areas as non-feasible.
         */
        if (mConfigLevel & mapHelperConfig) {
            if (mMapHelperFactoryPtr) {
                mMapHelperFactoryPtr.reset();
            }
            mMapHelperFactoryPtr = MapHelperFactoryPtr(new MapHelperFactory(mConfig.colThresh));
            mCalculator.setMapHelper(mMapHelperFactoryPtr->createMapHelper());
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
            mSpaceSampleFactoryPtr = createSpaceSamplerFromConfig(mConfig.spaceSamplerId);
            mCalculator.setSpaceSampler(mSpaceSampleFactoryPtr->createSpaceSampler());
        }

        /*
         * Intializes cameraModelFilter with a CameraModelFilter subclass specified by the parameter cameraFilterId :
         * - cameraFilterId == 1 => MapBasedStereoCameraModelFilter
         * - cameraFilterId == 2 => StereoCameraModelFilter
         * - cameraFilterId == 3 => MapBasedSingleCameraModelFilter
         * - cameraFilterId == 4 => SingleCameraModelFilter
         * Note that the first 2 ids are stereo based filters, while the last 2 are based on a single camera.
         * Furthermore even ids don't use ray tracing, while odd ids use ray tracing.
         */
        if (mConfigLevel & cameraModelConfig) {
            if (mCameraModelFactoryPtr) {
                mCameraModelFactoryPtr.reset();
            }
            mCameraModelFactoryPtr = createCameraModelFromConfig(mConfig.cameraFilterId);
            mCalculator.setCameraModelFilter(mCameraModelFactoryPtr->createCameraModelFilter());
            mCalculator.setCameraModelFilterAbstractFactoryPtr(mCameraModelFactoryPtr);
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

        // TODO: some robot model params are not taken from config
        if (mConfigLevel & robotModelConfig) {
            if (mRobotModelFactoryPtr) {
                mRobotModelFactoryPtr.reset();
            }
            mRobotModelFactoryPtr = createRobotModelFromConfig(mConfig.robotModelId);
            mCalculator.setRobotModel(mRobotModelFactoryPtr->createRobotModel());
        }

        /* DefaultRatingModule is a specialization of the abstract RatingModule class.
         * The rating module calculates the use and the costs of an operation.
         * - ratingModuleId == 1 => DefaultRatingModule
         */
        if (mConfigLevel & ratingConfig) {
            if (mRatingModuleFactoryPtr) {
                mRatingModuleFactoryPtr.reset();
            }
            mRatingModuleFactoryPtr = createRatingModuleFromConfig(mConfig.ratingModuleId);
            mCalculator.setRatingModule(mRatingModuleFactoryPtr->createRatingModule());
            mCalculator.setRatingModuleAbstractFactoryPtr(mRatingModuleFactoryPtr);
        }

        // TODO: defaulthypothesisupdater is missing
        /* PerspectiveHypothesisUpdater is a specialization of the abstract HypothesisUpdater.
         * - hypothesisUpdaterId == 1 => PerspectiveHypothesisUpdater
         */
        if (mConfigLevel & hypothesisUpdaterConfig) {
            if (mHypothesisUpdaterFactoryPtr) {
                mHypothesisUpdaterFactoryPtr.reset();
            }
            mHypothesisUpdaterFactoryPtr = createHypothesisUpdaterFromConfig(mConfig.hypothesisUpdaterId);
            mCalculator.setHypothesisUpdater(mHypothesisUpdaterFactoryPtr->createHypothesisUpdater());
        }

        // The settings of the modules.
        if (mConfigLevel & cropboxFileConfig) {
            mCalculator.loadCropBoxListFromFile(mConfig.mCropBoxListFilePath);
        }
        if (mConfigLevel & parameterConfig) {
            mCalculator.setEnableCropBoxFiltering(mConfig.enableCropBoxFiltering);
            mCalculator.setEnableIntermediateObjectWeighting(mConfig.enableIntermediateObjectWeighting);
            //Set the max amout of iterations
            mDebugHelperPtr->write(std::stringstream() << "maxIterationSteps: " << mConfig.maxIterationSteps, DebugHelper::PARAMETERS);
            mCalculator.setMaxIterationSteps(mConfig.maxIterationSteps);
            mShowSpaceSampling = mConfig.show_space_sampling;
            mShowPointcloud = mConfig.show_point_cloud;
            mShowFrustumPointCloud = mConfig.show_frustum_point_cloud;
            mShowFrustumMarkerArray = mConfig.show_frustum_marker_array;
        }

        mDebugHelperPtr->write(std::stringstream() << "boolClearBetweenIterations: " << mVisHelper.getBoolClearBetweenIterations(), DebugHelper::PARAMETERS);

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
            mResetCalculatorServer = mNodeHandle.advertiseService("reset_nbv_calculator", &NextBestView::processResetCalculatorServiceCall, this);

            mPushViewportServiceClient = mGlobalNodeHandle.serviceClient<world_model::PushViewport>("/env/world_model/push_viewport");
            mGetViewportListServiceClient = mGlobalNodeHandle.serviceClient<world_model::GetViewportList>("/env/world_model/get_viewport_list");
        }
    }

    UnitSphereSamplerAbstractFactoryPtr createSphereSamplerFromConfig(int moduleId) {
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

    MapHelperFactoryPtr createMapHelperFromConfig() {
        return MapHelperFactoryPtr(new MapHelperFactory(mConfig.colThresh));
    }

    SpaceSamplerAbstractFactoryPtr createSpaceSamplerFromConfig(int moduleId) {
        MapHelperFactoryPtr mapHelperFactoryPtr = createMapHelperFromConfig();
        switch (moduleId)
        {
        case 1:
            /* MapBasedHexagonSpaceSampler is a specialization of the abstract SpaceSampler class.
             * By space we denote the area in which the robot is moving. In our case there are just two degrees of freedom
             * in which the robot can move, namely the xy-plane. But we do also have a map on which we can base our sampling on.
             * There are a lot of ways to sample the xy-plane into points but we decided to use a hexagonal grid which we lay over
             * the map and calculate the points which are contained in the feasible map space.
             */
            return SpaceSamplerAbstractFactoryPtr(new MapBasedHexagonSpaceSamplerFactory(mapHelperFactoryPtr, mConfig.radius));
        case 2:
            return SpaceSamplerAbstractFactoryPtr(new MapBasedRandomSpaceSamplerFactory(mapHelperFactoryPtr, mConfig.sampleSizeMapBasedRandomSpaceSampler));
        case 3:
            return SpaceSamplerAbstractFactoryPtr(new PlaneSubSpaceSamplerFactory());
        case 4:
            return SpaceSamplerAbstractFactoryPtr(new Raytracing2DBasedSpaceSamplerFactory(mapHelperFactoryPtr));
        default:
            std::stringstream ss;
            ss << mConfig.spaceSamplerId << " is not a valid space sampler ID";
            ROS_ERROR_STREAM(ss.str());
            throw std::runtime_error(ss.str());
        }
    }

    CameraModelFilterAbstractFactoryPtr createCameraModelFromConfig(int moduleId) {
        SimpleVector3 leftCameraPivotPointOffset = SimpleVector3(0.0, -0.067, 0.04);
        SimpleVector3 rightCameraPivotPointOffset = SimpleVector3(0.0, 0.086, 0.04);
        SimpleVector3 oneCameraPivotPointOffset = (leftCameraPivotPointOffset + rightCameraPivotPointOffset) * 0.5;
        MapHelperFactoryPtr mapHelperFactoryPtr = createMapHelperFromConfig();
        switch (moduleId) {
        case 1:
            return CameraModelFilterAbstractFactoryPtr(
                        new Raytracing2DBasedStereoCameraModelFilterFactory(mapHelperFactoryPtr,
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
                        new Raytracing2DBasedSingleCameraModelFilterFactory(mapHelperFactoryPtr,
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

    RobotModelAbstractFactoryPtr createRobotModelFromConfig(int moduleId) {
        switch (moduleId) {
        case 1:
            mDebugHelperPtr->write("NBV: Using new IK model", DebugHelper::PARAMETERS);
            return RobotModelAbstractFactoryPtr(new MILDRobotModelWithExactIKFactory(mConfig.tiltMin, mConfig.tiltMax, mConfig.panMin, mConfig.panMax));
        case 2:
            mDebugHelperPtr->write("NBV: Using old IK model", DebugHelper::PARAMETERS);
            return RobotModelAbstractFactoryPtr(new MILDRobotModelWithApproximatedIKFactory(mConfig.tiltMin, mConfig.tiltMax, mConfig.panMin, mConfig.panMax));
        default:
            std::stringstream ss;
            ss << mConfig.robotModelId << " is not a valid robot model ID";
            ROS_ERROR_STREAM(ss.str());
            throw std::runtime_error(ss.str());
        }
    }

    RatingModuleAbstractFactoryPtr createRatingModuleFromConfig(int moduleId) {
        RobotModelAbstractFactoryPtr robotModelFactoryPtr;
        CameraModelFilterAbstractFactoryPtr cameraModelFactoryPtr;
        switch (moduleId) {
        case 1:
            robotModelFactoryPtr = createRobotModelFromConfig(mConfig.robotModelId);
            cameraModelFactoryPtr = createCameraModelFromConfig(mConfig.cameraFilterId);
            return RatingModuleAbstractFactoryPtr(
                        new DefaultRatingModuleFactory(mConfig.fovx, mConfig.fovy,
                                                       mConfig.fcp, mConfig.ncp,
                                                       robotModelFactoryPtr, cameraModelFactoryPtr,
                                                       45 / 180.0 * M_PI,
                                                       mConfig.mOmegaUtility, mConfig.mOmegaPan, mConfig.mOmegaTilt,
                                                       mConfig.mOmegaRot, mConfig.mOmegaBase, mConfig.mOmegaRecognizer));
        default:
            std::stringstream ss;
            ss << mConfig.ratingModuleId << " is not a valid rating module ID";
            ROS_ERROR_STREAM(ss.str());
            throw std::runtime_error(ss.str());
        }
    }

    HypothesisUpdaterAbstractFactoryPtr createHypothesisUpdaterFromConfig(int moduleId) {
        DefaultRatingModuleFactoryPtr ratingModuleFactoryPtr;
        switch (moduleId) {
        case 1:
            // create a DefaultRatingModule from config
            ratingModuleFactoryPtr = boost::static_pointer_cast<DefaultRatingModuleFactory>(createRatingModuleFromConfig(1));
            return HypothesisUpdaterAbstractFactoryPtr(new PerspectiveHypothesisUpdaterFactory(ratingModuleFactoryPtr));
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

    static void convertObjectPointCloudToAttributedPointCloud(const ObjectPointCloud &pointCloud, pbd_msgs::PbdAttributedPointCloud &pointCloudMessage) {
        pointCloudMessage.elements.clear();

        BOOST_FOREACH(ObjectPoint point, pointCloud) {
            pbd_msgs::PbdAttributedPoint aPoint;

            aPoint.pose = point.getPose();
            aPoint.type = point.type;

            pointCloudMessage.elements.push_back(aPoint);
        }
    }

    bool processGetPointCloudServiceCall(GetAttributedPointCloud::Request &request, GetAttributedPointCloud::Response &response) {
        mDebugHelperPtr->writeNoticeably("STARTING NBV GET-POINT-CLOUD SERVICE CALL", DebugHelper::SERVICE_CALLS);
        convertObjectPointCloudToAttributedPointCloud(*mCalculator.getPointCloudPtr(), response.point_cloud);

        mDebugHelperPtr->writeNoticeably("ENDING NBV GET-POINT-CLOUD SERVICE CALL", DebugHelper::SERVICE_CALLS);
        return true;
    }

    bool processSetPointCloudServiceCall(SetAttributedPointCloud::Request &request, SetAttributedPointCloud::Response &response) {

        mDebugHelperPtr->writeNoticeably("STARTING NBV SET-POINT-CLOUD SERVICE CALL", DebugHelper::SERVICE_CALLS);

        if (!mCalculator.setPointCloudFromMessage(request.point_cloud)) {
            ROS_ERROR("Could not set point cloud from message.");
            mDebugHelperPtr->writeNoticeably("ENDING NBV SET-POINT-CLOUD SERVICE CALL", DebugHelper::SERVICE_CALLS);
            return false;
        }

        if(mCalculator.getPointCloudPtr()->size() == 0)
        {
            response.is_empty = true;
            response.is_valid = false;
            mDebugHelperPtr->writeNoticeably("ENDING NBV SET-POINT-CLOUD SERVICE CALL", DebugHelper::SERVICE_CALLS);
            return true;
        }

        // Let's get the viewports and update the point cloud.
        world_model::GetViewportList getViewportListServiceCall;
        mGetViewportListServiceClient.call(getViewportListServiceCall);

        // convert to viewportPointCloud
        std::vector<ViewportPoint> viewportPointList(getViewportListServiceCall.response.viewport_list.elements.size());
        BOOST_FOREACH(pbd_msgs::PbdAttributedPoint &point, getViewportListServiceCall.response.viewport_list.elements)
        {
            ViewportPoint viewportConversionPoint(point.pose);
            viewportConversionPoint.object_type_set = boost::shared_ptr<ObjectTypeSet>(new ObjectTypeSet());
            viewportConversionPoint.object_type_set->insert(point.type);
            viewportPointList.push_back(viewportConversionPoint);
        }

        mCalculator.updateFromExternalObjectPointList(viewportPointList);

        response.is_valid = true;
        response.is_empty = false;

        // publish the visualization
        this->publishPointCloudVisualization();
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

        mCalculator.getRobotModel()->setCurrentRobotState(currentRobotStatePtr);

        mDebugHelperPtr->writeNoticeably("ENDING NBV SET-INIT-ROBOT-STATE SERVICE CALL", DebugHelper::SERVICE_CALLS);
        return true;
    }

    //COMMENT?
    bool processGetNextBestViewServiceCall(GetNextBestView::Request &request, GetNextBestView::Response &response) {
        mDebugHelperPtr->writeNoticeably("STARTING NBV GET-NEXT-BEST-VIEW SERVICE CALL", DebugHelper::SERVICE_CALLS);
        // Current camera view (frame of camera) of the robot.
        ViewportPoint currentCameraViewport(request.current_pose);
        //Contains Next best view.

        ViewportPoint resultingViewport;
        //Estimate the Next best view.
        if (!mCalculator.calculateNextBestView(currentCameraViewport, resultingViewport)) {
            //No points from input cloud in any nbv candidate or iterative search aborted (by user).
            mDebugHelperPtr->write("No more next best view found.", DebugHelper::SERVICE_CALLS);
            if (mShowFrustumMarkerArray)
            {
                mVisHelper.clearFrustumVisualization();
            }
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
        RobotStatePtr state = mCalculator.getRobotModel()->calculateRobotState(resultingViewport.getPosition(), resultingViewport.getSimpleQuaternion());
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
        response.utility = resultingViewport.score->getWeightedNormalizedUtility();
        response.inverse_costs = resultingViewport.score->getWeightedInverseCosts();
        response.base_translation_inverse_costs = resultingViewport.score->getUnweightedInverseMovementCostsBaseTranslation();
        response.base_rotation_inverse_costs = resultingViewport.score->getUnweightedInverseMovementCostsBaseRotation();
        response.ptu_movement_inverse_costs = resultingViewport.score->getUnweightedInverseMovementCostsPTU();
        response.recognition_inverse_costs = resultingViewport.score->getUnweightedInverseRecognitionCosts();

        mCurrentCameraViewport = resultingViewport;

        SimpleVector3 position = TypeHelper::getSimpleVector3(response.resulting_pose);
        SimpleQuaternion orientation = TypeHelper::getSimpleQuaternion(response.resulting_pose);
        mCalculator.getCameraModelFilter()->setPivotPointPose(position, orientation);

        this->triggerVisualization(resultingViewport);

        //Save next best view in world model to present points within it to be considered in future next best view estimation runs.
        world_model::PushViewport pushViewportServiceCall;
        pushViewportServiceCall.request.viewport.pose = resultingViewport.getPose();
        BOOST_FOREACH(std::string objectType, *resultingViewport.object_type_set) {
            pushViewportServiceCall.request.viewport.type = objectType;
            mPushViewportServiceClient.call(pushViewportServiceCall);
        }
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
                                        << mCalculator.getActiveIndices()->size(),
                                    DebugHelper::SERVICE_CALLS);
        mCalculator.doFrustumCulling(point, orientation, mCalculator.getActiveIndices(), viewportPoint);
        mDebugHelperPtr->write("Do update object point cloud", DebugHelper::SERVICE_CALLS);

        // copy objects to be updated from list to set
        ObjectTypeSetPtr objectTypeSetPtr = ObjectTypeSetPtr(new ObjectTypeSet(request.object_type_name_list.begin(), request.object_type_name_list.end()));
        unsigned int deactivatedNormals = mCalculator.updateObjectPointCloud(objectTypeSetPtr, viewportPoint);

        response.deactivated_object_normals = deactivatedNormals;

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
        mCalculator.getCameraModelFilter()->setPivotPointPose(position, orientation);

        mVisHelper.triggerNewFrustumVisualization(mCalculator.getCameraModelFilter());

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
        mCalculator.getCameraModelFilter()->setPivotPointPose(position, orientation);

        mVisHelper.triggerOldFrustumVisualization(this->mCalculator.getCameraModelFilter());

        mDebugHelperPtr->writeNoticeably("ENDING NBV TRIGGER-OLD-FRUSTUM-VISUALIZATION SERVICE CALL", DebugHelper::SERVICE_CALLS);
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
        mDebugHelperPtr->write(std::stringstream() << "Frustum Pivot Point : " << this->mCalculator.getCameraModelFilter()->getPivotPointPosition()[0]
                                        << " , " <<  this->mCalculator.getCameraModelFilter()->getPivotPointPosition()[1]
                                        << " , " << this->mCalculator.getCameraModelFilter()->getPivotPointPosition()[2],
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
                pointCloudIndices = *mCalculator.getActiveIndices();
            }

            ObjectPointCloud objectPointCloud = ObjectPointCloud(*mCalculator.getPointCloudPtr(), pointCloudIndices);
            std::map<std::string, std::string> typeToMeshResource = this->getMeshResources(objectPointCloud);

            mVisHelper.triggerObjectPointCloudVisualization(objectPointCloud, typeToMeshResource);
        }
        if (mShowFrustumPointCloud)
        {
            // publish frustum object point cloud
            ObjectPointCloud frustumObjectPointCloud = ObjectPointCloud(*mCalculator.getPointCloudPtr(), *viewport.child_indices);
            std::map<std::string, std::string> typeToMeshResource = this->getMeshResources(frustumObjectPointCloud);

            mVisHelper.triggerFrustumObjectPointCloudVisualization(frustumObjectPointCloud, typeToMeshResource);
        }
        if (mShowFrustumMarkerArray && publishFrustum)
        {
            // publish furstums visualization
            mVisHelper.triggerFrustumsVisualization(this->mCalculator.getCameraModelFilter());
        }
        mCurrentlyPublishingVisualization = false;
    }

    void publishPointCloudVisualization() {
        mDebugHelperPtr->write("Publishing Visualization without viewport", DebugHelper::VISUALIZATION);

        if (mShowPointcloud)
        {
            // publish object point cloud
            ObjectPointCloud objectPointCloud = ObjectPointCloud(*mCalculator.getPointCloudPtr(), *mCalculator.getActiveIndices());
            std::map<std::string, std::string> typeToMeshResource = this->getMeshResources(objectPointCloud);

            mVisHelper.triggerObjectPointCloudVisualization(objectPointCloud, typeToMeshResource);
        }
    }

    NextBestViewCalculator& getCalculator() {
        return mCalculator;
    }

    void getIndicesOutsideFrustum(const ViewportPoint &viewport, Indices &resultIndices) {
        IndicesPtr activeIndices = mCalculator.getActiveIndices();
        IndicesPtr childIndices = viewport.child_indices;
        std::set_difference(activeIndices->begin(), activeIndices->end(),
                            childIndices->begin(), childIndices->end(),
                            std::back_inserter(resultIndices));
    }

    std::map<std::string, std::string> getMeshResources(ObjectPointCloud objectPointCloud) {
        std::map<std::string, std::string> typeToMeshResource;
        for(ObjectPointCloud::iterator it = objectPointCloud.begin(); it < objectPointCloud.end(); it++) {
            if (typeToMeshResource.count(it->type) == 0) {
                std::string path = mCalculator.getMeshPathByName(it->type);
                // check if the path is valid
                if (path.compare("-2") != 0) {
                    typeToMeshResource[it->type] = mCalculator.getMeshPathByName(it->type);
                }
            }
        }
        return typeToMeshResource;
    }

    void dynamicReconfigureCallback(DynamicParametersConfig &config, uint32_t level) {
        // TODO split this up
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

