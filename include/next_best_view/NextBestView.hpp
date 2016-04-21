/*
 * NextBestView.hpp
 *
 *  Created on: Aug 31, 2014
 *      Author: ralfschleicher
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
#include <tf/transform_listener.h>
#include <world_model/GetViewportList.h>
#include <world_model/PushViewport.h>
#include <std_msgs/ColorRGBA.h>

// Local Includes
#include "typedef.hpp"
#include "next_best_view/SetupVisualization.h"
#include "next_best_view/TriggerFrustumVisualization.h"
#include "next_best_view/GetAttributedPointCloud.h"
#include "next_best_view/GetPointCloud2.h"
#include "next_best_view/GetNextBestView.h"
#include "next_best_view/GetSpaceSampling.h"
#include "next_best_view/SetAttributedPointCloud.h"
#include "next_best_view/ResetCalculator.h"
#include "next_best_view/UpdatePointCloud.h"
#include "next_best_view/helper/MapHelper.hpp"
#include "next_best_view/NextBestViewCalculator.hpp"
#include "next_best_view/helper/VisualizationsHelper.hpp"
#include "pbd_msgs/PbdAttributedPointCloud.h"
#include "pbd_msgs/PbdAttributedPoint.h"
#include "next_best_view/camera_model_filter/impl/Raytracing2DBasedSingleCameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/impl/SingleCameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/impl/Raytracing2DBasedStereoCameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/impl/StereoCameraModelFilter.hpp"
#include "next_best_view/helper/DebugHelper.hpp"
#include "next_best_view/helper/VisualizationsHelper.hpp"
#include "next_best_view/hypothesis_updater/impl/PerspectiveHypothesisUpdater.hpp"
#include "next_best_view/robot_model/impl/MILDRobotModel_with_IK.hpp"
#include "next_best_view/robot_model/impl/MILDRobotModel.hpp"
#include "next_best_view/robot_model/impl/MILDRobotState.hpp"
#include "next_best_view/unit_sphere_sampler/impl/SpiralApproxUnitSphereSampler.hpp"
#include "next_best_view/space_sampler/impl/PlaneSubSpaceSampler.hpp"
#include "next_best_view/space_sampler/impl/Raytracing2DBasedSpaceSampler.hpp"
#include "next_best_view/space_sampler/impl/MapBasedHexagonSpaceSampler.hpp"
#include "next_best_view/space_sampler/impl/MapBasedRandomSpaceSampler.hpp"

#include "next_best_view/rating/impl/DefaultRatingModule.hpp"

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
    ros::ServiceServer mGetNextBestViewServiceServer;
    ros::ServiceServer mGetSpaceSamplingServiceServer;
    ros::ServiceServer mUpdatePointCloudServiceServer;
    ros::ServiceServer mSetupVisualizationServiceServer;
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
    ObjectPointCloudPtr mPointCloudPtr;
    KdTreePtr mKdTreePtr;
    DebugHelperPtr mDebugHelperPtr;
    VisualizationHelper mVisHelper;
    SetupVisualizationRequest mVisualizationSettings;
    bool mCurrentlyPublishingVisualization;

    // Bool for set point cloud flags
    bool mEnableIntermediateObjectWeighting;
    bool mEnableCropBoxFiltering;

public:
    /*!
         * \brief Creates an instance of the NextBestView class.
         */
    NextBestView()
    {
        mGlobalNodeHandle = ros::NodeHandle();
        mNodeHandle = ros::NodeHandle(ros::this_node::getName());
        mGetPointCloud2ServiceServer = mNodeHandle.advertiseService("get_point_cloud2", &NextBestView::processGetPointCloud2ServiceCall, this);
        mGetPointCloudServiceServer = mNodeHandle.advertiseService("get_point_cloud", &NextBestView::processGetPointCloudServiceCall, this);
        mGetNextBestViewServiceServer = mNodeHandle.advertiseService("next_best_view", &NextBestView::processGetNextBestViewServiceCall, this);
        mSetPointCloudServiceServer = mNodeHandle.advertiseService("set_point_cloud", &NextBestView::processSetPointCloudServiceCall, this);
        mUpdatePointCloudServiceServer = mNodeHandle.advertiseService("update_point_cloud", &NextBestView::processUpdatePointCloudServiceCall, this);
        mGetSpaceSamplingServiceServer = mNodeHandle.advertiseService("get_space_sampling", &NextBestView::processGetSpaceSamplingServiceCall, this);
        mSetupVisualizationServiceServer = mNodeHandle.advertiseService("setup_visualization", &NextBestView::processSetupVisualizationServiceCall, this);
        mTriggerFrustumVisualizationServer = mNodeHandle.advertiseService("trigger_frustum_visualization", &NextBestView::processTriggerFrustumVisualization, this);
        mTriggerOldFrustumVisualizationServer = mNodeHandle.advertiseService("trigger_old_frustum_visualization", &NextBestView::processTriggerOldFrustumVisualization, this);
        mResetCalculatorServer = mNodeHandle.advertiseService("reset_nbv_calculator", &NextBestView::processResetCalculatorServiceCall, this);

        mPushViewportServiceClient = mGlobalNodeHandle.serviceClient<world_model::PushViewport>("/env/world_model/push_viewport");
        mGetViewportListServiceClient = mGlobalNodeHandle.serviceClient<world_model::GetViewportList>("/env/world_model/get_viewport_list");

        mDebugHelperPtr = DebugHelper::getInstance();

        initialize();
    }

    //dtor
    virtual ~NextBestView() { }

    void initialize()
    {
        mDebugHelperPtr->writeNoticeably("STARTING NBV PARAMETER OUTPUT", DebugHelper::PARAMETERS);

        mDebugHelperPtr->write(std::stringstream() << "debugLevels: " << mDebugHelperPtr->getLevelString(), DebugHelper::PARAMETERS);

        mPointCloudPtr = ObjectPointCloudPtr(new ObjectPointCloud());
        mCurrentlyPublishingVisualization = false;

        // setup the visualization defaults
        bool show_space_sampling, show_point_cloud, show_frustum_point_cloud, show_frustum_marker_array, move_robot;
        mNodeHandle.param("show_space_sampling", show_space_sampling, false);
        mNodeHandle.param("show_point_cloud", show_point_cloud, false);
        mNodeHandle.param("show_frustum_point_cloud", show_frustum_point_cloud, false);
        mNodeHandle.param("show_frustum_marker_array", show_frustum_marker_array, false);
        mNodeHandle.param("move_robot", move_robot, false);

        //get XML path
        std::string mCropBoxListFilePath;
        mNodeHandle.param("mCropBoxListFilePath", mCropBoxListFilePath,std::string());

        //Set point cloud parameters flags
        mNodeHandle.param("enableCropBoxFiltering", mEnableCropBoxFiltering, false);
        mNodeHandle.param("enableIntermediateObjectWeighting", mEnableIntermediateObjectWeighting, false);

        // assign the values to the settings struct
        mVisualizationSettings.space_sampling = show_space_sampling;
        mVisualizationSettings.point_cloud = show_point_cloud;
        mVisualizationSettings.frustum_point_cloud = show_frustum_point_cloud;
        mVisualizationSettings.frustum_marker_array = show_frustum_marker_array;
        mVisualizationSettings.move_robot = move_robot;

        /* These are the parameters for the CameraModelFilter. By now they will be kept in here, but for the future they'd better
            * be defined in the CameraModelFilter specialization class.
            * TODO: Export these parameters to the specialization of the CameraModelFilter class. This makes sense, because you have to
            * keep in mind that there are stereo cameras which might have slightly different settings of the frustums. So we will be
            * able to adjust the parameters for each camera separateley.
            */
        double fovx, fovy, ncp, fcp, speedFactorRecognizer;
        double radius,sampleSizeUnitSphereSampler,colThresh;
        mNodeHandle.param("fovx", fovx, 62.5);
        mNodeHandle.param("fovy", fovy, 48.9);
        mNodeHandle.param("ncp", ncp, .5);
        mNodeHandle.param("fcp", fcp, 5.0);
        mNodeHandle.param("radius", radius, 0.75);
        mNodeHandle.param("colThresh", colThresh, 45.0);
        mNodeHandle.param("sampleSizeUnitSphereSampler", sampleSizeUnitSphereSampler, 128.0);
        mNodeHandle.param("speedFactorRecognizer", speedFactorRecognizer, 5.0);
        mDebugHelperPtr->write(std::stringstream() << "fovx: " << fovx, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "fovy: " << fovy, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "ncp: " << ncp, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "fcp: " << fcp, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "radius: " << radius, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "colThresh: " << colThresh, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "samples: " << sampleSizeUnitSphereSampler, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "speedFactorRecognizer: " << speedFactorRecognizer, DebugHelper::PARAMETERS);

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
             */
        SpiralApproxUnitSphereSamplerPtr unitSphereSamplerPtr(new SpiralApproxUnitSphereSampler());
        unitSphereSamplerPtr->setSamples(sampleSizeUnitSphereSampler);

        /* MapHelper does get the maps on which we are working on and modifies them for use with applications like raytracing and others.
             * TODO: The maps may have areas which are marked feasible but in fact are not, because of different reasons. The main
             * reason may be that the map may contain areas where the robot cannot fit through and therefore cannot move to the
             * wanted position. You have to consider if there is any possibility to mark these areas as non-feasible.
             */
        MapHelperPtr mapHelperPtr(new MapHelper());
        mapHelperPtr->setCollisionThreshold(colThresh);


        /*
             * Intializes SpaceSampler spaceSampler with the SpaceSampler sublcass specified by the parameter samplerId :
             * - samplerId == 1 => MapBasedHexagonSpaceSampler
             * - smaplerId == 2 => MapBasedRandomSpaceSampler
             */

        int sampleSizeMapBasedRandomSpaceSampler, spaceSamplerId;
        mNodeHandle.param("sampleSizeMapBasedRandomSpaceSampler", sampleSizeMapBasedRandomSpaceSampler, 100);
        mNodeHandle.param("spaceSamplerId", spaceSamplerId, 1);

        mDebugHelperPtr->write(std::stringstream() << "sampleSizeMapBasedRandomSpaceSampler: " << sampleSizeMapBasedRandomSpaceSampler, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "spaceSamplerId: " << spaceSamplerId, DebugHelper::PARAMETERS);

        SpaceSamplerPtr spaceSamplerPtr;
        MapBasedRandomSpaceSamplerPtr mapBasedRandomSpaceSampler;
        MapBasedHexagonSpaceSamplerPtr mapBasedHexagonSpaceSampler;
        switch (spaceSamplerId)
        {
        case 1:
            /* MapBasedHexagonSpaceSampler is a specialization of the abstract SpaceSampler class.
                 * By space we denote the area in which the robot is moving. In our case there are just two degrees of freedom
                 * in which the robot can move, namely the xy-plane. But we do also have a map on which we can base our sampling on.
                 * There are a lot of ways to sample the xy-plane into points but we decided to use a hexagonal grid which we lay over
                 * the map and calculate the points which are contained in the feasible map space.
                 */
            mapBasedHexagonSpaceSampler = MapBasedHexagonSpaceSamplerPtr(new MapBasedHexagonSpaceSampler(mapHelperPtr));
            mapBasedHexagonSpaceSampler->setHexagonRadius(radius);
            spaceSamplerPtr = mapBasedHexagonSpaceSampler;
            break;
        case 2:
            mapBasedRandomSpaceSampler = MapBasedRandomSpaceSamplerPtr(new MapBasedRandomSpaceSampler(mapHelperPtr, sampleSizeMapBasedRandomSpaceSampler));
            spaceSamplerPtr = mapBasedRandomSpaceSampler;
            break;
        default:
            std::stringstream ss;
            ss << spaceSamplerId << " is not a valid sampler ID";
            ROS_ERROR_STREAM(ss.str());
            throw std::runtime_error(ss.str());
            break;

        }

        /* MapBasedSingleCameraModelFilterPtr is a specialization of the abstract CameraModelFilter class.
             * The camera model filter takes account for the fact, that there are different cameras around in the real world.
             * In respect of the parameters of these cameras the point cloud gets filtered by theses camera filters. Plus
             * the map-based version of the camera filter also uses the knowledge of obstacles or walls between the camera and
             * the object to be observed. So, map-based in this context means that the way from the lens to the object is ray-traced.
             */
        //MapBasedSingleCameraModelFilterPtr cameraModelFilterPtr(new MapBasedSingleCameraModelFilter(mapHelperPtr, SimpleVector3(0.0, 0.0, 0.1)));
        bool useRaytracing;
        mNodeHandle.param("useRaytracing", useRaytracing, false);
        CameraModelFilterPtr cameraModelFilterPtr;
        if (useRaytracing)
            cameraModelFilterPtr = CameraModelFilterPtr(new Raytracing2DBasedStereoCameraModelFilter(mapHelperPtr, SimpleVector3(0.0, -0.067 , 0.04), SimpleVector3(0, 0.086, 0.04)));
        else
            cameraModelFilterPtr = CameraModelFilterPtr(new StereoCameraModelFilter(SimpleVector3(0.0, -0.067 , 0.04), SimpleVector3(0, 0.086, 0.04)));

        cameraModelFilterPtr->setHorizontalFOV(fovx);
        cameraModelFilterPtr->setVerticalFOV(fovy);
        cameraModelFilterPtr->setNearClippingPlane(ncp);
        cameraModelFilterPtr->setFarClippingPlane(fcp);
        cameraModelFilterPtr->setRecognizerCosts((float)speedFactorRecognizer, "");

        double panMin, panMax, tiltMin, tiltMax;
        mNodeHandle.param("panMin", panMin, -60.);
        mNodeHandle.param("panMax", panMax, 60.);
        mNodeHandle.param("tiltMin", tiltMin, -45.);
        mNodeHandle.param("tiltMax", tiltMax, 45.);
        mDebugHelperPtr->write(std::stringstream() << "panMin: " << panMin, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "panMax: " << panMax, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "tiltMin: " << tiltMin, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "tiltMax: " << tiltMax, DebugHelper::PARAMETERS);

        /* MILDRobotModel is a specialization of the abstract RobotModel class.
             * The robot model maps takes the limitations of the used robot into account and by this it is possible to filter out
             * non-reachable configurations of the robot which can therefore be ignored during calculation.
             */
        bool useNewIK;
        mNodeHandle.param("useNewIK", useNewIK, false);
        RobotModelPtr robotModelPtr;
        if (useNewIK)
        {
            mDebugHelperPtr->write("NBV: Using new IK model", DebugHelper::PARAMETERS);
            MILDRobotModelWithIK *tempRobotModel = new MILDRobotModelWithIK();
            tempRobotModel->setTiltAngleLimits(tiltMin, tiltMax);
            tempRobotModel->setPanAngleLimits(panMin, panMax);
            robotModelPtr = MILDRobotModelWithIKPtr(tempRobotModel);
        }
        else
        {
            mDebugHelperPtr->write("NBV: Using old IK model", DebugHelper::PARAMETERS);
            MILDRobotModel *tempRobotModel = new MILDRobotModel();
            tempRobotModel->setTiltAngleLimits(tiltMin, tiltMax);
            tempRobotModel->setPanAngleLimits(panMin, panMax);
            robotModelPtr = MILDRobotModelPtr(tempRobotModel);
        }


        /* DefaultRatingModule is a specialization of the abstract RatingModule class.
             * The rating module calculates the use and the costs of an operation.
             */
        DefaultRatingModulePtr ratingModulePtr(new DefaultRatingModule(fovx,fovy,fcp,ncp,robotModelPtr, cameraModelFilterPtr));
        ratingModulePtr->setNormalAngleThreshold(45 / 180.0 * M_PI);

        double mOmegaUtility, mOmegaTilt, mOmegaPan, mOmegaRot, mOmegaBase, mOmegaRecognizer;
        mNodeHandle.param("mOmegaUtility", mOmegaUtility, 1.0);
        mNodeHandle.param("mOmegaTilt", mOmegaTilt, 1.0);
        mNodeHandle.param("mOmegaPan", mOmegaPan, 1.0);
        mNodeHandle.param("mOmegaRot", mOmegaRot, 1.0);
        mNodeHandle.param("mOmegaBase", mOmegaBase, 1.0);
        mNodeHandle.param("mOmegaRecognizer", mOmegaRecognizer, 1.0);

        ratingModulePtr->setOmegaParameters(mOmegaUtility, mOmegaPan, mOmegaTilt, mOmegaRot,mOmegaBase, mOmegaRecognizer);

        /* PerspectiveHypothesisUpdater is a specialization of the abstract HypothesisUpdater.
             */
        PerspectiveHypothesisUpdaterPtr hypothesisUpdaterPtr(new PerspectiveHypothesisUpdater());
        hypothesisUpdaterPtr->setDefaultRatingModule(ratingModulePtr);

        // The setting of the modules.
        mCalculator.setHypothesisUpdater(hypothesisUpdaterPtr);
        mCalculator.setUnitSphereSampler(unitSphereSamplerPtr);
        mCalculator.setSpaceSampler(spaceSamplerPtr);
        mCalculator.setCameraModelFilter(cameraModelFilterPtr);
        mCalculator.setRobotModel(robotModelPtr);
        mCalculator.setRatingModule(ratingModulePtr);
        mCalculator.loadCropBoxListFromFile(mCropBoxListFilePath);
        mCalculator.setEnableCropBoxFiltering(mEnableCropBoxFiltering);
        mCalculator.setEnableIntermediateObjectWeighting(mEnableIntermediateObjectWeighting);
        //Set the max amout of iterations
        int maxIterationSteps;
        mNodeHandle.param("maxIterationSteps", maxIterationSteps, 20);
        mDebugHelperPtr->write(std::stringstream() << "maxIterationSteps: " << maxIterationSteps, DebugHelper::PARAMETERS);
        mCalculator.setMaxIterationSteps(maxIterationSteps);

        mDebugHelperPtr->write(std::stringstream() << "boolClearBetweenIterations: " << mVisHelper.getBoolClearBetweenIterations(), DebugHelper::PARAMETERS);

        mDebugHelperPtr->writeNoticeably("ENDING NBV PARAMETER OUTPUT", DebugHelper::PARAMETERS);
    }

    bool processSetupVisualizationServiceCall(SetupVisualizationRequest &request, SetupVisualizationResponse &response) {
        mDebugHelperPtr->writeNoticeably("STARTING NBV SETUP-VISUALIZATION SERVICE CALL", DebugHelper::SERVICE_CALLS);
        mVisualizationSettings = SetupVisualizationRequest(request);

        this->triggerVisualization(mCurrentCameraViewport);

        mDebugHelperPtr->writeNoticeably("ENDING NBV SETUP-VISUALIZATION SERVICE CALL", DebugHelper::SERVICE_CALLS);
        return true;
    }

    bool processResetCalculatorServiceCall(ResetCalculator::Request &request, ResetCalculator::Response &response) {
        mDebugHelperPtr->writeNoticeably("STARTING NBV RESET-CALCULATOR SERVICE CALL", DebugHelper::SERVICE_CALLS);
        initialize();

        mDebugHelperPtr->writeNoticeably("ENDING NBV RESET-CALCULATOR SERVICE CALL", DebugHelper::SERVICE_CALLS);
        return true;
    }

    bool processGetSpaceSamplingServiceCall(GetSpaceSampling::Request &request, GetSpaceSampling::Response &response) {
        mDebugHelperPtr->writeNoticeably("STARTING NBV GET-SPACE-SAMPLING SERVICE CALL", DebugHelper::SERVICE_CALLS);
        double contractor = request.contractor;
        SimpleVector3 position = TypeHelper::getSimpleVector3(request.position);

        SamplePointCloudPtr pointcloud = mCalculator.getSpaceSampler()->getSampledSpacePointCloud(position, contractor);
        mCalculator.setHeight(pointcloud, position[2]);
        IndicesPtr feasibleIndices(new Indices());
        mCalculator.getFeasibleSamplePoints(pointcloud, feasibleIndices);

        SamplePointCloud pcl = SamplePointCloud(*pointcloud, *feasibleIndices);
        pcl::toROSMsg(pcl, response.point_cloud);
        response.point_cloud.header.frame_id = "/map";
        sensor_msgs::PointField rgb;
        rgb.name = "rgb";
        rgb.datatype = sensor_msgs::PointField::UINT32;
        rgb.offset = offsetof(ObjectPoint, rgb);
        response.point_cloud.fields.push_back(rgb);

        mDebugHelperPtr->writeNoticeably("ENDING NBV GET-SPACE-SAMPLING SERVICE CALL", DebugHelper::SERVICE_CALLS);
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

    bool processGetPointCloud2ServiceCall(GetPointCloud2::Request &request, GetPointCloud2::Response &response) {
        mDebugHelperPtr->writeNoticeably("STARTING NBV GET-POINT-CLOUD2 SERVICE CALL", DebugHelper::SERVICE_CALLS);
        pcl::toROSMsg(*mCalculator.getPointCloudPtr(), response.point_cloud);
        response.point_cloud.header.frame_id = "/map";

        sensor_msgs::PointField rgb;
        rgb.name = "rgb";
        rgb.datatype = sensor_msgs::PointField::UINT32;
        rgb.offset = offsetof(ObjectPoint, rgb);
        response.point_cloud.fields.push_back(rgb);

        mDebugHelperPtr->writeNoticeably("ENDING NBV GET-POINT-CLOUD2 SERVICE CALL", DebugHelper::SERVICE_CALLS);
        return true;
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

        mCurrentCameraViewport = ViewportPoint(request.pose);
        mCalculator.getCameraModelFilter()->setOrientation(mCurrentCameraViewport.getSimpleQuaternion());
        mCalculator.getCameraModelFilter()->setPivotPointPosition(mCurrentCameraViewport.getPosition());

        MILDRobotStatePtr currentRobotStatePtr(new MILDRobotState());
        currentRobotStatePtr->pan = 0;
        currentRobotStatePtr->tilt = 0;
        currentRobotStatePtr->rotation = 0;
        currentRobotStatePtr->x = mCurrentCameraViewport.x;
        currentRobotStatePtr->y = mCurrentCameraViewport.y;
        mCalculator.getRobotModel()->setCurrentRobotState(currentRobotStatePtr);
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
        this->publishVisualization(request.pose, false);
        mDebugHelperPtr->writeNoticeably("ENDING NBV SET-POINT-CLOUD SERVICE CALL", DebugHelper::SERVICE_CALLS);
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
            if (mVisualizationSettings.frustum_marker_array)
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
        SimpleVector3 point = TypeHelper::getSimpleVector3(request.update_pose);
        SimpleQuaternion orientation = TypeHelper::getSimpleQuaternion(request.update_pose);
        ViewportPoint viewportPoint;
        mDebugHelperPtr->write(std::stringstream() << "Do frustum culling: ActiveIndices="
                                        << mCalculator.getActiveIndices()->size(),
                                    DebugHelper::SERVICE_CALLS);
        mCalculator.doFrustumCulling(point, orientation, mCalculator.getActiveIndices(), viewportPoint);
        mDebugHelperPtr->write("Do update object point cloud", DebugHelper::SERVICE_CALLS);
        unsigned int deactivatedNormals = mCalculator.updateObjectPointCloud(viewportPoint);

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

        boost::thread t = boost::thread(&NextBestView::publishVisualization, this, viewport, true);
        return true;
    }

    /*!
     * \brief Publishes the Visualization of the NextBestView
     * \param publishFrustum whether the frustum should be published
     */
    void publishVisualization(ViewportPoint viewport, bool publishFrustum) {
        mDebugHelperPtr->write("Publishing Visualization", DebugHelper::VISUALIZATION);
        mDebugHelperPtr->write(std::stringstream() << "Frustum Pivot Point : " << this->mCalculator.getCameraModelFilter()->getPivotPointPosition()[0]
                                        << " , " <<  this->mCalculator.getCameraModelFilter()->getPivotPointPosition()[1]
                                        << " , " << this->mCalculator.getCameraModelFilter()->getPivotPointPosition()[2],
                                    DebugHelper::VISUALIZATION);

        mDebugHelperPtr->write(std::stringstream() << "Object points in the viewport: " << viewport.child_indices->size(),
                    DebugHelper::VISUALIZATION);

        if (mVisualizationSettings.point_cloud)
        {
            // publish object point cloud
            Indices pointCloudIndices;
            if (mVisualizationSettings.frustum_point_cloud) {
                // if the frustum point cloud is published seperately only publish points outside frustum
                this->getIndicesOutsideFrustum(viewport, pointCloudIndices);
            } else {
                pointCloudIndices = *mCalculator.getActiveIndices();
            }

            ObjectPointCloud objectPointCloud = ObjectPointCloud(*mCalculator.getPointCloudPtr(), pointCloudIndices);
            std::map<std::string, std::string> typeToMeshResource = this->getMeshResources(objectPointCloud);

            mVisHelper.triggerObjectPointCloudVisualization(objectPointCloud, typeToMeshResource);
        }
        if (mVisualizationSettings.frustum_point_cloud)
        {
            // publish frustum object point cloud
            ObjectPointCloud frustumObjectPointCloud = ObjectPointCloud(*mCalculator.getPointCloudPtr(), *viewport.child_indices);
            std::map<std::string, std::string> typeToMeshResource = this->getMeshResources(frustumObjectPointCloud);

            mVisHelper.triggerFrustumObjectPointCloudVisualization(frustumObjectPointCloud, typeToMeshResource);
        }
        if (mVisualizationSettings.frustum_marker_array && publishFrustum)
        {
            // publish furstums visualization
            mVisHelper.triggerFrustumsVisualization(this->mCalculator.getCameraModelFilter());
        }
        mCurrentlyPublishingVisualization = false;
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
};
}

