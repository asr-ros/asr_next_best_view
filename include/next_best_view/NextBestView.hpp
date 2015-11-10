/*
 * NextBestView.hpp
 *
 *  Created on: Aug 31, 2014
 *      Author: ralfschleicher
 */

#ifndef NEXTBESTVIEW_HPP_
#define NEXTBESTVIEW_HPP_

// Global Includes
#include <algorithm>
#include <boost/filesystem.hpp>
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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
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
#include "next_best_view/helper/MarkerHelper.hpp"
#include "next_best_view/helper/MathHelper.hpp"
#include "pbd_msgs/PbdAttributedPointCloud.h"
#include "pbd_msgs/PbdAttributedPoint.h"
#include "next_best_view/camera_model_filter/impl/MapBasedSingleCameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/impl/SingleCameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/impl/MapBasedStereoCameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/impl/StereoCameraModelFilter.hpp"
#include "next_best_view/hypothesis_updater/impl/PerspectiveHypothesisUpdater.hpp"
#include "next_best_view/robot_model/impl/MILDRobotModel.hpp"
#include "next_best_view/robot_model/impl/MILDRobotState.hpp"
#include "next_best_view/unit_sphere_sampler/impl/SpiralApproxUnitSphereSampler.hpp"
#include "next_best_view/space_sampler/impl/PlaneSubSpaceSampler.hpp"
#include "next_best_view/space_sampler/impl/MapBasedSpaceSampler.hpp"
#include "next_best_view/space_sampler/impl/MapBasedHexagonSpaceSampler.hpp"
#include "next_best_view/rating/impl/DefaultRatingModule.hpp"


namespace next_best_view {
	// Defining namespace shorthandles
	namespace fs = boost::filesystem;
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

		ros::Publisher mSpaceSamplingPublisher;
		ros::Publisher mPointCloudPublisher;
		ros::Publisher mFrustumPointCloudPublisher;
		ros::Publisher mUnitSpherPointCloudPublisher;
		ros::Publisher mFrustumMarkerArrayPublisher;
		ros::Publisher mInitialPosePublisher;
        ros::Publisher mPointObjectNormalPublisher;
        ros::Publisher mObjectMeshMarkerPublisher;
        ros::Publisher mFrustumObjectMeshMarkerPublisher;

		// Action Clients
		MoveBaseActionClientPtr mMoveBaseActionClient;

		// ServiceClients and Subscriber
		ros::ServiceClient mObjectTypeServiceClient;
		ros::ServiceClient mPushViewportServiceClient;
		ros::ServiceClient mGetViewportListServiceClient;

		// Etcetera
		ViewportPoint mCurrentCameraViewport;
		ObjectPointCloudPtr mPointCloudPtr;
		KdTreePtr mKdTreePtr;
		SetupVisualizationRequest mVisualizationSettings;
		bool mCurrentlyPublishingVisualization;
        viz::MarkerArray::Ptr mMarkerArrayPtr;
        viz::MarkerArray::Ptr mMeshMarkerArrayPtr;
        viz::MarkerArray::Ptr mFrustumMeshMarkerArrayPtr;
        viz::MarkerArray::Ptr mObjectNormalsMarkerArrayPtr;
        unsigned int numberSearchedObjects;
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

            mSpaceSamplingPublisher = mNodeHandle.advertise<sensor_msgs::PointCloud2>("space_sampling_point_cloud", 100);
            mPointCloudPublisher = mNodeHandle.advertise<sensor_msgs::PointCloud2>("point_cloud", 1000);
            mFrustumPointCloudPublisher = mNodeHandle.advertise<sensor_msgs::PointCloud2>("frustum_point_cloud", 1000);
            mFrustumMarkerArrayPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>("frustum_marker_array", 1000);
            mInitialPosePublisher = mNodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 100, false);
            mPointObjectNormalPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>("/nbv/object_normals", 100, false);
            mObjectMeshMarkerPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>("/nbv/object_meshes", 100, false);
            mFrustumObjectMeshMarkerPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>("/nbv/frustum_object_meshes", 100, false);

            mObjectTypeServiceClient = mGlobalNodeHandle.serviceClient<odb::ObjectType>("/object_database/object_type");
            mPushViewportServiceClient = mGlobalNodeHandle.serviceClient<world_model::PushViewport>("/env/world_model/push_viewport");
            mGetViewportListServiceClient = mGlobalNodeHandle.serviceClient<world_model::GetViewportList>("/env/world_model/get_viewport_list");

            initialize();
		}

		//dtor
		virtual ~NextBestView() { }

        void initialize()
        {
            mPointCloudPtr = ObjectPointCloudPtr(new ObjectPointCloud());
            mCurrentlyPublishingVisualization = false;

            // setup the visualization defaults
            bool show_space_sampling, show_point_cloud, show_frustum_point_cloud, show_frustum_marker_array, move_robot;
            mNodeHandle.param("show_space_sampling", show_space_sampling, false);
            mNodeHandle.param("show_point_cloud", show_point_cloud, false);
            mNodeHandle.param("show_frustum_point_cloud", show_frustum_point_cloud, false);
            mNodeHandle.param("show_frustum_marker_array", show_frustum_marker_array, false);
            mNodeHandle.param("move_robot", move_robot, false);

            // assign the values to the settings struct
            mVisualizationSettings.space_sampling = show_space_sampling;
            mVisualizationSettings.point_cloud = show_point_cloud;
            mVisualizationSettings.frustum_point_cloud = show_frustum_point_cloud;
            mVisualizationSettings.frustum_marker_array = show_frustum_marker_array;
            mVisualizationSettings.move_robot = move_robot;

            //Initialize marker arrays
            viz::MarkerArray *mMeshMarkerArray = new viz::MarkerArray();
            viz::MarkerArray *mFrustumMeshMarkerArray = new viz::MarkerArray();
            viz::MarkerArray *mObjectNormalsMarkerArray = new viz::MarkerArray();
            mMeshMarkerArrayPtr = boost::make_shared<viz::MarkerArray>(*mMeshMarkerArray);
            mFrustumMeshMarkerArrayPtr = boost::make_shared<viz::MarkerArray>(*mFrustumMeshMarkerArray);
            mObjectNormalsMarkerArrayPtr = boost::make_shared<viz::MarkerArray>(*mObjectNormalsMarkerArray);

            /* These are the parameters for the CameraModelFilter. By now they will be kept in here, but for the future they'd better
            * be defined in the CameraModelFilter specialization class.
            * TODO: Export these parameters to the specialization of the CameraModelFilter class. This makes sense, because you have to
            * keep in mind that there are stereo cameras which might have slightly different settings of the frustums. So we will be
            * able to adjust the parameters for each camera separateley.
            */
            numberSearchedObjects = 0;
            double fovx, fovy, ncp, fcp, speedFactorRecognizer;
            double radius,samples,colThresh;
            mNodeHandle.param("fovx", fovx, 62.5);
            mNodeHandle.param("fovy", fovy, 48.9);
            mNodeHandle.param("ncp", ncp, .5);
            mNodeHandle.param("fcp", fcp, 5.0);
            mNodeHandle.param("radius", radius, 0.75);
            mNodeHandle.param("colThresh", colThresh, 45.0);
            mNodeHandle.param("samples", samples, 128.0);
            mNodeHandle.param("speedFactorRecognizer", speedFactorRecognizer, 5.0);
            ROS_DEBUG_STREAM("fovx: " << fovx);
            ROS_DEBUG_STREAM("fovy: " << fovy);
            ROS_DEBUG_STREAM("ncp: " << ncp);
            ROS_DEBUG_STREAM("fcp: " << fcp);
            ROS_DEBUG_STREAM("radius: " << radius);
            ROS_DEBUG_STREAM("colThresh: " << colThresh);
            ROS_DEBUG_STREAM("samples: " << samples);
            ROS_DEBUG_STREAM("speedFactorRecognizer: " << speedFactorRecognizer);

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
             * - RatingModule (rating_module/RatingModule.hpp)viz::MarkerArray
             * - HypothesisUpdater (hypothesis_updater/HypothesisUpdater.hpp)
             *
             * Every of these modules is an abstract class which providDefaultRatingModulees an interface to interact with.
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
            unitSphereSamplerPtr->setSamples(samples);

            /* MapHelper does get the maps on which we are working on and modifies them for use with applications like raytracing and others.
             * TODO: The maps may have areas which are marked feasible but in fact are not, because of different reasons. The main
             * reason may be that the map may contain areas where the robot cannot fit through and therefore cannot move to the
             * wanted position. You have to consider if there is any possibility to mark these areas as non-feasible.
             */
            MapHelperPtr mapHelperPtr(new MapHelper());
            mapHelperPtr->setCollisionThreshold(colThresh);

            /* MapBasedHexagonSpaceSampler is a specialization of the abstract SpaceSampler class.
             * By space we denote the area in which the robot is moving. In our case there are just two degrees of freedom
             * in which the robot can move, namely the xy-plane. But we do also have a map on which we can base our sampling on.
             * There are a lot of ways to sample the xy-plane into points but we decided to use a hexagonal grid which we lay over
             * the map and calculate the points which are contained in the feasible map space.
             */
            MapBasedHexagonSpaceSamplerPtr spaceSamplerPtr(new MapBasedHexagonSpaceSampler(mapHelperPtr));
            spaceSamplerPtr->setHexagonRadius(radius);

            /* MapBasedSingleCameraModelFilterPtr is a specialization of the abstract CameraModelFilter class.
             * The camera model filter takes account for the fact, that there are different cameras around in the real world.
             * In respect of the parameters of these cameras the point cloud gets filtered by theses camera filters. Plus
             * the map-based version of the camera filter also uses the knowledge of obstacles or walls between the camera and
             * the object to be observed. So, map-based in this context means that the way from the lens to the object is ray-traced.
             */
            //MapBasedSingleCameraModelFilterPtr cameraModelFilterPtr(new MapBasedSingleCameraModelFilter(mapHelperPtr, SimpleVector3(0.0, 0.0, 0.1)));
            MapBasedStereoCameraModelFilterPtr cameraModelFilterPtr(new MapBasedStereoCameraModelFilter(mapHelperPtr, SimpleVector3(0.0, -0.067 , 0.04), SimpleVector3(0, 0.086, 0.04)));
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
            ROS_DEBUG_STREAM("panMin: " << panMin);
            ROS_DEBUG_STREAM("panMax: " << panMax);
            ROS_DEBUG_STREAM("tiltMin: " << tiltMin);
            ROS_DEBUG_STREAM("tiltMax: " << tiltMax);

            /* MILDRobotModel is a specialization of the abstract RobotModel class.
             * The robot model maps takes the limitations of the used robot into account and by this it is possible to filter out
             * non-reachable conros::Publishefigurations of the robot which can therefore be ignored during calculation.
             */
            MILDRobotModelPtr robotModelPtr(new MILDRobotModel());
            robotModelPtr->setTiltAngleLimits(tiltMin, tiltMax);
            robotModelPtr->setPanAngleLimits(panMin, panMax);

            /* DefaultRatingModule is a specialization of the abstract RatingModule class.
             * The rating module calculates the use and the costs of an operation.
             */
            DefaultRatingModulePtr ratingModulePtr(new DefaultRatingModule(fovx,fovy,fcp,ncp));
            ratingModulePtr->setNormalityRatingAngle(45 / 180.0 * M_PI);

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

            double mOmegaTilt, mOmegaPan, mOmegaRot, mOmegaBase, mOmegaRecognizer;
            mNodeHandle.param("mOmegaTilt", mOmegaTilt, 1.0);
            mNodeHandle.param("mOmegaPan", mOmegaPan, 1.0);
            mNodeHandle.param("mOmegaRot", mOmegaRot, 1.0);
            mNodeHandle.param("mOmegaBase", mOmegaBase, 1.0);
            mNodeHandle.param("mOmegaRecognizer", mOmegaRecognizer, 1.0);

            mCalculator.setOmegaParameters(mOmegaPan, mOmegaTilt, mOmegaRot,mOmegaBase, mOmegaRecognizer);
        }

		bool processSetupVisualizationServiceCall(SetupVisualizationRequest &request, SetupVisualizationResponse &response) {
			mVisualizationSettings = SetupVisualizationRequest(request);

			this->triggerVisualization(mCurrentCameraViewport);

			return true;
		}

        bool processResetCalculatorServiceCall(ResetCalculator::Request &request, ResetCalculator::Response &response) {
            initialize();

            return true;
        }

		bool processGetSpaceSamplingServiceCall(GetSpaceSampling::Request &request, GetSpaceSampling::Response &response) {
			double contractor = request.contractor;
			SimpleVector3 position = TypeHelper::getSimpleVector3(request.position);

			SamplePointCloudPtr pointcloud = mCalculator.getSpaceSampler()->getSampledSpacePointCloud(position, contractor);
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

			return true;
		}

	  static void convertObjectPointCloudToAttributedPointCloud(const ObjectPointCloud &pointCloud, pbd_msgs::PbdAttributedPointCloud &pointCloudMessage) {
			pointCloudMessage.elements.clear();

			BOOST_FOREACH(ObjectPoint point, pointCloud) {
				pbd_msgs::PbdAttributedPoint aPoint;

				aPoint.pose = point.getPose();
                aPoint.type = point.object_type_name;

				pointCloudMessage.elements.push_back(aPoint);
			}
		}

        bool processGetPointCloud2ServiceCall(GetPointCloud2::Request &request, GetPointCloud2::Response &response) {
			pcl::toROSMsg(*mCalculator.getPointCloudPtr(), response.point_cloud);
			response.point_cloud.header.frame_id = "/map";

			sensor_msgs::PointField rgb;
			rgb.name = "rgb";
			rgb.datatype = sensor_msgs::PointField::UINT32;
			rgb.offset = offsetof(ObjectPoint, rgb);
            response.point_cloud.fields.push_back(rgb);

			return true;
		}

		bool processGetPointCloudServiceCall(GetAttributedPointCloud::Request &request, GetAttributedPointCloud::Response &response) {
			convertObjectPointCloudToAttributedPointCloud(*mCalculator.getPointCloudPtr(), response.point_cloud);

			return true;
		}

		bool processSetPointCloudServiceCall(SetAttributedPointCloud::Request &request, SetAttributedPointCloud::Response &response) {
			if (!mCalculator.setPointCloudFromMessage(request.point_cloud)) {
                ROS_ERROR("Could not set point cloud from message.");
				return false;
			}

            mCurrentCameraViewport = ViewportPoint(request.pose);
			mCalculator.getCameraModelFilter()->setOrientation(mCurrentCameraViewport.getSimpleQuaternion());
			mCalculator.getCameraModelFilter()->setPivotPointPosition(mCurrentCameraViewport.getSimpleVector3());


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
				viewportConversionPoint.object_name_set = boost::shared_ptr<ObjectNameSet>(new ObjectNameSet());
                viewportConversionPoint.object_name_set->insert(point.type);
				viewportPointList.push_back(viewportConversionPoint);
			}

			mCalculator.updateFromExternalObjectPointList(viewportPointList);

			response.is_valid = true;
            ROS_DEBUG_STREAM("processSetPointCloudServiceCall3: " << mCalculator.getCameraModelFilter()->getPivotPointPosition());
			// publish the visualization
            this->publishVisualization(request.pose, true, false);
			return true;
		}

	  //COMMENT?
		bool processGetNextBestViewServiceCall(GetNextBestView::Request &request, GetNextBestView::Response &response) {
			ViewportPoint currentCameraViewport(request.current_pose);

			ViewportPoint resultingViewport;
			if (!mCalculator.calculateNextBestView(currentCameraViewport, resultingViewport)) {
				ROS_DEBUG("No more found");
                if (mVisualizationSettings.frustum_marker_array)
                {
                    if (mMarkerArrayPtr)
                    {
                        deleteFrustumInRviz();
                        mMarkerArrayPtr.reset();
                    }
                }
				response.found = false;
				return true;
			}
			response.found = true;
			response.resulting_pose = resultingViewport.getPose();

			// copying the object to be searched for into a list
			response.object_name_list = ObjectNameList(resultingViewport.object_name_set->size());
			std::copy(resultingViewport.object_name_set->begin(), resultingViewport.object_name_set->end(), response.object_name_list.begin());

			// robot state.
			// TODO: This solution is very dirty because we get the specialization of RobotState and this will break if we change the RobotModel and RobotState type.
			RobotStatePtr state = mCalculator.getRobotModel()->calculateRobotState(resultingViewport.getSimpleVector3(), resultingViewport.getSimpleQuaternion());
			MILDRobotStatePtr mildState = boost::static_pointer_cast<MILDRobotState>(state);

			RobotStateMessage robotStateMsg;
			robotStateMsg.pan = mildState->pan;
			robotStateMsg.tilt = mildState->tilt;
			robotStateMsg.rotation = mildState->rotation;
			robotStateMsg.x = mildState->x;
			robotStateMsg.y = mildState->y;

			// set it to the response
			response.robot_state = robotStateMsg;

			mCurrentCameraViewport = resultingViewport;

			SimpleVector3 position = TypeHelper::getSimpleVector3(response.resulting_pose);
			SimpleQuaternion orientation = TypeHelper::getSimpleQuaternion(response.resulting_pose);
			mCalculator.getCameraModelFilter()->setPivotPointPose(position, orientation);

			ROS_DEBUG("Trigger Visualization");
			this->triggerVisualization(resultingViewport);
			ROS_DEBUG("Visualization triggered");

            // the point cloud should be updated via a service call
            //mCalculator.updateObjectPointCloud(resultingViewport);


			// push to the viewport list.
			world_model::PushViewport pushViewportServiceCall;
			pushViewportServiceCall.request.viewport.pose = resultingViewport.getPose();
			BOOST_FOREACH(std::string objectName, *resultingViewport.object_name_set) {
                pushViewportServiceCall.request.viewport.type = objectName;
				mPushViewportServiceClient.call(pushViewportServiceCall);
			}

			return true;
		}

		bool processUpdatePointCloudServiceCall(UpdatePointCloud::Request &request, UpdatePointCloud::Response &response) {
            ROS_DEBUG_STREAM("Called service processUpdatePointCloudServiceCall");
			SimpleVector3 point = TypeHelper::getSimpleVector3(request.update_pose);
			SimpleQuaternion orientation = TypeHelper::getSimpleQuaternion(request.update_pose);
			ViewportPoint viewportPoint;
            ROS_DEBUG_STREAM("Do frustum culling: ActiveIndices="<< mCalculator.getActiveIndices()->size());
			mCalculator.doFrustumCulling(point, orientation, mCalculator.getActiveIndices(), viewportPoint);
            ROS_DEBUG_STREAM("Do update object point cloud");
			mCalculator.updateObjectPointCloud(viewportPoint);

			return true;
		}
        bool processTriggerFrustumVisualization(TriggerFrustumVisualization::Request &request,
                                                TriggerFrustumVisualization::Response &response)
        {
            ROS_DEBUG("trigger frustum visualization");
            geometry_msgs::Pose pose = request.current_pose;
            SimpleVector3 position = TypeHelper::getSimpleVector3(pose);
            SimpleQuaternion orientation = TypeHelper::getSimpleQuaternion(pose);
            mCalculator.getCameraModelFilter()->setPivotPointPose(position, orientation);
            uint32_t sequence = 0;
	    if (mMarkerArrayPtr)
	      deleteFrustumInRviz();
	    mMarkerArrayPtr = this->mCalculator.getCameraModelFilter()->getVisualizationMarkerArray(sequence, 0.0);
            
	    ROS_DEBUG("Publish actual frustum");
            for (unsigned int i = 0; i < mMarkerArrayPtr->markers.size(); i++)
            {
                    mMarkerArrayPtr->markers.at(i).color.r = 0;
                    mMarkerArrayPtr->markers.at(i).color.g = 1;
                    mMarkerArrayPtr->markers.at(i).color.b = 1;
                    mMarkerArrayPtr->markers.at(i).ns = "actual_nbv_frustum";
             }
             std::string result = "searched objects: " + boost::lexical_cast<std::string>(numberSearchedObjects);
             viz::Marker textMarker = MarkerHelper::getTextMarker(mMarkerArrayPtr->markers.size(), result);
             textMarker.ns = "searched_obj";
             textMarker.pose = pose;
             mMarkerArrayPtr->markers.push_back(textMarker);

             mFrustumMarkerArrayPublisher.publish(*mMarkerArrayPtr);
            return true;
        }

        bool processTriggerOldFrustumVisualization(TriggerFrustumVisualization::Request &request,
                                                TriggerFrustumVisualization::Response &response)
        {
            ROS_DEBUG("trigger frustum visualization");
            geometry_msgs::Pose pose = request.current_pose;
            SimpleVector3 position = TypeHelper::getSimpleVector3(pose);
            SimpleQuaternion orientation = TypeHelper::getSimpleQuaternion(pose);
            mCalculator.getCameraModelFilter()->setPivotPointPose(position, orientation);
            uint32_t sequence = 0;
            viz::MarkerArray::Ptr markerArrayPtr;
            markerArrayPtr = this->mCalculator.getCameraModelFilter()->getVisualizationMarkerArray(sequence, 0.0);
                ROS_DEBUG("Publish previous frustum");
                for (unsigned int i = 0; i < markerArrayPtr->markers.size(); i++)
                {
                    markerArrayPtr->markers.at(i).color.r = 1;
                    markerArrayPtr->markers.at(i).color.g = 0;
                    markerArrayPtr->markers.at(i).color.b = 1;
                    markerArrayPtr->markers.at(i).lifetime = ros::Duration(4.0);
                    markerArrayPtr->markers.at(i).ns = "previous_nbv_frustum";
                }
                //std::string result = "searched objects: " + boost::lexical_cast<std::string>(numberSearchedObjects);
                //viz::Marker textMarker = MarkerHelper::getTextMarker(mMarkerArrayPtr->markers.size(), result);
                //textMarker.pose = pose;
                //mMarkerArrayPtr->markers.push_back(textMarker);

                mFrustumMarkerArrayPublisher.publish(*markerArrayPtr);
            return true;
        }

		bool triggerVisualization() {
			return this->triggerVisualization(mCurrentCameraViewport, false);
		}

		bool triggerVisualization(ViewportPoint viewport) {
			return this->triggerVisualization(viewport, false);
		}

		bool triggerVisualization(ViewportPoint viewport, bool is_initial) {
			if (mCurrentlyPublishingVisualization) {
				ROS_DEBUG("Currently generating visualization data.");
				return false;
			}

			mCurrentlyPublishingVisualization = true;

            boost::thread t = boost::thread(&NextBestView::publishVisualization, this, viewport, is_initial, true);
            //publishVisualization(viewport,is_initial);
			return true;
		}
        void deleteFrustumInRviz()
        {
            ROS_DEBUG("Deleted last frustum in rviz");
            for (unsigned int i = 0; i < mMarkerArrayPtr->markers.size(); i++)
            {
                mMarkerArrayPtr->markers.at(i).action = visualization_msgs::Marker::DELETE;
            }
            mFrustumMarkerArrayPublisher.publish(*mMarkerArrayPtr);
        }

        void deleteMarkerArray(viz::MarkerArray::Ptr array,ros::Publisher publisher )
        {
            ROS_DEBUG("Deleted last frustum in rviz");
            for (unsigned int i = 0; i < array->markers.size(); i++)
            {
                array->markers.at(i).action = visualization_msgs::Marker::DELETE;
            }
            publisher.publish(*array);
            array->markers.clear();
        }

        /**
        * \brief Generate Rviz marker messages from object messages.
        *
        * \param object The Object message.
        */
        visualization_msgs::Marker createObjectMarker(RealObjectPoint objectPoints,unsigned int i, std_msgs::ColorRGBA color, std::string ns) {
            visualization_msgs::Marker marker;
            // Set the frame ID and timestamp.  See the TF tutorials for information on these.
            marker.header.stamp = ros::Time();
            marker.header.frame_id = "/map";
            marker.id = i;

            // Set the namespace and id for this marker.  This serves to create a unique ID
            // Any marker sent with the same namespace and id will overwrite the old one
            marker.ns = ns;

            if (mCalculator.getMeshPathByName(objectPoints.object_type_name) == "-2")
            {
                marker.type = visualization_msgs::Marker::SPHERE;
                // Set the scale of the marker -- 1x1x1 here means 1m on a side
                marker.scale.x =  marker.scale.y = marker.scale.z = 0.01;

            }
            else
            {
                marker.type = visualization_msgs::Marker::MESH_RESOURCE;
                marker.mesh_use_embedded_materials = true;
                // Cut of .iv, append .dae
                fs::path mesh_resource = fs::path(mCalculator.getMeshPathByName(objectPoints.object_type_name)).replace_extension(".dae");
                marker.mesh_resource = mesh_resource.string();
                // the model size unit is mm
                marker.scale.x =  marker.scale.y = marker.scale.z = 0.0005;
            }
            marker.color = color;
            // Set the marker action.  Options are ADD and DELETE
            marker.action = visualization_msgs::Marker::ADD;

            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            marker.pose = objectPoints.getPose();

            return marker;
        }


		/*!
		 * \brief Publishes the Visualization of the NextBestView
		 * \param robot_pose, the new pose of the robot
		 * \param is_initial, marks if the given robot pose was initial
		 */
        void publishVisualization(ViewportPoint viewport, bool is_initial, bool publishFrustum) {
			ROS_DEBUG("Publishing Visualization");
            ROS_DEBUG_STREAM("Frustum Pivot Point : " << this->mCalculator.getCameraModelFilter()->getPivotPointPosition()[0] <<
                             " , " <<  this->mCalculator.getCameraModelFilter()->getPivotPointPosition()[1]
                             << " , " << this->mCalculator.getCameraModelFilter()->getPivotPointPosition()[2]);

			// If the visualization of the robot movement is wished, execute this block
			// TODO: Programmcode is throwing "Updating ModelState: model [mild] does not exist", probably mild.dae missing - don't know
            /*if (mVisualizationSettings.move_robot) {

                MILDRobotStatePtr robot_pose = boost::static_pointer_cast<MILDRobotState>(mCalculator.getRobotModel()->getCurrentRobotState());

                geometry_msgs::Pose movePose;
                movePose.orientation = tf::createQuaternionMsgFromYaw(robot_pose->rotation);
                movePose.position.x = robot_pose->x;
                movePose.position.y = robot_pose->y;

                ROS_DEBUG_STREAM("We wqnt to go to pose :" << movePose);
				if (is_initial) {
					ROS_DEBUG("Initializing the Robot Position");

					this->setInitialRobotPose(movePose);
				} else {
					ROS_DEBUG("Move the Robot to new Pose");

					this->moveRobotToPose(movePose);
				}
            }*/
            if (mVisualizationSettings.point_cloud)
            {
				ROS_DEBUG("Publishing Point Cloud");

				Indices pointCloudIndices;
				if (mVisualizationSettings.frustum_point_cloud) {
					point_cloud_indices_diff(mCalculator.getActiveIndices(), viewport.child_indices, pointCloudIndices);
                    ROS_DEBUG_STREAM("viewport.child_indices Size: " << viewport.child_indices->size());
				} else {
					pointCloudIndices = *mCalculator.getActiveIndices();
				}
                ROS_DEBUG_STREAM("Publishing "<< pointCloudIndices.size() <<" points");


                ObjectPointCloud objectPointCloud = ObjectPointCloud(*mCalculator.getPointCloudPtr(), pointCloudIndices);
                /*sensor_msgs::PointCloud2 point_cloud;
                pcl::toROSMsg(objectPointCloud, point_cloud);

				point_cloud.header.frame_id = "/map";
				point_cloud.header.seq = 0;

				sensor_msgs::PointField rgb;
				rgb.name = "rgb";
				rgb.datatype = sensor_msgs::PointField::UINT32;
				rgb.offset = offsetof(ObjectPoint, rgb);
				point_cloud.fields.push_back(rgb);

                mPointCloudPublisher.publish(point_cloud);*/

                this->deleteMarkerArray(mMeshMarkerArrayPtr, mObjectMeshMarkerPublisher);
                std_msgs::ColorRGBA colorMeshMarker;
                colorMeshMarker.a = 0;
                colorMeshMarker.r = 0;
                colorMeshMarker.b = 0;
                colorMeshMarker.g = 0;
                unsigned int index = 0;
                for(ObjectPointCloud::iterator it = objectPointCloud.begin(); it < objectPointCloud.end(); it++)
                {
                    visualization_msgs::Marker niceMarker = createObjectMarker((*it),index++,colorMeshMarker,"mMeshMarkerArray");
                    mMeshMarkerArrayPtr->markers.push_back(niceMarker);
                }
                mObjectMeshMarkerPublisher.publish(*mMeshMarkerArrayPtr);

                index = 0;
                this->deleteMarkerArray(mObjectNormalsMarkerArrayPtr, mPointObjectNormalPublisher);
                for(unsigned int i = 0; i < objectPointCloud.points.size(); i++)
                {
                    ObjectPoint point = objectPointCloud.points[i];
                    for(Indices::iterator it = point.active_normal_vectors->begin(); it < point.active_normal_vectors->end(); ++it)
                    {
                        geometry_msgs::Point start = point.getPoint();
                        geometry_msgs::Point end = TypeHelper::getPointMSG(0.07*point.normal_vectors->at(*it));
                        end.x += start.x;
                        end.y += start.y;
                        end.z += start.z;
                        visualization_msgs::Marker objectNormalMarker;

                        objectNormalMarker.header.stamp = ros::Time();
                        objectNormalMarker.header.frame_id = "/map";
                        objectNormalMarker.type = objectNormalMarker.ARROW;
                        objectNormalMarker.action = objectNormalMarker.ADD;
                        objectNormalMarker.id = index++;
                        objectNormalMarker.lifetime = ros::Duration();
                        objectNormalMarker.ns = "ObjectNormals";
                        objectNormalMarker.scale.x = 0.005;
                        objectNormalMarker.scale.y = 0.01;
                        objectNormalMarker.scale.z = 0.005;
                        objectNormalMarker.color.a = 1;
                        objectNormalMarker.color.r = 1;
                        objectNormalMarker.color.g = 1;
                        objectNormalMarker.color.b = 0;
                        objectNormalMarker.points.push_back(start);
                        objectNormalMarker.points.push_back(end);
                        objectNormalMarker.pose.orientation.w=1;

                        mObjectNormalsMarkerArrayPtr->markers.push_back(objectNormalMarker);
                    }
                }
                mPointObjectNormalPublisher.publish(*mObjectNormalsMarkerArrayPtr);
			}
            if (mVisualizationSettings.frustum_point_cloud)
            {
                ROS_DEBUG("Publishing Frustum Marker Array");
                //sensor_msgs::PointCloud2 frustum_point_cloud;
                //pcl::toROSMsg(ObjectPointCloud(*mCalculator.getPointCloudPtr(), *viewport.child_indices), frustum_point_cloud);
                std_msgs::ColorRGBA colorFrustumMeshMarker;
                colorFrustumMeshMarker.a = 0.8;
                colorFrustumMeshMarker.r = 0;
                colorFrustumMeshMarker.b = 1;
                colorFrustumMeshMarker.g = 0;
                ROS_DEBUG("Creating frustumObjectPointCloud");
                ROS_DEBUG_STREAM("viewport.child_indices Size bis: " << viewport.child_indices->size());
                ObjectPointCloud frustumObjectPointCloud = ObjectPointCloud(*mCalculator.getPointCloudPtr(), *viewport.child_indices);
                ROS_DEBUG_STREAM("viewport.child_indices Size bis : " << viewport.child_indices->size());
                ROS_DEBUG("Deleting array...");
                this->deleteMarkerArray(mFrustumMeshMarkerArrayPtr, mFrustumObjectMeshMarkerPublisher);
                unsigned int index = 0;
                for(ObjectPointCloud::iterator it = frustumObjectPointCloud.begin(); it < frustumObjectPointCloud.end(); it++)
                {
                    visualization_msgs::Marker niceMarker = createObjectMarker((*it),index++,colorFrustumMeshMarker,"mFrustumObjectMesh");
                    mFrustumMeshMarkerArrayPtr->markers.push_back(niceMarker);
                }
                mFrustumObjectMeshMarkerPublisher.publish(*mFrustumMeshMarkerArrayPtr);

                /*frustum_point_cloud.header.frame_id = "/map";
				frustum_point_cloud.header.seq = 0;

                mFrustumPointCloudPublisher.publish(frustum_point_cloud);*/
			}
            if (mVisualizationSettings.frustum_marker_array && publishFrustum)
            {
                viz::MarkerArray::Ptr TempMarkerArray(new viz::MarkerArray);
                uint32_t sequence = 0;
                if (mMarkerArrayPtr)
                {
                    deleteFrustumInRviz();
                    ROS_DEBUG("Publishing old frustum");
                    for (unsigned int i = 0; i < mMarkerArrayPtr->markers.size(); i++)
                    {
                        mMarkerArrayPtr->markers.at(i).lifetime = ros::Duration(4.0);
                        mMarkerArrayPtr->markers.at(i).action = visualization_msgs::Marker::ADD;
                        mMarkerArrayPtr->markers.at(i).id +=  mMarkerArrayPtr->markers.size();
                        mMarkerArrayPtr->markers.at(i).color.r = 1;
                        mMarkerArrayPtr->markers.at(i).color.g = 0;
                        mMarkerArrayPtr->markers.at(i).color.b = 1;
                        mMarkerArrayPtr->markers.at(i).ns = "old_nbv_frustum";
                        TempMarkerArray->markers.push_back(mMarkerArrayPtr->markers.at(i));
                    }
                    //mFrustumMarkerArrayPublisher.publish(*mMarkerArrayPtr);

                }
                ROS_DEBUG_STREAM("Frustum Pivot Point : " << this->mCalculator.getCameraModelFilter()->getPivotPointPosition()[0] <<
                                 " , " <<  this->mCalculator.getCameraModelFilter()->getPivotPointPosition()[1]
                                 << " , " << this->mCalculator.getCameraModelFilter()->getPivotPointPosition()[2]);

                mMarkerArrayPtr = this->mCalculator.getCameraModelFilter()->getVisualizationMarkerArray(sequence, 0.0);
                ROS_DEBUG("Publishing new frustum");
                for (unsigned int i = 0; i < mMarkerArrayPtr->markers.size(); i++)
                {
                    mMarkerArrayPtr->markers.at(i).color.r = 0;
                    mMarkerArrayPtr->markers.at(i).color.g = 1;
                    mMarkerArrayPtr->markers.at(i).color.b = 1;
                    mMarkerArrayPtr->markers.at(i).ns = "new_nbv_frustum";
                }
                if (!is_initial)
                {
                    ROS_DEBUG("Adding text");
                    numberSearchedObjects = viewport.object_name_set->size();
                    ROS_INFO_STREAM("numberSearchedObjects: " <<  numberSearchedObjects);
                    std::string result = "searched objects: " + boost::lexical_cast<std::string>(numberSearchedObjects);
                    viz::Marker textMarker = MarkerHelper::getTextMarker(mMarkerArrayPtr->markers.size(), result);
                    textMarker.pose = viewport.getPose();
                    textMarker.ns = "new_nbv_frustum_text";
                    mMarkerArrayPtr->markers.push_back(textMarker);
                }
                if (TempMarkerArray) mFrustumMarkerArrayPublisher.publish(*TempMarkerArray);
                mFrustumMarkerArrayPublisher.publish(*mMarkerArrayPtr);
            }
			mCurrentlyPublishingVisualization = false;
		}

		/*!
		 * \brief Sets the Initial Pose of the Robot to the given param value without any uncertainty.
		 * \param initialPose the initial pose of the Robot
		 */
		void setInitialRobotPose(const geometry_msgs::Pose &initialPose) {
			// waiting for buffers to fill
			ros::Duration(5.0).sleep();

			geometry_msgs::PoseWithCovarianceStamped pose;
			pose.header.frame_id = "map";
			boost::array<double, 36> a =  {
					// x, y, z, roll, pitch, yaw
					0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 0.0, 0.0, 0.0
			};
			pose.pose.covariance = a;
			pose.pose.pose = initialPose;

			mInitialPosePublisher.publish(pose);
		}

		void moveRobotToPose(const geometry_msgs::Pose &pose) {
			double interval_time = 5.0;
			while(!mMoveBaseActionClient->waitForServer(ros::Duration(interval_time))) {
                ROS_DEBUG_STREAM("Waiting for the move_base action server to come up. Checking Interval:" << interval_time <<" seconds" );
			}

			move_base_msgs::MoveBaseGoal goal;
			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now();

			goal.target_pose.pose = pose;

			mMoveBaseActionClient->sendGoal(goal);

			mMoveBaseActionClient->waitForResult();
			if(mMoveBaseActionClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
				ROS_DEBUG("Hooray, the base moved");
			} else {
				ROS_ERROR("The base failed to move");
			}
		}

		static void point_cloud_indices_diff(const IndicesPtr &AVect, const IndicesPtr &BVect, Indices &resultIndices) {
			std::set<int> A(AVect->begin(), AVect->end());
			std::set<int> B(BVect->begin(), BVect->end());

			std::set_difference( A.begin(), A.end(), B.begin(), B.end(),
					std::back_inserter( resultIndices ) );
		}
	};
}


#endif /* NEXTBESTVIEW_HPP_ */
