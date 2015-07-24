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
#include <boost/foreach.hpp>
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
#include <object_database/ObjectManager.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

// Local Includes
#include "typedef.hpp"
#include "next_best_view/SetupVisualization.h"
#include "next_best_view/GetAttributedPointCloud.h"
#include "next_best_view/GetPointCloud2.h"
#include "next_best_view/GetNextBestView.h"
#include "next_best_view/GetSpaceSampling.h"
#include "next_best_view/SetAttributedPointCloud.h"
#include "next_best_view/UpdatePointCloud.h"
#include "next_best_view/helper/MapHelper.hpp"
#include "next_best_view/NextBestViewCalculator.hpp"
#include "next_best_view/helper/MarkerHelper.hpp"
#include "next_best_view/helper/MathHelper.hpp"
#include "next_best_view/AttributedPointCloud.h"
#include "next_best_view/AttributedPoint.h"
#include "next_best_view/camera_model_filter/impl/MapBasedSingleCameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/impl/SingleCameraModelFilter.hpp"
#include "next_best_view/hypothesis_updater/impl/PerspectiveHypothesisUpdater.hpp"
#include "next_best_view/robot_model/impl/MILDRobotModel.hpp"
#include "next_best_view/robot_model/impl/MILDRobotState.hpp"
#include "next_best_view/unit_sphere_sampler/impl/SpiralApproxUnitSphereSampler.hpp"
#include "next_best_view/space_sampler/impl/PlaneSubSpaceSampler.hpp"
#include "next_best_view/space_sampler/impl/MapBasedSpaceSampler.hpp"
#include "next_best_view/space_sampler/impl/MapBasedHexagonSpaceSampler.hpp"
#include "next_best_view/rating/impl/DefaultRatingModule.hpp"


namespace next_best_view {
	// Defining shorthandle for action client.
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseActionClient;
	typedef boost::shared_ptr<MoveBaseActionClient> MoveBaseActionClientPtr;

	// Defining namespace shorthandles
	namespace fs = boost::filesystem;
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

		ros::Publisher mSpaceSamplingPublisher;
		ros::Publisher mPointCloudPublisher;
		ros::Publisher mFrustumPointCloudPublisher;
		ros::Publisher mUnitSpherPointCloudPublisher;
		ros::Publisher mFrustumMarkerArrayPublisher;
		ros::Publisher mInitialPosePublisher;

		// ServiceClients and Subscriber
		ros::ServiceClient mObjectTypeServiceClient;

		// Action Clients
		MoveBaseActionClientPtr mMoveBaseActionClient;

		// Node-Wide Variables.

		ViewportPoint mInitialCameraViewport;
		ObjectPointCloudPtr mPointCloudPtr;
		KdTreePtr mKdTreePtr;
		SetupVisualizationRequest mVisualizationSettings;
	public:
		/*!
		 * \brief Creates an instance of the NextBestView class.
		 */
		NextBestView() : mGlobalNodeHandle(), mNodeHandle(ros::this_node::getName()), mPointCloudPtr(new ObjectPointCloud()) {
			mGetPointCloud2ServiceServer = mNodeHandle.advertiseService("get_point_cloud2", &NextBestView::processGetPointCloud2ServiceCall, this);
			mGetPointCloudServiceServer = mNodeHandle.advertiseService("get_point_cloud", &NextBestView::processGetPointCloudServiceCall, this);
			mGetNextBestViewServiceServer = mNodeHandle.advertiseService("next_best_view", &NextBestView::processGetNextBestViewServiceCall, this);
			mSetPointCloudServiceServer = mNodeHandle.advertiseService("set_point_cloud", &NextBestView::processSetPointCloudServiceCall, this);
			mUpdatePointCloudServiceServer = mNodeHandle.advertiseService("update_point_cloud", &NextBestView::processUpdatePointCloudServiceCall, this);
			mGetSpaceSamplingServiceServer = mNodeHandle.advertiseService("get_space_sampling", &NextBestView::processGetSpaceSamplingServiceCall, this);
			mSetupVisualizationServiceServer = mNodeHandle.advertiseService("setup_visualization", &NextBestView::processSetupVisualizationServiceCall, this);

			mSpaceSamplingPublisher = mNodeHandle.advertise<sensor_msgs::PointCloud2>("space_sampling_point_cloud", 100);
			mPointCloudPublisher = mNodeHandle.advertise<sensor_msgs::PointCloud2>("point_cloud", 1000);
			mFrustumPointCloudPublisher = mNodeHandle.advertise<sensor_msgs::PointCloud2>("frustum_point_cloud", 1000);
			mFrustumMarkerArrayPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>("frustum_marker_array", 1000);
			mInitialPosePublisher = mNodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 100, false);

			mObjectTypeServiceClient = mGlobalNodeHandle.serviceClient<odb::ObjectType>("/object_database/object_type");

			mMoveBaseActionClient = MoveBaseActionClientPtr(new MoveBaseActionClient("move_base", true));

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

			/* These are the parameters for the CameraModelFilter. By now they will be kept in here, but for the future they'd better
			* be defined in the CameraModelFilter specialization class.
			* TODO: Export these parameters to the specialization of the CameraModelFilter class. This makes sense, because you have to
			* keep in mind that there are stereo cameras which might have slightly different settings of the frustums. So we will be
			* able to adjust the parameters for each camera separateley.
			*/
			double fovx, fovy, ncp, fcp;
			mNodeHandle.param("fovx", fovx, 62.5);
			mNodeHandle.param("fovy", fovy, 48.9);
			mNodeHandle.param("ncp", ncp, .5);
			mNodeHandle.param("fcp", fcp, 5.0);

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
			 * Therefore we used an approximation algorithm which picks approximates the surface with a
			 * projection of a spiral on the sphere's surface. Resulting in this wonderful sounding name.
			 */
			SpiralApproxUnitSphereSamplerPtr unitSphereSamplerPtr(new SpiralApproxUnitSphereSampler());
			unitSphereSamplerPtr->setSamples(128);

			/* MapHelper does get the maps on which we are working on and modifies them for use with applications like raytracing and others.
			 * TODO: The maps may have areas which are marked feasible but in fact are not, because of different reasons. The main
			 * reason may be that the map may contain areas where the robot cannot fit through and therefore cannot move to the
			 * wanted position. You have to consider if there is any possibility to mark these areas as non-feasible.
			 */
			MapHelperPtr mapHelperPtr(new MapHelper());
			mapHelperPtr->setCollisionThreshold(45);

			/* MapBasedHexagonSpaceSampler is a specialization of the abstract SpaceSampler class.
			 * By space we denote the area in which the robot is moving. In our case there are just two degrees of freedom
			 * in which the robot can move, namely the xy-plane. But we do also have a map on which we can base our sampling on.
			 * There are a lot of ways to sample the xy-plane into points but we decided to use a hexagonal grid which we lay over
			 * the map and calculate the points which are contained in the feasible map space.
			 */
			MapBasedHexagonSpaceSamplerPtr spaceSamplerPtr(new MapBasedHexagonSpaceSampler(mapHelperPtr));
			spaceSamplerPtr->setHexagonRadius(0.75);

			/* MapBasedSingleCameraModelFilterPtr is a specialization of the abstract CameraModelFilter class.
			 * The camera model filter takes account for the fact, that there are different cameras around in the real world.
			 * In respect of the parameters of these cameras the point cloud gets filtered by theses camera filters. Plus
			 * the map-based version of the camera filter also uses the knowledge of obstacles or walls between the camera and
			 * the object to be observed. So, map-based in this context means that the way from the lens to the object is ray-traced.
			 */
			MapBasedSingleCameraModelFilterPtr cameraModelFilterPtr(new MapBasedSingleCameraModelFilter(mapHelperPtr, SimpleVector3(0.0, 0.0, 0.1)));
			cameraModelFilterPtr->setHorizontalFOV(fovx);
			cameraModelFilterPtr->setVerticalFOV(fovy);
			cameraModelFilterPtr->setNearClippingPlane(ncp);
			cameraModelFilterPtr->setFarClippingPlane(fcp);

			/* MILDRobotModel is a specialization of the abstract RobotModel class.
			 * The robot model maps takes the limitations of the used robot into account and by this it is possible to filter out
			 * non-reachable configurations of the robot which can therefore be ignored during calculation.
			 */
			MILDRobotModelPtr robotModelPtr(new MILDRobotModel());
			robotModelPtr->setTiltAngleLimits(-45, 45);
			robotModelPtr->setPanAngleLimits(-60, 60);

			/* DefaultRatingModule is a specialization of the abstract RatingModule class.
			 * The rating module calculates the use and the costs of an operation.
			 */
			DefaultRatingModulePtr ratingModulePtr(new DefaultRatingModule());
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
		}

		//dtor
		virtual ~NextBestView() { }

		bool processSetupVisualizationServiceCall(SetupVisualizationRequest &request, SetupVisualizationResponse &response) {
			mVisualizationSettings = SetupVisualizationRequest(request);

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


		static void convertObjectPointCloudToAttributedPointCloud(const ObjectPointCloud &pointCloud, AttributedPointCloud &pointCloudMessage) {
			pointCloudMessage.elements.clear();

			BOOST_FOREACH(ObjectPoint point, pointCloud) {
				AttributedPoint aPoint;

				aPoint.pose = point.getPose();
				aPoint.object_type = point.object_type_name;

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
			mCalculator.setPointCloudFromMessage(request.point_cloud);

			ViewportPoint initialCameraViewport(request.pose);
			mInitialCameraViewport = initialCameraViewport;
			mCalculator.getCameraModelFilter()->setOrientation(initialCameraViewport.getSimpleQuaternion());
			mCalculator.getCameraModelFilter()->setPivotPointPosition(initialCameraViewport.getSimpleVector3());

			MILDRobotStatePtr currentRobotStatePtr(new MILDRobotState());
			currentRobotStatePtr->pan = 0;
			currentRobotStatePtr->tilt = 0;
			currentRobotStatePtr->rotation = 0;
			currentRobotStatePtr->x = initialCameraViewport.x;
			currentRobotStatePtr->y = initialCameraViewport.y;
			mCalculator.getRobotModel()->setCurrentRobotState(currentRobotStatePtr);

			response.is_valid = true;

			// publish the visualization
			this->publishVisualization(request.pose, true);

			return true;
		}

	  //COMMENT?
		bool processGetNextBestViewServiceCall(GetNextBestView::Request &request, GetNextBestView::Response &response) {
			ViewportPoint currentCameraViewport(request.initial_pose);

			ViewportPoint resultingViewport;
			if (!mCalculator.calculateNextBestView(currentCameraViewport, resultingViewport)) {
				ROS_INFO("No more found");
				response.found = false;
				return true;
			}
			response.found = true;
			response.resulting_pose = resultingViewport.getPose();

			SimpleVector3 position = TypeHelper::getSimpleVector3(response.resulting_pose);
			SimpleQuaternion orientation = TypeHelper::getSimpleQuaternion(response.resulting_pose);
			mCalculator.getCameraModelFilter()->setPivotPointPose(position, orientation);

			uint32_t seq = 0;
			response.frustum_marker_array = *mCalculator.getCameraModelFilter()->getVisualizationMarkerArray(seq, 0.0);

			pcl::toROSMsg(ObjectPointCloud(*mCalculator.getPointCloudPtr(), *resultingViewport.child_indices), response.frustum_point_cloud);
			response.frustum_point_cloud.header.frame_id ="/map";

			Indices indices;
			for (int index = 0; index < mCalculator.getPointCloudPtr()->size(); index++) {
				ObjectPoint &objectPoint = mCalculator.getPointCloudPtr()->at(index);

				if (objectPoint.active_normal_vectors->size() == 0) {
					continue;
				}

				indices.push_back(index);
			}

			Indices resultIndices;
			vector_diff(indices, *resultingViewport.child_indices, resultIndices);

			pcl::toROSMsg(ObjectPointCloud(*mCalculator.getPointCloudPtr(), resultIndices), response.point_cloud);
			response.point_cloud.header.frame_id = "/map";

			SimpleQuaternionCollectionPtr collectionPtr = mCalculator.getUnitSphereSampler()->getSampledUnitSphere();
			ObjectPointCloud opc;
			BOOST_FOREACH(SimpleQuaternion orientation, *collectionPtr) {
				if (!mCalculator.getRobotModel()->isPoseReachable(SimpleVector3(0, 0, 0), orientation)) {
					continue;
				}
				SimpleVector3 visualAxis = MathHelper::getVisualAxis(orientation);
				ObjectPoint point(visualAxis);
				opc.push_back(point);
			}
			pcl::toROSMsg(opc, response.unit_sphere_sampling_point_cloud);
			response.unit_sphere_sampling_point_cloud.header.frame_id = "/map";

			sensor_msgs::PointField rgb;
			rgb.name = "rgb";
			rgb.datatype = sensor_msgs::PointField::UINT32;
			rgb.offset = offsetof(ObjectPoint, rgb);
			response.point_cloud.fields.push_back(rgb);

			mCalculator.updateObjectPointCloud(resultingViewport);

			this->publishVisualization(resultingViewport.getPose());

			return true;
		}

		bool processUpdatePointCloudServiceCall(UpdatePointCloud::Request &request, UpdatePointCloud::Response &response) {
			SimpleVector3 point = TypeHelper::getSimpleVector3(request.update_pose);
			SimpleQuaternion orientation = TypeHelper::getSimpleQuaternion(request.update_pose);
			ViewportPoint viewportPoint;
			mCalculator.doFrustumCulling(point, orientation, IndicesPtr(), viewportPoint);

			mCalculator.updateObjectPointCloud(viewportPoint);

			return true;
		}

		/*!
		 * \brief Passes the request to the other publishVisualization with is_initial = false by default.
		 */
		void publishVisualization(geometry_msgs::Pose robot_pose) {
			this->publishVisualization(robot_pose, false);
		}

		/*!
		 * \brief Publishes the Visualization of the NextBestView
		 * \param robot_pose, the new pose of the robot
		 * \param is_initial, marks if the given robot pose was initial
		 */
		void publishVisualization(geometry_msgs::Pose robot_pose, bool is_initial) {
			ROS_INFO("Publishing Visualization");

			// If the visualization of the robot movement is wished, execute this block
			if (mVisualizationSettings.move_robot) {
				double yaw = tf::getYaw(robot_pose.orientation);

				geometry_msgs::Pose movePose(robot_pose);
				movePose.orientation = tf::createQuaternionMsgFromYaw(yaw);

				if (is_initial) {
					ROS_INFO("Initializing the Robot Position");

					this->setInitialRobotPose(movePose);
				} else {
					ROS_INFO("Move the Robot to new Pose");

					this->moveRobotToPose(movePose);
				}
			}

			if (mVisualizationSettings.space_sampling) {
				ROS_INFO("Publishing Space Sampling");

			}

			if (mVisualizationSettings.point_cloud) {
				ROS_INFO("Publishing Point Cloud");
				GetPointCloud2 gpc2ServiceCall;
				this->processGetPointCloud2ServiceCall(gpc2ServiceCall.request, gpc2ServiceCall.response);
				mPointCloudPublisher.publish(gpc2ServiceCall.response.point_cloud);
			}

			if (mVisualizationSettings.frustum_point_cloud) {
				ROS_INFO("Publishing Frustum Point Cloud");

			}

			if (mVisualizationSettings.frustum_marker_array) {
				ROS_INFO("Publishing Frustum Marker Array");
			}
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
				ROS_INFO("Waiting for the move_base action server to come up. Checking Interval: %f seconds", interval_time);
			}

			move_base_msgs::MoveBaseGoal goal;
			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now();

			goal.target_pose.pose = pose;

			mMoveBaseActionClient->sendGoal(goal);

			mMoveBaseActionClient->waitForResult();
			if(mMoveBaseActionClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
				ROS_INFO("Hooray, the base moved");
			} else {
				ROS_ERROR("The base failed to move");
			}
		}

		static void vector_diff(const Indices &AVect, const Indices &BVect, Indices &resultIndices) {
			std::set<int> A(AVect.begin(), AVect.end());
			std::set<int> B(BVect.begin(), BVect.end());

			std::set_difference( A.begin(), A.end(), B.begin(), B.end(),
					std::back_inserter( resultIndices ) );
		}
	};
}


#endif /* NEXTBESTVIEW_HPP_ */
