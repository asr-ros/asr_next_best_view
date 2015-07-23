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
#include "next_best_view/SetVisualization.h"
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
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

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
		ros::ServiceServer mGetPointCloud2ServiceClient;
		ros::ServiceServer mGetPointCloudServiceClient;
		ros::ServiceServer mSetPointCloudServiceClient;
		ros::ServiceServer mGetNextBestViewServiceClient;
		ros::ServiceServer mGetSpaceSamplingServer;
		ros::ServiceServer mUpdatePointCloud;
		ros::Publisher mSpaceSamplingPublisher;
		ros::Publisher mPointCloudPublisher;
		ros::Publisher mFrustumPointCloudPublisher;
		ros::Publisher mUnitSpherPointCloudPublisher;
		ros::Publisher mFrustumMarkerArrayPublisher;

		// ServiceClients and Subscriber
		ros::ServiceClient mObjectTypeServiceClient;

		// Node-Wide Variables.
		ViewportPoint mInitialCameraViewport;
		ObjectPointCloudPtr mPointCloudPtr;
		KdTreePtr mKdTreePtr;
		SetVisualization::Request mLastVisualizationSettings;
	public:
		/*!
		 * \brief Creates an instance of the NextBestView class.
		 */
		NextBestView() : mGlobalNodeHandle(), mNodeHandle(ros::this_node::getName()), mPointCloudPtr(new ObjectPointCloud()) {
			mGetPointCloud2ServiceClient = mNodeHandle.advertiseService("get_point_cloud2", &NextBestView::processGetPointCloud2ServiceCall, this);
			mGetPointCloudServiceClient = mNodeHandle.advertiseService("get_point_cloud", &NextBestView::processGetPointCloudServiceCall, this);
			mGetNextBestViewServiceClient = mNodeHandle.advertiseService("next_best_view", &NextBestView::processGetNextBestViewServiceCall, this);
			mSetPointCloudServiceClient = mNodeHandle.advertiseService("set_point_cloud", &NextBestView::processSetPointCloudServiceCall, this);
			mUpdatePointCloud = mNodeHandle.advertiseService("update_point_cloud", &NextBestView::processUpdatePointCloudServiceCall, this);
			mGetSpaceSamplingServer = mNodeHandle.advertiseService("get_space_sampling", &NextBestView::processGetSpaceSamplingServiceCall, this);
			mObjectTypeServiceClient = mGlobalNodeHandle.serviceClient<odb::ObjectType>("/object_database/object_type");

			mSpaceSamplingPublisher = mNodeHandle.advertise<sensor_msgs::PointCloud2>("space_sampling_point_cloud", 100);
			mPointCloudPublisher = mNodeHandle.advertise<sensor_msgs::PointCloud2>("point_cloud", 1000);
			mFrustumPointCloudPublisher = mNodeHandle.advertise<sensor_msgs::PointCloud2>("frustum_point_cloud", 1000);
			mFrustumMarkerArrayPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>("frustum_marker_array", 1000);

			// set the visualization defaults
			//mNodeHandle.param("visualize_space_sampling_point_cloud", mLastVisualizationSettings.visualize_space_sampling_point_cloud, false);
			//mNodeHandle.param("visualize_point_cloud", mLastVisualizationSettings.visualize_point_cloud, false);
			//mNodeHandle.param("visualize_frustum_point_cloud", mLastVisualizationSettings.visualize_frustum_point_cloud, false);
			bool viz_frustum_marker_array;
			mNodeHandle.param("visualize_frustum_marker_array", viz_frustum_marker_array, false);

			mLastVisualizationSettings.visualize_frustum_marker_array = viz_frustum_marker_array;

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

			ROS_INFO("FOVX: %f, FOVY: %f, NCP: %f, FCP: %f", fovx, fovy, ncp, fcp);
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

		bool processSetVisualizationServiceCall(SetVisualization::Request &request, SetVisualization::Response) {

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

			return true;
//			// TODO wrap into PointCloudMessage.
//			ViewportPoint initialCameraViewport;
//			initialCameraViewport.x = 22.15;
//			initialCameraViewport.y = 10.55;
//			initialCameraViewport.z = 1.32;
//			initialCameraViewport.qw = 1.0;
//			initialCameraViewport.qx = 0.0;
//			initialCameraViewport.qy = 0.0;
//			initialCameraViewport.qz = 0.0;
//
//
//			ViewportPointCloudPtr viewportPointCloudPtr(new ViewportPointCloud());
//			ViewportPoint currentCameraViewport = initialCameraViewport;
//			uint32_t ccount = 0;
//			while(ros::ok()) {
//				ViewportPoint viewportPoint;
//				if (!calculator.calculateNextBestView(currentCameraViewport, viewportPoint)) {
//					break;
//				}
//				ccount++;
//				ROS_INFO("Iteration Count: %d", ccount);
//				calculator.updateObjectPointCloud(viewportPoint);
//				viewportPointCloudPtr->push_back(viewportPoint);
//				currentCameraViewport = viewportPoint;
//			}
//
//
//			////
//			// VISUALIZATION PART - DON'T CARE ABOUT THAT MESS!
//			////
//			uint32_t seq = 0;
//
////			viz::Marker pointMarker = MarkerHelper::getBasicMarker(seq++);
////			pointMarker.type = viz::Marker::POINTS;
////			pointMarker.lifetime = ros::Duration(30.0);
////			pointMarker.scale.x = pointMarker.scale.y = 0.05;
////			MarkerHelper::getRainbowColor(pointMarker, 2.0 / 6.0, 1.0);
////			pointMarker.points.push_back(viewportPoint.getPoint());
//
//			SimpleVector4 poisonGreenColorVector = SimpleVector4(191.0 / 255.0, 255.0 / 255.0, 0.0 / 255.0, 1.0);
//			SimpleVector4 darkBlueColorVector = SimpleVector4(4.0 / 255.0, 59.0 / 255.0, 89.0 / 255.0, 1.0);
//			SimpleVector4 blueColorVector = SimpleVector4(56.0 / 255.0, 129.0 / 255.0, 168.0 / 255.0, 1.0);
//			viz::MarkerArray arrowMarkerArray;
//			double ratio = 1.0 / ((double) viewportPointCloudPtr->size());
//			SimpleVector3 displacement(0.0, 0.0, 1.0);
//			for (std::size_t idx = 0; idx < viewportPointCloudPtr->size() - 1; idx++) {
//				ViewportPoint startViewportPoint = viewportPointCloudPtr->at(idx);
//				ViewportPoint endViewportPoint = viewportPointCloudPtr->at(idx + 1);
//
//				SimpleVector3 startPoint =  startViewportPoint.getSimpleVector3();
//				startPoint[2] = 0.0;
//				startPoint += idx * ratio * displacement;
//				SimpleVector3 endPoint = endViewportPoint.getSimpleVector3();
//				endPoint[2] = 0.0;
//				endPoint += (idx + 1) * ratio * displacement;
//
//				viz::Marker arrowMarker = MarkerHelper::getArrowMarker(seq++, startPoint, endPoint, blueColorVector);
//				arrowMarkerArray.markers.push_back(arrowMarker);
//			}
//
//			for (std::size_t idx = 0; idx < viewportPointCloudPtr->size(); idx++) {
//				ViewportPoint startViewportPoint = viewportPointCloudPtr->at(idx);
//
//				SimpleVector3 startPoint =  startViewportPoint.getSimpleVector3();
//				startPoint[2] = 1.32;
//
//				SimpleVector3 endPoint(startPoint);
//				endPoint[2] = 0.0;
//
//
//				viz::Marker arrowMarker = MarkerHelper::getArrowMarker(seq++, startPoint, endPoint, darkBlueColorVector);
//				arrowMarkerArray.markers.push_back(arrowMarker);
//
//
//
//				SimpleVector3 orientationStart = startPoint;
//				SimpleVector3 orientationVector = startViewportPoint.getSimpleQuaternion().toRotationMatrix() * SimpleVector3::UnitX();
//				SimpleVector3 orientationEnd = orientationStart + orientationVector / 3.0;
//
//				viz::Marker orientationMarker = MarkerHelper::getArrowMarker(seq++, orientationStart, orientationEnd, poisonGreenColorVector);
//				arrowMarkerArray.markers.push_back(orientationMarker);
//			}
//
//			ROS_WARN("%d points", viewportPointCloudPtr->size());
//
//			sensor_msgs::PointCloud2 pcROS2;
//			pcl::PCLPointCloud2 pcPCL2;
//			pcl::toPCLPointCloud2(*mCalculator.getPointCloudPtr(), pcPCL2);
//			pcl_conversions::fromPCL(pcPCL2, pcROS2);
//			pcROS2.header.frame_id = "map";
//
//			SamplePointCloudPtr cloudPtr = spaceSamplerPtr->getSampledSpacePointCloud(SimpleVector3(22.1536, 10.5516, 1.32));
//			sensor_msgs::PointCloud2 cloudPtrROS2;
//			pcl::PCLPointCloud2 cloudPtrPCL2;
//			pcl::toPCLPointCloud2(*cloudPtr, cloudPtrPCL2);
//			pcl_conversions::fromPCL(cloudPtrPCL2, cloudPtrROS2);
//			cloudPtrROS2.header.frame_id = "map";
//
//
//			ros::Publisher samplePub = mGlobalNodeHandle.advertise<sensor_msgs::PointCloud2>("samples", 1000);
//			ros::Publisher pcPub = mGlobalNodeHandle.advertise<sensor_msgs::PointCloud2>("object_positions", 1000);
//			ros::Publisher pub = mGlobalNodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
//			ros::Publisher arrayPub = mGlobalNodeHandle.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1000);
//			ros::Duration(5.0).sleep();
//
//			uint32_t sequence = seq;
//			while(ros::ok()) {
//				//pub.publish(pointMarker);
//				samplePub.publish(cloudPtrROS2);
//				pcPub.publish(pcROS2);
//				arrayPub.publish(arrowMarkerArray);
//
//				ros::spinOnce();
//				ros::Duration(20.0).sleep();
//			}
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

		static void vector_diff(const Indices &AVect, const Indices &BVect, Indices &resultIndices) {
			std::set<int> A(AVect.begin(), AVect.end());
			std::set<int> B(BVect.begin(), BVect.end());

			std::set_difference( A.begin(), A.end(), B.begin(), B.end(),
					std::back_inserter( resultIndices ) );
		}
	};
}


#endif /* NEXTBESTVIEW_HPP_ */
