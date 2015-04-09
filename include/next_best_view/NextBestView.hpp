/*
 * NextBestView.hpp
 *
 *  Created on: Aug 31, 2014
 *      Author: ralfschleicher
 */

#ifndef NEXTBESTVIEW_HPP_
#define NEXTBESTVIEW_HPP_

#include <eigen3/Eigen/Dense>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <vector>
#include <set>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <object_database/ObjectManager.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>

#include "typedef.hpp"
#include "next_best_view/GetPointCloud2.h"
#include "next_best_view/GetAttributedPointCloud.h"
#include "next_best_view/SetAttributedPointCloud.h"
#include "next_best_view/GetNextBestView.h"
#include "next_best_view/GetSpaceSampling.h"
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
	class NextBestView {
	private:
		NextBestViewCalculator mCalculator;
		ros::NodeHandle mGlobalNodeHandle;
		ros::NodeHandle mNodeHandle;
		ros::ServiceServer mGetPointCloud2ServiceClient;
		ros::ServiceServer mGetPointCloudServiceClient;
		ros::ServiceServer mSetPointCloudServiceClient;
		ros::ServiceServer mGetNextBestViewServiceClient;
		ros::ServiceServer mGetSpaceSamplingServer;
		ros::ServiceServer mUpdatePointCloud;
		ros::ServiceClient mObjectTypeServiceClient;
		ViewportPoint mInitialCameraViewport;
		ObjectPointCloudPtr mPointCloudPtr;
		KdTreePtr mKdTreePtr;
	public:
		NextBestView() : mGlobalNodeHandle(), mNodeHandle("/next_best_view"), mPointCloudPtr(new ObjectPointCloud()) {
			mGetPointCloud2ServiceClient = mNodeHandle.advertiseService("get_point_cloud2", &NextBestView::processGetPointCloud2ServiceCall, this);
			mGetPointCloudServiceClient = mNodeHandle.advertiseService("get_point_cloud", &NextBestView::processGetPointCloudServiceCall, this);
			mGetNextBestViewServiceClient = mNodeHandle.advertiseService("next_best_view", &NextBestView::processGetNextBestViewServiceCall, this);
			mSetPointCloudServiceClient = mNodeHandle.advertiseService("set_point_cloud", &NextBestView::processSetPointCloudServiceCall, this);
			mUpdatePointCloud = mNodeHandle.advertiseService("update_point_cloud", &NextBestView::processUpdatePointCloudServiceCall, this);
			mGetSpaceSamplingServer = mNodeHandle.advertiseService("get_space_sampling", &NextBestView::processGetSpaceSamplingServiceCall, this);
			mObjectTypeServiceClient = mGlobalNodeHandle.serviceClient<odb::ObjectType>("/object_database/object_type");

			ros::Publisher pub = mGlobalNodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
			ros::Publisher arrayPub = mGlobalNodeHandle.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1000);

			//Comment?: Please export to ROS parameter server.
			float fovx = 62.5;
			float fovy = 48.9;
			float ncp = .5;
			float fcp = 5.0;

			//Comment? Export data structure configuration
			SpiralApproxUnitSphereSamplerPtr unitSphereSamplerPtr(new SpiralApproxUnitSphereSampler());
			unitSphereSamplerPtr->setSamples(128);

//			PlaneSubSpaceSamplerPtr spaceSamplerPtr(new PlaneSubSpaceSampler());
//			spaceSamplerPtr->setSamples(32);

			MapHelperPtr mapHelperPtr(new MapHelper());
			mapHelperPtr->setCollisionThreshold(45);

			MapBasedHexagonSpaceSamplerPtr spaceSamplerPtr(new MapBasedHexagonSpaceSampler(mapHelperPtr));
			spaceSamplerPtr->setHexagonRadius(0.75);

			//Comment: Map based -> with raytracing 
			MapBasedSingleCameraModelFilterPtr cameraModelFilterPtr(new MapBasedSingleCameraModelFilter(mapHelperPtr, SimpleVector3(0.0, 0.0, 0.1)));
			cameraModelFilterPtr->setHorizontalFOV(fovx);
			cameraModelFilterPtr->setVerticalFOV(fovy);
			cameraModelFilterPtr->setNearClippingPlane(ncp);
			cameraModelFilterPtr->setFarClippingPlane(fcp);

			//Comment?
			MILDRobotModelPtr robotModelPtr(new MILDRobotModel());
			robotModelPtr->setTiltAngleLimits(-45, 45);
			robotModelPtr->setPanAngleLimits(-60, 60);

			DefaultRatingModulePtr ratingModulePtr(new DefaultRatingModule());
			//Comment? explain deviation calculation
			ratingModulePtr->setNormalityRatingAngle(45 / 180.0 * M_PI);

			PerspectiveHypothesisUpdaterPtr hypothesisUpdaterPtr(new PerspectiveHypothesisUpdater());
			hypothesisUpdaterPtr->setDefaultRatingModule(ratingModulePtr);

			mCalculator.setHypothesisUpdater(hypothesisUpdaterPtr);
			mCalculator.setUnitSphereSampler(unitSphereSamplerPtr);
			mCalculator.setSpaceSampler(spaceSamplerPtr);
			mCalculator.setCameraModelFilter(cameraModelFilterPtr);
			mCalculator.setRobotModel(robotModelPtr);
			mCalculator.setRatingModule(ratingModulePtr);
		}

		//dtor
		virtual ~NextBestView() { }

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
