/*
 * NextBestView.hpp
 *
 *  Created on: Aug 31, 2014
 *      Author: ralfschleicher
 */

#ifndef NEXTBESTVIEW_HPP_
#define NEXTBESTVIEW_HPP_

#include <Eigen/Dense>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <object_database/ObjectManager.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "typedef.hpp"
#include "next_best_view/utility/MapUtility.hpp"
#include "next_best_view/NextBestViewCalculator.hpp"
#include "next_best_view/helper/MarkerHelper.hpp"
#include "next_best_view/helper/MathHelper.hpp"
#include "next_best_view/AttributedPointCloud.h"
#include "next_best_view/AttributedPoint.h"
#include "next_best_view/camera_model_filter/impl/CostmapBasedStereoCameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/impl/SingleCameraModelFilter.hpp"
#include "next_best_view/hypothesis_updater/impl/PerspectiveHypothesisUpdater.hpp"
#include "next_best_view/robot_model/impl/PTURobotModel.hpp"
#include "next_best_view/unit_sphere_sampler/impl/SpiralApproxUnitSphereSampler.hpp"
#include "next_best_view/space_sampler/impl/PlaneSubSpaceSampler.hpp"
#include "next_best_view/space_sampler/impl/CostmapBasedSpaceSampler.hpp"
#include "next_best_view/rating/impl/DefaultRatingModule.hpp"


namespace next_best_view {
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

	namespace fs = boost::filesystem;
	namespace viz = visualization_msgs;
	namespace odb = object_database;
	class NextBestView {
	private:
		ros::NodeHandle mGlobalNodeHandle;
		ros::NodeHandle mNodeHandle;
		ros::Subscriber mPointCloudMessageSubscriber;
		ros::ServiceClient mObjectTypeServiceClient;
		ObjectPointCloudPtr mPointCloudPtr;
		KdTreePtr mKdTreePtr;
	public:
		NextBestView() : mGlobalNodeHandle(), mNodeHandle("/next_best_view"), mPointCloudPtr(new ObjectPointCloud()) {
			mPointCloudMessageSubscriber = mNodeHandle.subscribe("set_point_cloud", 100, &NextBestView::processPointCloudMessage, this);
			mObjectTypeServiceClient = mGlobalNodeHandle.serviceClient<odb::ObjectType>("/object_database/object_type");

			ros::Publisher pub = mGlobalNodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
			ros::Publisher arrayPub = mGlobalNodeHandle.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1000);
		}

		//dtor
		virtual ~NextBestView() { }

		void processPointCloudMessage(const AttributedPointCloud &msg) {
			// TODO wrap into PointCloudMessage.
			ViewportPoint initialCameraViewport;
			initialCameraViewport.x = 22.15;
			initialCameraViewport.y = 10.55;
			initialCameraViewport.z = 1.32;
			initialCameraViewport.qw = 1.0;
			initialCameraViewport.qx = 0.0;
			initialCameraViewport.qy = 0.0;
			initialCameraViewport.qz = 0.0;

			float fovx = 90.0;
			float fovy = 60.0;
			float ncp = 1.0;
			float fcp = 5.0;

			SpiralApproxUnitSphereSamplerPtr unitSphereSamplerPtr(new SpiralApproxUnitSphereSampler());
			unitSphereSamplerPtr->setSamples(24);

//			PlaneSubSpaceSamplerPtr spaceSamplerPtr(new PlaneSubSpaceSampler());
//			spaceSamplerPtr->setSamples(32);
			MapUtilityPtr mapUtilityPtr(new MapUtility());

			SimpleVector3 startPoint(22.1536, 10.5516, 0.0);
			for (double x = 0; x < 2 * M_PI; x += 2 * M_PI / 9.0) {
				SimpleVector3 targetPoint(cos(x), sin(x), 0);
				targetPoint += startPoint;
				ROS_INFO("CollisionFree: %s", (mapUtilityPtr->doRaytracing(startPoint, targetPoint) ? " true" : "false"));
			}

			CostmapBasedSpaceSamplerPtr spaceSamplerPtr(new CostmapBasedSpaceSampler(mapUtilityPtr));
			spaceSamplerPtr->setSamples(128);

			CostmapBasedStereoCameraModelFilterPtr cameraModelFilterPtr(new CostmapBasedStereoCameraModelFilter(mapUtilityPtr, SimpleVector3(0.0, 0.05, 0.1), SimpleVector3(0.0, -0.05, 0.1)));
			cameraModelFilterPtr->setHorizontalFOV(fovx);
			cameraModelFilterPtr->setVerticalFOV(fovy);
			cameraModelFilterPtr->setNearClippingPlane(ncp);
			cameraModelFilterPtr->setFarClippingPlane(fcp);
			cameraModelFilterPtr->setOrientation(initialCameraViewport.getSimpleQuaternion());
			//cameraModelFilterPtr->setFilteringType(StereoCameraModelFilter::BOTH);
			cameraModelFilterPtr->setPivotPointPosition(initialCameraViewport.getSimpleVector3());

			PTURobotModelPtr robotModelPtr(new PTURobotModel());
			robotModelPtr->setTiltAngleLimits(-45, 45);
			robotModelPtr->setPanAngleLimits(-60, 60);

			DefaultRatingModulePtr ratingModulePtr(new DefaultRatingModule());
			ratingModulePtr->setNormalityRatingAngle(60.0 / 180.0 * M_PI);

			PerspectiveHypothesisUpdaterPtr hypothesisUpdaterPtr(new PerspectiveHypothesisUpdater());
			hypothesisUpdaterPtr->setDefaultRatingModule(ratingModulePtr);

			NextBestViewCalculator calculator;
			calculator.setHypothesisUpdater(hypothesisUpdaterPtr);
			calculator.setUnitSphereSampler(unitSphereSamplerPtr);
			calculator.setSpaceSampler(spaceSamplerPtr);
			calculator.setCameraModelFilter(cameraModelFilterPtr);
			calculator.setRobotModel(robotModelPtr);
			calculator.setRatingModule(ratingModulePtr);
			calculator.setPointCloudFromMessage(msg);

			ViewportPointCloudPtr viewportPointCloudPtr(new ViewportPointCloud());
			ViewportPoint currentCameraViewport = initialCameraViewport;
			while(ros::ok()) {
				ViewportPoint viewportPoint;
				if (!calculator.calculateNextBestView(currentCameraViewport, viewportPoint)) {
					break;
				}

				calculator.updateObjectPoints(viewportPoint);
				viewportPointCloudPtr->push_back(viewportPoint);
				currentCameraViewport = viewportPoint;
			}


			////
			// VISUALIZATION PART - DON'T CARE ABOUT THAT MESS!
			////
			uint32_t seq = 0;

//			viz::Marker pointMarker = MarkerHelper::getBasicMarker(seq++);
//			pointMarker.type = viz::Marker::POINTS;
//			pointMarker.lifetime = ros::Duration(30.0);
//			pointMarker.scale.x = pointMarker.scale.y = 0.05;
//			MarkerHelper::getRainbowColor(pointMarker, 2.0 / 6.0, 1.0);
//			pointMarker.points.push_back(viewportPoint.getPoint());

			viz::MarkerArray arrowMarkerArray;
			double ratio = 1.0 / ((double) viewportPointCloudPtr->size());
			SimpleVector3 displacement(0.0, 0.0, 1.0);
			for (std::size_t idx = 0; idx < viewportPointCloudPtr->size() - 1; idx++) {
				ViewportPoint startViewportPoint = viewportPointCloudPtr->at(idx);
				ViewportPoint endViewportPoint = viewportPointCloudPtr->at(idx + 1);

				SimpleVector3 startPoint =  startViewportPoint.getSimpleVector3();
				startPoint[2] = 0.0;
				startPoint += idx * ratio * displacement;
				SimpleVector3 endPoint = endViewportPoint.getSimpleVector3();
				endPoint[2] = 0.0;
				endPoint += (idx + 1) * ratio * displacement;

				viz::Marker arrowMarker = MarkerHelper::getArrowMarker(seq++, startPoint, endPoint);
				MarkerHelper::getRainbowColor(arrowMarker, idx * ratio * 5.0 / 6.0, 1.0);
				arrowMarkerArray.markers.push_back(arrowMarker);
			}

			for (std::size_t idx = 0; idx < viewportPointCloudPtr->size(); idx++) {
				ViewportPoint startViewportPoint = viewportPointCloudPtr->at(idx);

				SimpleVector3 startPoint =  startViewportPoint.getSimpleVector3();
				startPoint[2] = 1.0;

				SimpleVector3 endPoint(startPoint);
				endPoint[2] = 0.0;

				viz::Marker arrowMarker = MarkerHelper::getArrowMarker(seq++, startPoint, endPoint);
				MarkerHelper::getRainbowColor(arrowMarker, idx * ratio, 1.0);
				arrowMarkerArray.markers.push_back(arrowMarker);
			}

			ROS_WARN("%d points", viewportPointCloudPtr->size());

			sensor_msgs::PointCloud2 pcROS2;
			pcl::PCLPointCloud2 pcPCL2;
			pcl::toPCLPointCloud2(*calculator.getPointCloudPtr(), pcPCL2);
			pcl_conversions::fromPCL(pcPCL2, pcROS2);
			pcROS2.header.frame_id = "map";

			SamplePointCloudPtr cloudPtr = spaceSamplerPtr->getSampledSpacePointCloud(SimpleVector3(22.1536, 10.5516, 1.32));
			sensor_msgs::PointCloud2 cloudPtrROS2;
			pcl::PCLPointCloud2 cloudPtrPCL2;
			pcl::toPCLPointCloud2(*cloudPtr, cloudPtrPCL2);
			pcl_conversions::fromPCL(cloudPtrPCL2, cloudPtrROS2);
			cloudPtrROS2.header.frame_id = "map";


			ros::Publisher samplePub = mGlobalNodeHandle.advertise<sensor_msgs::PointCloud2>("samples", 1000);
			ros::Publisher pcPub = mGlobalNodeHandle.advertise<sensor_msgs::PointCloud2>("object_positions", 1000);
			ros::Publisher pub = mGlobalNodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
			ros::Publisher arrayPub = mGlobalNodeHandle.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1000);
			ros::Duration(5.0).sleep();

			uint32_t sequence = seq;
			while(ros::ok()) {
				//pub.publish(pointMarker);
				samplePub.publish(cloudPtrROS2);
				pcPub.publish(pcROS2);
				arrayPub.publish(arrowMarkerArray);

				ros::spinOnce();
				ros::Duration(20.0).sleep();
			}
		}
	};
}


#endif /* NEXTBESTVIEW_HPP_ */
