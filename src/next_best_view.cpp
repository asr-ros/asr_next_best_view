/*
 * next_best_view.cpp
 *
 *  Created on: Aug 31, 2014
 *      Author: ralfschleicher
 */

#include <assert.h>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <set>

#include "typedef.hpp"
#include "next_best_view/helper/MapHelper.hpp"
#include "next_best_view/NextBestView.hpp"
#include "next_best_view/helper/MathHelper.hpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "next_best_view");
	ros::start();

	next_best_view::NextBestView nbv;

//	next_best_view::PTURobotModel model;
//	model.setPanAngleLimits(-60, 60);
//	for(double pan = 0; pan < 360; pan += 40) {
//		model.setCurrentState(pan, 0.0, 0.0);
//		for(double ppan = 0; ppan < 360; ppan += 20) {
//			next_best_view::SimpleQuaternion quat = next_best_view::MathHelper::getQuaternionByAngles(ppan / 180.0 * M_PI, 0.0, 0.0);
//			model.getMovementRating(quat);
//		}
//	}

//	printf("%e, %e, %e \n", std::numeric_limits<float>::min(), std::numeric_limits<float>::max(), 1.0 / std::numeric_limits<float>::max());
//
//	next_best_view::MapHelper mapHelper;
//	next_best_view::NextBestView nbv;
//	std::vector<std::string> objectTypeNames;
//	objectTypeNames.push_back("Smacks");
//	objectTypeNames.push_back("CoffeeBox");
//
//	float roomHeight = 2.2;
//
//	pbd_msgs::PbdAttributedPointCloud msg;
//	next_best_view::SimpleVector3 mue(mapHelper.getMetricWidth() / 2.0, mapHelper.getMetricHeight() / 2.0, roomHeight / 2.0);
//	next_best_view::SimpleVector3 deviation(mapHelper.getMetricWidth() / 4.0, mapHelper.getMetricHeight() / 4.0, roomHeight / 8.0);
//	next_best_view::SimpleVector3 mean = mue;
//	ROS_DEBUG("Start creation of pointcloud");
//	for (std::size_t i = 0; i < 400; i += 1) {
//		int8_t occupancyValue = -1;
//		while (i % 200 == 0 && occupancyValue != 0) {
//			mean = next_best_view::MathHelper::getRandomVector(mue, deviation);
//			occupancyValue = mapHelper.getMapOccupancyValue(mean);
//		}
//		next_best_view::SimpleVector3 randomVector;
//		do {
//			randomVector = next_best_view::MathHelper::getRandomVector(mean, next_best_view::SimpleVector3(0.2, 0.2, 0.005));
//			occupancyValue = mapHelper.getMapOccupancyValue(randomVector);
//
//			if (randomVector[2] <= 0 || randomVector[2] >= roomHeight) {
//				ROS_DEBUG_STREAM("Retry, occupancy value " << occupancyValue << "\n" << randomVector);
//				continue;
//			}
//		} while(occupancyValue != 0);
//
//		next_best_view::SimpleQuaternion randomQuat = next_best_view::MathHelper::getRandomQuaternion();
//
//		pbd_msgs::PbdAttributedPoint element;
//
//		geometry_msgs::Pose pose;
//		pose.orientation.w = randomQuat.w();
//		pose.orientation.x = randomQuat.x();
//		pose.orientation.y = randomQuat.y();
//		pose.orientation.z = randomQuat.z();
//		pose.position.x = randomVector[0];
//		pose.position.y = randomVector[1];
//		pose.position.z = randomVector[2];
//
//		element.object_type = objectTypeNames[next_best_view::MathHelper::getRandomInteger(0, objectTypeNames.size() - 1)];
//		element.pose = pose;
//
//		msg.elements.push_back(element);
//	}
//	ROS_DEBUG("Finished creation of pointcloud");
////	for (float phi = - M_PI / 2.0; phi <= M_PI / 2.0; phi += M_PI / 16.0) {
////		for (float theta = 0.0; theta < 2.0 * M_PI; theta += M_PI / 16.0) {
////			next_best_view::PointCloudElement element;
////
////			element.object_type = "Smacks";
////			float r = fabsf(next_best_view::MathHelper::getRandomNumber(0, 2.5));
////			element.pose.position.x = cos(phi) * cos(theta) * r;
////			element.pose.position.y = cos(phi) * sin(theta) * r;
////			element.pose.position.z = sin(phi) * r;
////
////			msg.elements.push_back(element);
////		}
////	}
//	nbv.processPointCloudMessage(msg);

	ros::spin();

	return 0;
}



