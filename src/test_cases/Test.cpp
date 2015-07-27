/*
 * TestRecognitionManager.h
 *
 *  Created on: Mar 15, 2014
 *      Author: ralfschleicher
 */
#define BOOST_TEST_STATIC_LINK

#include <ros/ros.h>
#include <boost/test/included/unit_test.hpp>
#include <boost/array.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <glpk.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/PointCloud2.h>

#include "object_database/ObjectType.h"
#include "object_database/ObjectManager.hpp"
#include "next_best_view/NextBestView.hpp"
#include "next_best_view/GetNextBestView.h"
#include "next_best_view/GetPointCloud2.h"
#include "next_best_view/SetAttributedPointCloud.h"
#include "next_best_view/UpdatePointCloud.h"
#include "next_best_view/AttributedPoint.h"
#include "next_best_view/helper/MathHelper.hpp"
#include "next_best_view/helper/MarkerHelper.hpp"
#include "next_best_view/helper/MapHelper.hpp"
#include "next_best_view/helper/TypeHelper.hpp"
#include "next_best_view/robot_model/impl/MILDRobotModel.hpp"
#include "next_best_view/robot_model/impl/MILDRobotState.hpp"
#include <tf/tf.h>

#include "next_best_view/GetSpaceSampling.h"

using namespace next_best_view;
using namespace boost::unit_test;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef boost::shared_ptr<MoveBaseClient> MoveBaseClientPtr;

class Test {
private:
	ros::NodeHandle mNodeHandle;
	MoveBaseClientPtr mMoveBaseClient;
	ros::Publisher mInitPosePub;
public:
	Test() {
		mInitPosePub = mNodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 100, false);
		mMoveBaseClient = MoveBaseClientPtr(new MoveBaseClient("move_base", true));
	}

	virtual ~Test() {}

	/*!
	 * \brief evaluates the correctness of the Sphere To Cartesian and Cartesian To Sphere Methods.
	 */
	void evaluateS2CandC2S() {
		ROS_INFO("Running Test for S2C and C2S");

		// tolerance
		double tolerance = 2E-7;

		// Unit Sphere Tests
		int thetaDivisor = 32;
		double thetaStepSize = M_PI / (double) thetaDivisor;
		int phiDivisor = 64;
		double phiStepSize = M_2_PI / (double) phiDivisor;
		for (int thetaFac = 0; thetaFac < thetaDivisor; thetaFac++) {
			double theta = - M_PI_2 + thetaFac * thetaStepSize;
			for (int phiFac = 0; phiFac < phiDivisor; phiFac++) {
				double phi = - M_PI + phiFac * phiStepSize;

				// create coordinates
				SimpleSphereCoordinates scoords(1, theta, phi);
				// convert to cartesian
				SimpleVector3 ccoords = MathHelper::convertS2C(scoords);
				// convert to sphere again
				SimpleSphereCoordinates rescoords = MathHelper::convertC2S(ccoords);
				// convert to cartesian again
				SimpleVector3 reccoords = MathHelper::convertS2C(rescoords);

				double cerror = (ccoords - reccoords).lpNorm<2>();

				// error has to be minimal.
				BOOST_REQUIRE(cerror <= tolerance);
			}
		}
	}

	void solveLinearProblem() {
		MILDRobotStatePtr robotState(new MILDRobotState());
		robotState->pan = 0;
		robotState->tilt = M_PI / 6.0;
		robotState->rotation = 25.0 * M_PI / 32.0;

		MILDRobotModel model;
		model.setPanAngleLimits(-45, 45);
		model.setTiltAngleLimits(-45, 45);
		RobotStatePtr state = model.calculateRobotState(robotState, SimpleVector3(20, 50, 0), MathHelper::getQuaternionByAngles(M_PI / 64.0, 0, 0));
		ROS_INFO("costs: %g", model.getMovementCosts(robotState, state));
	}

	void visualizeSingleObjectWithNormals() {
		ros::Publisher pubOne = mNodeHandle.advertise<visualization_msgs::Marker>("/visualization_marker", 100, false);
		ros::Publisher pub = mNodeHandle.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 100, false);

		visualization_msgs::MarkerArray markerArray;
		object_database::ObjectManager manager;
		object_database::ObjectTypeResponsePtr objectTypeResPtr = manager.get("Smacks");

		ros::Duration(5).sleep();

		visualization_msgs::Marker deleteMarker = MarkerHelper::getBasicMarker(0);

		deleteMarker.action = 3u;
		pubOne.publish(deleteMarker);


		ros::Duration(5).sleep();
		int id = 0;
		SimpleVector3 zero(0, 0, 0);
		SimpleQuaternion rotation = MathHelper::getQuaternionByAngles(0, 0, -M_PI / 2.0);

		Indices ind;
		for (std::size_t idx = 0; idx < objectTypeResPtr->normal_vectors.size(); idx++) {
			geometry_msgs::Point pt = objectTypeResPtr->normal_vectors.at(idx);
			SimpleVector3 pos = TypeHelper::getSimpleVector3(pt);
			bool rebel = false;
			BOOST_FOREACH(std::size_t index, ind) {
				geometry_msgs::Point pt2 = objectTypeResPtr->normal_vectors.at(index);
				SimpleVector3 pos2 = TypeHelper::getSimpleVector3(pt2);
				double val = pos.dot(pos2);
				if (val >= cos(M_PI / 3.0)) {
					rebel = true;
				}
			}

			if (!rebel) {
				ind.push_back(idx);
			}
		}

		BOOST_FOREACH(std::size_t index, ind) {
			geometry_msgs::Point pt = objectTypeResPtr->normal_vectors.at(index);
			SimpleVector3 pos = TypeHelper::getSimpleVector3(pt);
			pos = rotation.toRotationMatrix() * pos;
			visualization_msgs::Marker arrowMarker = MarkerHelper::getArrowMarker(id, zero, pos, SimpleVector4(0.3, 0.3, 1.0, 1.0));
			markerArray.markers.push_back(arrowMarker);
			id++;

		}

		visualization_msgs::Marker objectMarker = MarkerHelper::getMeshMarker(id, objectTypeResPtr->object_mesh_resource, zero, rotation);
		markerArray.markers.push_back(objectMarker);

		pub.publish(markerArray);

		ros::spinOnce();
	}

	void setInitialPose(const geometry_msgs::Pose &initialPose) {
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

		mInitPosePub.publish(pose);
	}

	void moveToPose(const geometry_msgs::Pose &pose) {

		while(!mMoveBaseClient->waitForServer(ros::Duration(5.0))) {
			ROS_INFO("Waiting for the move_base action server to come up");
		}

		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose = pose;

		mMoveBaseClient->sendGoal(goal);

		mMoveBaseClient->waitForResult();
		if(mMoveBaseClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO("Hooray, the base moved");
		} else {
			ROS_ERROR("The base failed to move");
		}
	}

	void visualizeSpaceSampling() {
		ros::ServiceClient client = mNodeHandle.serviceClient<GetSpaceSampling>("/next_best_view/get_space_sampling");
		ros::Publisher pub = mNodeHandle.advertise<sensor_msgs::PointCloud2>("/test/space_sampling_point_cloud", 100);


		GetSpaceSampling* sampling = new GetSpaceSampling[20];
		for (int i = 0; i < 20; i++) {
			sampling[i].request.contractor = 1.0 / (double) (pow(2, i));
			if (!client.call(sampling[i])) {
				ROS_INFO("Some error occured");
				return;
			}
		}

		ros::Duration(5).sleep();

		while(ros::ok()) {
			for (int i = 0; i < 20 && ros::ok(); i++) {
				pub.publish(sampling[i].response.point_cloud);

				ros::spinOnce();
				ros::Duration(2.5).sleep();
			}

			ros::spinOnce();
			ros::Duration(5).sleep();
		}
	}

	void iterationTest() {
		ros::ServiceClient setPointCloudClient = mNodeHandle.serviceClient<SetAttributedPointCloud>("/nbv/set_point_cloud");
		ros::ServiceClient getPointCloud2Client = mNodeHandle.serviceClient<GetPointCloud2>("/nbv/get_point_cloud2");
		ros::ServiceClient getNextBestViewClient = mNodeHandle.serviceClient<GetNextBestView>("/nbv/next_best_view");
		ros::ServiceClient updatePointCloudClient = mNodeHandle.serviceClient<UpdatePointCloud>("/nbv/update_point_cloud");
		ros::ServiceClient getSpaceSamplingClient = mNodeHandle.serviceClient<GetSpaceSampling>("/nbv/get_space_sampling");

		SetAttributedPointCloud apc;

		ROS_INFO("Generiere Häufungspunkte");
		// Häufungspunkte
		int hpSize = 3;
		SimpleVector3* hp = new SimpleVector3[hpSize];
		hp[0] = SimpleVector3(16.0, 16.0, 0.98);
		hp[1] = SimpleVector3(25.0, 11.0, 1.32);
		hp[2] = SimpleVector3(18.6, 7.3, 1.80);

		std::vector<std::string> objectTypeNames;
		objectTypeNames.push_back("Smacks");
		//objectTypeNames.push_back("Coffeebox");

		int sampleSize = 200;

		MapHelper mapHelper;

		for (std::size_t idx = 0; idx < hpSize; idx++) {
			for (std::size_t cnt = 0; cnt < sampleSize; cnt++) {
				SimpleVector3 randomVector;
				int8_t occupancyValue;
				do {
					randomVector = MathHelper::getRandomVector(hp[idx], SimpleVector3(.5, .5, 0.1));
					occupancyValue = mapHelper.getRaytracingMapOccupancyValue(randomVector);
				} while(!mapHelper.isOccupancyValueAcceptable(occupancyValue));

				SimpleQuaternion randomQuat = next_best_view::MathHelper::getRandomQuaternion();

				AttributedPoint element;

				geometry_msgs::Pose pose;
				pose.orientation.w = randomQuat.w();
				pose.orientation.x = randomQuat.x();
				pose.orientation.y = randomQuat.y();
				pose.orientation.z = randomQuat.z();
				pose.position.x = randomVector[0];
				pose.position.y = randomVector[1];
				pose.position.z = randomVector[2];

				element.object_type = objectTypeNames[next_best_view::MathHelper::getRandomInteger(0, objectTypeNames.size() - 1)];
				element.pose = pose;

				apc.request.point_cloud.elements.push_back(element);
			}
		}

		ROS_INFO("Setze initiale Pose");
		geometry_msgs::Pose initialPose;
		initialPose.position.x = 2.45717;
		initialPose.position.y = 22.276;
		initialPose.position.z = 1.32;

		initialPose.orientation.w = 0.971467;
		initialPose.orientation.x = 0.0;
		initialPose.orientation.y = 0.0;
		initialPose.orientation.z = -0.237177;
		this->setInitialPose(initialPose);

		apc.request.pose = initialPose;

		// Setze PointCloud
		setPointCloudClient.call(apc.request, apc.response);

		GetNextBestView nbv;
		nbv.request.initial_pose = initialPose;
		ViewportPointCloudPtr viewportPointCloudPtr(new ViewportPointCloud());
		int x = 1;
		while(ros::ok()) {
			ROS_INFO("Kalkuliere NBV (%d)", x);
			if (!getNextBestViewClient.call(nbv.request, nbv.response)) {
				ROS_ERROR("Something went wrong in next best view");
				break;
			}

			if (!nbv.response.found) {
				break;
			}
			x++;

			ros::spinOnce();
			waitForEnter();
			ros::Duration(2).sleep();
		}
//
//		uint32_t seq = 0;
//
//		SimpleVector4 poisonGreenColorVector = SimpleVector4(191.0 / 255.0, 255.0 / 255.0, 0.0 / 255.0, 1.0);
//		SimpleVector4 darkBlueColorVector = SimpleVector4(4.0 / 255.0, 59.0 / 255.0, 89.0 / 255.0, 1.0);
//		SimpleVector4 blueColorVector = SimpleVector4(56.0 / 255.0, 129.0 / 255.0, 168.0 / 255.0, 1.0);
//		viz::MarkerArray arrowMarkerArray;
//		double ratio = 1.0 / ((double) viewportPointCloudPtr->size());
//		SimpleVector3 displacement(0.0, 0.0, 1.0);
//		for (std::size_t idx = 0; idx < viewportPointCloudPtr->size() - 1; idx++) {
//			ViewportPoint startViewportPoint = viewportPointCloudPtr->at(idx);
//			ViewportPoint endViewportPoint = viewportPointCloudPtr->at(idx + 1);
//
//			SimpleVector3 startPoint =  startViewportPoint.getSimpleVector3();
//			startPoint[2] = 0.0;
//			startPoint += idx * ratio * displacement;
//			SimpleVector3 endPoint = endViewportPoint.getSimpleVector3();
//			endPoint[2] = 0.0;
//			endPoint += (idx + 1) * ratio * displacement;
//
//			viz::Marker arrowMarker = MarkerHelper::getArrowMarker(seq++, startPoint, endPoint, blueColorVector);
//			arrowMarkerArray.markers.push_back(arrowMarker);
//		}
//
//		for (std::size_t idx = 0; idx < viewportPointCloudPtr->size(); idx++) {
//			ViewportPoint startViewportPoint = viewportPointCloudPtr->at(idx);
//
//			SimpleVector3 startPoint =  startViewportPoint.getSimpleVector3();
//			startPoint[2] = 1.32;
//
//			SimpleVector3 endPoint(startPoint);
//			endPoint[2] = 0.0;
//
//
//			viz::Marker arrowMarker = MarkerHelper::getArrowMarker(seq++, startPoint, endPoint, darkBlueColorVector);
//			arrowMarkerArray.markers.push_back(arrowMarker);
//
//
//
//			SimpleVector3 orientationStart = startPoint;
//			SimpleVector3 orientationVector = startViewportPoint.getSimpleQuaternion().toRotationMatrix() * SimpleVector3::UnitX();
//			SimpleVector3 orientationEnd = orientationStart + orientationVector / 3.0;
//
//			viz::Marker orientationMarker = MarkerHelper::getArrowMarker(seq++, orientationStart, orientationEnd, poisonGreenColorVector);
//			arrowMarkerArray.markers.push_back(orientationMarker);
//		}
//
//		while(ros::ok()) {
//			markerArrayPublisher.publish(arrowMarkerArray);
//
//			ros::spinOnce();
//			ros::Duration(20.0).sleep();
//		}
	}

	static void waitForEnter() {
//		std::string dummy;
//		std::cout << "Press ENTER to continue.." << std::endl << ">";
//		std::getline(std::cin, dummy);
//		std::cout << std::endl;
	}
};

test_suite* init_unit_test_suite( int argc, char* argv[] ) {
	ros::init(argc, argv, "nbv_test");
	ros::start();

	ros::Duration(5).sleep();

	test_suite* evaluation = BOOST_TEST_SUITE("Evaluation NBV");

	boost::shared_ptr<Test> testPtr(new Test());

//	evaluation->add(BOOST_CLASS_TEST_CASE(&Test::evaluateS2CandC2S, testPtr));
//	evaluation->add(BOOST_CLASS_TEST_CASE(&Test::solveLinearProblem, testPtr));
//	evaluation->add(BOOST_CLASS_TEST_CASE(&Test::visualizeSingleObjectWithNormals, testPtr));
//	evaluation->add(BOOST_CLASS_TEST_CASE(&Test::visualizeSpaceSampling, testPtr));
	evaluation->add(BOOST_CLASS_TEST_CASE(&Test::iterationTest, testPtr));
//	interfaceTest->add(BOOST_CLASS_TEST_CASE(&Test::testCategoryList, testRecognitionManagerPtr));
//	interfaceTest->add(BOOST_CLASS_TEST_CASE(&Test::testEachCategoryEntryList, testRecognitionManagerPtr));
//	interfaceTest->add(BOOST_CLASS_TEST_CASE(&Test::testGetRecognizerNoInput, testRecognitionManagerPtr));
//	interfaceTest->add(BOOST_CLASS_TEST_CASE(&Test::testGetRecognizer, testRecognitionManagerPtr));
//	interfaceTest->add(BOOST_CLASS_TEST_CASE(&Test::testReleaseRecognizer, testRecognitionManagerPtr));
//	interfaceTest->add(BOOST_CLASS_TEST_CASE(&Test::testGetAndReleaseOfManyRecognizers, testRecognitionManagerPtr));
//	interfaceTest->add(BOOST_CLASS_TEST_CASE(&Test::testGetAndReleaseOfManyRecognizersViaRecognizerHandle, testRecognitionManagerPtr));

    framework::master_test_suite().add(evaluation);

    return 0;
}

