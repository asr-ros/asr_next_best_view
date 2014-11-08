/*
 * TestRecognitionManager.h
 *
 *  Created on: Mar 15, 2014
 *      Author: ralfschleicher
 */
#define BOOST_TEST_STATIC_LINK

#include <ros/ros.h>
#include <boost/test/included/unit_test.hpp>
#include <iostream>
#include <vector>
#include <glpk.h>

#include <visualization_msgs/MarkerArray.h>

#include "object_database/ObjectType.h"
#include "object_database/ObjectManager.hpp"
#include "next_best_view/helper/MathHelper.hpp"
#include "next_best_view/helper/MarkerHelper.hpp"
#include "next_best_view/helper/TypeHelper.hpp"
#include "next_best_view/robot_model/impl/MILDRobotModel.hpp"
#include "next_best_view/robot_model/impl/MILDRobotState.hpp"

using namespace next_best_view;
using namespace boost::unit_test;
class Test {
public:
	Test() {
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
		int argc = 0;
		char** argv = new char*[0];
		ros::init(argc, argv, "nbv_test");
		ros::start();

		ros::NodeHandle nodeHandle;
		ros::Publisher pub = nodeHandle.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 100, false);

		visualization_msgs::MarkerArray markerArray;
		object_database::ObjectManager manager;
		object_database::ObjectTypeResponsePtr objectTypeResPtr = manager.get("Smacks");

		ros::Duration(5).sleep();

		visualization_msgs::MarkerArray deleteMarkerArray;

		for (int id = 0; id < 200; id++) {
			 visualization_msgs::Marker deleteMarker = MarkerHelper::getBasicMarker(id);
			 deleteMarker.action = visualization_msgs::Marker::DELETE;

			 deleteMarkerArray.markers.push_back(deleteMarker);
		}

		pub.publish(deleteMarkerArray);

		ros::Duration(5).sleep();
		int id = 0;
		SimpleVector3 zero(0, 0, 0);
		SimpleQuaternion rotation = MathHelper::getQuaternionByAngles(0, M_PI / 2, 0);
		BOOST_FOREACH(geometry_msgs::Point pt, objectTypeResPtr->normal_vectors) {
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
		ros::Rate(10).sleep();

		ros::shutdown();
	}
};

test_suite* init_unit_test_suite( int argc, char* argv[] ) {
	test_suite* evaluation = BOOST_TEST_SUITE("Evaluation NBV");

	boost::shared_ptr<Test> testPtr(new Test());

	evaluation->add(BOOST_CLASS_TEST_CASE(&Test::evaluateS2CandC2S, testPtr));
	evaluation->add(BOOST_CLASS_TEST_CASE(&Test::solveLinearProblem, testPtr));
	evaluation->add(BOOST_CLASS_TEST_CASE(&Test::visualizeSingleObjectWithNormals, testPtr));
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

