
/*
 *
 *  Created on: Nov 6, 2015
 *      Author: florianaumann
 */
#define BOOST_TEST_STATIC_LINK

#include <boost/test/included/unit_test.hpp>
#include "next_best_view/test_cases/BaseTest.h"
#include "next_best_view/robot_model/impl/MILDRobotModel_with_IK.hpp"
#include "next_best_view/robot_model/impl/MILDRobotState.hpp"
#include "next_best_view/GetAttributedPointCloud.h"

using namespace next_best_view;
using namespace boost::unit_test;

class IKTest : public BaseTest{
public:
    IKTest() : BaseTest (){
    }

    virtual ~IKTest() {}

    void cameraPoseTest() {
        std::vector<SimpleVector3> targetCameraPositions;
        std::vector<SimpleQuaternion> targetCameraOrientations;
        //Initialize Poses
        targetCameraPositions.push_back(SimpleVector3(0, 0, 0));
        targetCameraOrientations.push_back(ZXZ2Quaternion(0.0, 0.0, 0.0));
        targetCameraPositions.push_back(SimpleVector3(0.725892364979, 1.67344818115, 1.4));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-170, 0.0, 0 ));
        targetCameraPositions.push_back(SimpleVector3(0.725892364979, 1.67344818115, 1.4));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-170, 80.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.725892364979, 1.67344818115, 1.4));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-180, 100.0, 0 ));
        targetCameraPositions.push_back(SimpleVector3(0.725892364979, 1.67344818115, 1.4));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-200, 40.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.725892364979, 1.67344818115, 1.4));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-180, -30.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.725892364979, 1.67344818115, 1.4));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-190, -50.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.725892364979, 1.67344818115, 1.4));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-120, -20.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.725892364979, 1.67344818115, 1.4));
        targetCameraOrientations.push_back(ZXZ2Quaternion(120, -20.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.725892364979, 1.67344818115, 1.5));
        targetCameraOrientations.push_back(ZXZ2Quaternion(20, -20.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.725892364979, 1.67344818115, 0.8));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-10, 0.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.725892364979, 1.67344818115, 1.7));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-170, 0.0, 0 ));
        targetCameraPositions.push_back(SimpleVector3(0.725892364979, 1.67344818115, 0.4));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-170, 80.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.725892364979, 1.67344818115, 0.5));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-180, 100.0, 0 ));
        targetCameraPositions.push_back(SimpleVector3(0.725892364979, 1.67344818115, 1.0));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-200, 40.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.725892364979, 1.67344818115, 1.8));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-180, -30.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.725892364979, 1.67344818115, 1.3));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-190, -50.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.725892364979, 1.67344818115, 1.1));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-120, -20.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.725892364979, 1.67344818115, 1.7));
        targetCameraOrientations.push_back(ZXZ2Quaternion(120, -20.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.725892364979, 1.67344818115, 1.0));
        targetCameraOrientations.push_back(ZXZ2Quaternion(20, -20.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.725892364979, 1.67344818115, 0.9));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-10, 0.0, 0));
        //a couple of very difficult test cases
        targetCameraPositions.push_back(SimpleVector3(0.725892364979, 1.67344818115, 2.5));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-20, -20.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.725892364979, 1.67344818115, 0.0));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-10, 30.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.725892364979, 1.67344818115, -1.0));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-10, 30.0, 0));


        ROS_INFO_STREAM("Initializing...");

        //Initialize robot model
        MILDRobotModelWithIK *myRobotModel = new MILDRobotModelWithIK();
        MILDRobotModelWithIKPtr myRobotModelPtr(myRobotModel);
        MILDRobotState * startState = new MILDRobotState(0,0,0,-1.59984135628,-0.908775866032);
        MILDRobotStatePtr startStatePtr(startState);

        myRobotModelPtr->setPanAngleLimits(-80, 80);
        ROS_INFO_STREAM("Running test...");
        ros::spinOnce();
        for (unsigned int i = 0; i < targetCameraPositions.size(); i++)
        {
            ROS_INFO_STREAM("Testpose " << (i+1));
            SimpleVector3 currentPosition = targetCameraPositions[i];
            SimpleQuaternion currentOrientation = targetCameraOrientations[i];
            RobotStatePtr newStatePtr = myRobotModelPtr->calculateRobotState(startStatePtr, currentPosition, currentOrientation);
            ros::spinOnce();
            waitForEnter();
            ros::Duration(2).sleep();
        }
        ROS_INFO_STREAM("All tests done.");
	}
};

test_suite* init_unit_test_suite( int argc, char* argv[] ) {
	ros::init(argc, argv, "nbv_test");
	ros::start();

	ros::Duration(5).sleep();

	test_suite* evaluation = BOOST_TEST_SUITE("Evaluation NBV");

    boost::shared_ptr<IKTest> testPtr(new IKTest());

    //evaluation->add(BOOST_CLASS_TEST_CASE(&MultiSceneTest::visualizeSingleObjectWithNormals, testPtr));
    evaluation->add(BOOST_CLASS_TEST_CASE(&IKTest::cameraPoseTest, testPtr));

    framework::master_test_suite().add(evaluation);

    return 0;
}

