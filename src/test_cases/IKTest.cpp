
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

    void camerPoseTest() {
        std::vector<SimpleVector3> targetCameraPositions;
        std::vector<SimpleQuaternion> targetCameraOrientations;
        //Initialize Poses
        targetCameraPositions.push_back(SimpleVector3(0.725892364979, 1.57344818115, 0.8));
        targetCameraOrientations.push_back(euler2Quaternion(-90, 0.0, 0.0));
        targetCameraPositions.push_back(SimpleVector3(0.725892364979, 1.67344818115, 0.8));
        targetCameraOrientations.push_back(euler2Quaternion(-100, 0.0, 0.0));

        //Initialize robot model
        next_best_view::MILDRobotModelWithIKPtr myRobotModel;
        MILDRobotStatePtr startState;
        startState->pan = 0;
        startState->tilt = 0;
        startState->rotation = 0;
        startState->x = 0;
        startState->y = 0;

        for (unsigned int i = 0; i < targetCameraPositions.size(); i++)
        {
            SimpleVector3 currentPosition = targetCameraPositions[i];
            SimpleQuaternion currentOrientation = targetCameraOrientations[i];
            RobotStatePtr newState = myRobotModel->calculateRobotState(startState, currentPosition, currentOrientation);
            ros::spinOnce();
            waitForEnter();
            ros::Duration(2).sleep();
        }
	}
};

test_suite* init_unit_test_suite( int argc, char* argv[] ) {
	ros::init(argc, argv, "nbv_test");
	ros::start();

	ros::Duration(5).sleep();

	test_suite* evaluation = BOOST_TEST_SUITE("Evaluation NBV");

    boost::shared_ptr<IKTest> testPtr(new IKTest());

    //evaluation->add(BOOST_CLASS_TEST_CASE(&MultiSceneTest::visualizeSingleObjectWithNormals, testPtr));
    evaluation->add(BOOST_CLASS_TEST_CASE(&IKTest::camerPoseTest, testPtr));

    framework::master_test_suite().add(evaluation);

    return 0;
}

