
/*
 *
 *  Created on: Nov 6, 2015
 *      Author: florianaumann
 */
#define BOOST_TEST_STATIC_LINK

#include <boost/test/included/unit_test.hpp>
#include "next_best_view/test_cases/BaseTest.h"
#include "next_best_view/robot_model/impl/MILDRobotModelWithExactIK.hpp"
#include "next_best_view/robot_model/impl/MILDRobotModel.hpp"
#include "next_best_view/robot_model/impl/MILDRobotState.hpp"
#include "next_best_view/GetAttributedPointCloud.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

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
        targetCameraPositions.push_back(SimpleVector3(0.404993116856, -2.28920769691, 1.4));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-170, 0.0, 0 ));
        targetCameraPositions.push_back(SimpleVector3(0.404993116856, -2.28920769691, 1.4));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-170, 80.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.404993116856, -2.28920769691, 1.4));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-180, 100.0, 0 ));
        targetCameraPositions.push_back(SimpleVector3(0.404993116856, -2.28920769691, 1.4));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-200, 40.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.404993116856, -2.28920769691, 1.4));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-180, -30.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.404993116856, -2.28920769691, 1.4));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-190, -50.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.404993116856, -2.28920769691, 1.4));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-120, -20.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.404993116856, -2.28920769691, 1.4));
        targetCameraOrientations.push_back(ZXZ2Quaternion(120, -20.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.404993116856, -2.28920769691, 1.5));
        targetCameraOrientations.push_back(ZXZ2Quaternion(20, -20.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.404993116856, -2.28920769691, 0.8));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-10, 0.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.404993116856, -2.28920769691, 1.7));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-170, 0.0, 0 ));
        targetCameraPositions.push_back(SimpleVector3(0.404993116856, -2.28920769691, 0.4));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-170, 80.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.404993116856, -2.28920769691, 0.5));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-180, 100.0, 0 ));
        targetCameraPositions.push_back(SimpleVector3(0.404993116856, -2.28920769691, 1.0));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-200, 40.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.404993116856, -2.28920769691, 1.8));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-180, -30.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.404993116856, -2.28920769691, 1.3));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-190, -50.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.404993116856, -2.28920769691, 1.1));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-120, -20.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.404993116856, -2.28920769691, 1.7));
        targetCameraOrientations.push_back(ZXZ2Quaternion(120, -20.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.404993116856, -2.28920769691, 1.0));
        targetCameraOrientations.push_back(ZXZ2Quaternion(20, -20.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.404993116856, -2.28920769691, 0.9));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-10, 0.0, 0));
        //a couple of very difficult test cases
        targetCameraPositions.push_back(SimpleVector3(0.404993116856, -2.28920769691, 2.5));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-20, -20.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.404993116856, -2.28920769691, 0.0));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-10, 30.0, 0));
        targetCameraPositions.push_back(SimpleVector3(0.404993116856, -2.28920769691, -1.0));
        targetCameraOrientations.push_back(ZXZ2Quaternion(-10, 30.0, 0));


        ROS_INFO_STREAM("Initializing...");

        //Initialize robot model
        MILDRobotModelWithExactIK *myRobotModel = new MILDRobotModelWithExactIK();
        MILDRobotModelWithExactIKPtr myRobotModelPtr(myRobotModel);
        double position_x = 0.013948535919;
        double position_y = -1.60322499275;
        double rotationAngle = -1.57;

        MILDRobotState * startState = new MILDRobotState(0,0,rotationAngle,position_x,position_y);
        MILDRobotStatePtr startStatePtr(startState);

        //Publish current pose
        ros::NodeHandle n;
        ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000);
        geometry_msgs::PoseWithCovarianceStamped startPoseStamped = geometry_msgs::PoseWithCovarianceStamped();
        geometry_msgs::PoseWithCovariance startPose = geometry_msgs::PoseWithCovariance();
        SimpleQuaternion rotation = ZXZ2Quaternion(0, 0, rotationAngle*180/M_PI);
        startPose.pose.position.x = position_x;
        startPose.pose.position.y = position_y;
        startPose.pose.position.z = 0;
        startPose.pose.orientation.w = rotation.w();
        startPose.pose.orientation.x = rotation.x();
        startPose.pose.orientation.y = rotation.y();
        startPose.pose.orientation.z = rotation.z();
        startPoseStamped.pose = startPose;
        startPoseStamped.header.stamp = ros::Time::now();
        pose_pub.publish(startPoseStamped);


        myRobotModelPtr->setPanAngleLimits(-180, 180);
        myRobotModelPtr->setTiltAngleLimits(-180, 180);
        ROS_INFO_STREAM("Running test...");
        ros::spinOnce();
        for (unsigned int i = 0; i < targetCameraPositions.size(); i++)
        {
            ROS_INFO_STREAM("Testpose " << (i+1));
            SimpleVector3 currentPosition = targetCameraPositions[i];
            SimpleQuaternion currentOrientation = targetCameraOrientations[i];
            //ROS_INFO_STREAM("Calculating inverse kinematics...");
            //RobotStatePtr newStatePtr = myRobotModelPtr->calculateRobotState(startStatePtr, currentPosition, currentOrientation);
            ROS_INFO_STREAM("Calculating camera pose correction...");
            PTUConfig ptuConfig = myRobotModelPtr->calculateCameraPoseCorrection(startStatePtr, currentPosition, currentOrientation);
            ROS_INFO_STREAM("Got pan = " << std::get<0>(ptuConfig)*180/M_PI << ", tilt = " << std::get<1>(ptuConfig)*180/M_PI);
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

	test_suite* evaluation = BOOST_TEST_SUITE("Evaluation NBV");

    boost::shared_ptr<IKTest> testPtr(new IKTest());

    evaluation->add(BOOST_CLASS_TEST_CASE(&IKTest::cameraPoseTest, testPtr));

    framework::master_test_suite().add(evaluation);

    return 0;
}

