
/*
 * AbstractTest.cpp
 *
 *  Created on: Mar 17, 2015
 *      Author: florianaumann
 */

#include "next_best_view/test_cases/BaseTest.h"

using namespace next_best_view;

    BaseTest::BaseTest() {
        mMoveBaseClient = MoveBaseClientPtr(new MoveBaseClient("move_base", true));
        mSetInitRobotStateClient = mNodeHandle.serviceClient<SetInitRobotState>("/nbv/set_init_robot_state");
        setPointCloudClient = mNodeHandle.serviceClient<SetAttributedPointCloud>("/nbv/set_point_cloud");
        getPointCloudClient = mNodeHandle.serviceClient<GetAttributedPointCloud>("/nbv/get_point_cloud");
        getNextBestViewClient = mNodeHandle.serviceClient<GetNextBestView>("/nbv/next_best_view");
        updatePointCloudClient = mNodeHandle.serviceClient<UpdatePointCloud>("/nbv/update_point_cloud");
    }

    BaseTest::~BaseTest() {}

    void BaseTest::setInitialPose(const geometry_msgs::Pose &initialPose) {
        MILDRobotStatePtr statePtr = this->getRobotState(initialPose);

        SetInitRobotState sirb;
        sirb.request.robotState.pan = statePtr->pan;
        sirb.request.robotState.tilt = statePtr->tilt;
        sirb.request.robotState.rotation = statePtr->rotation;
        sirb.request.robotState.x = statePtr->x;
        sirb.request.robotState.y = statePtr->y;

        ros::service::waitForService("/nbv/set_init_robot_state", -1);
        if (!mSetInitRobotStateClient.call(sirb)) {
            ROS_ERROR("Failed to call service SetInitRobotState.");
        }
    }

    MILDRobotStatePtr BaseTest::getRobotState(const geometry_msgs::Pose &initialPose) {
        tf::Quaternion q(initialPose.orientation.x, initialPose.orientation.y, initialPose.orientation.z, initialPose.orientation.w);
        tf::Matrix3x3 m(q);
        double yaw, pitch, roll;
        m.getRPY(roll, pitch, yaw);

        MILDRobotStatePtr statePtr(new MILDRobotState(0, 0, yaw, initialPose.position.x, initialPose.position.y));

        return statePtr;
    }

    void BaseTest::waitForEnter() {
        std::string dummy;
        std::cout << "Press ENTER to continue.." << std::endl << ">";
        std::getline(std::cin, dummy);
        std::cout << std::endl;
    }

   SimpleQuaternion BaseTest::euler2Quaternion( const Precision roll,
                  const Precision pitch,
                  const Precision yaw)
    {

        Eigen::AngleAxis<Precision> rollAngle(M_PI*roll/180.0,SimpleVector3::UnitX());
        Eigen::AngleAxis<Precision> pitchAngle(M_PI*pitch/180.0,SimpleVector3::UnitY());
        Eigen::AngleAxis<Precision> yawAngle(M_PI*yaw/180.0,SimpleVector3::UnitZ());

        SimpleQuaternion q = rollAngle*pitchAngle*yawAngle;
        return q;
    }

   SimpleQuaternion BaseTest::ZXZ2Quaternion( const Precision roll,
                  const Precision pitch,
                  const Precision yaw)
    {

        Eigen::AngleAxis<Precision> Z1_Angle(M_PI*roll/180.0,SimpleVector3::UnitZ());
        Eigen::AngleAxis<Precision> X_Angle(M_PI*pitch/180.0,SimpleVector3::UnitX());
        Eigen::AngleAxis<Precision> Z2_Angle(M_PI*yaw/180.0,SimpleVector3::UnitZ());

        SimpleQuaternion q = Z1_Angle*X_Angle*Z2_Angle;
        return q;
    }

