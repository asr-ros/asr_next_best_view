/**

Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Braun Kai, Heller Florian, Hutmacher Robin, Karrenbauer Oliver, Marek Felix, Mayr Matthias, Mehlhaus Jonas, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Stroh Daniel, Trautmann Jeremias, Walter Milena
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

4. The use is explicitly not permitted to any application which deliberately try to kill or do harm to any living creature.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "next_best_view/test_cases/BaseTest.h"

using namespace next_best_view;

    BaseTest::BaseTest() {
        initRosServicesAndPublishers();
    }

    BaseTest::BaseTest(bool useRos, bool silent) {
        this->silent = silent;
        if (useRos) {
            initRosServicesAndPublishers();
        }
    }

    BaseTest::~BaseTest() {}

    void BaseTest::initRosServicesAndPublishers() {
        this->mNodeHandle = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());

        // services
        ros::service::waitForService("/nbv/set_init_robot_state", -1);
        mSetInitRobotStateClient = mNodeHandle->serviceClient<SetInitRobotState>("/nbv/set_init_robot_state");
        ros::service::waitForService("/nbv/set_point_cloud", -1);
        mSetPointCloudClient = mNodeHandle->serviceClient<SetAttributedPointCloud>("/nbv/set_point_cloud");
        ros::service::waitForService("/nbv/get_point_cloud", -1);
        mGetPointCloudClient = mNodeHandle->serviceClient<GetAttributedPointCloud>("/nbv/get_point_cloud");
        ros::service::waitForService("/nbv/next_best_view", -1);
        mGetNextBestViewClient = mNodeHandle->serviceClient<GetNextBestView>("/nbv/next_best_view");
        ros::service::waitForService("/nbv/update_point_cloud", -1);
        mUpdatePointCloudClient = mNodeHandle->serviceClient<UpdatePointCloud>("/nbv/update_point_cloud");
        ros::service::waitForService("/nbv/reset_nbv_calculator", -1);
        mResetCalculatorClient = mNodeHandle->serviceClient<ResetCalculator>("/nbv/reset_nbv_calculator");

        // publishers
        mInitPosePub = mNodeHandle->advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 100, false);
    }

    void BaseTest::setInitialPose(const geometry_msgs::Pose &initialPose) {
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

        this->setInitialRobotState(initialPose);
    }

    void BaseTest::setInitialRobotState(const geometry_msgs::Pose &initialPose) {
        MILDRobotStatePtr statePtr = this->getRobotState(initialPose);

        SetInitRobotState sirb;
        sirb.request.robotState.pan = statePtr->pan;
        sirb.request.robotState.tilt = statePtr->tilt;
        sirb.request.robotState.rotation = statePtr->rotation;
        sirb.request.robotState.x = statePtr->x;
        sirb.request.robotState.y = statePtr->y;

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
        if (silent) {
            return;
        }
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

