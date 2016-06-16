
/*
 * AbstractTest.cpp
 *
 *  Created on: Mar 17, 2015
 *      Author: florianaumann
 */

#include "next_best_view/test_cases/BaseTest.h"

using namespace next_best_view;

    BaseTest::BaseTest() {
        mInitPosePub = mNodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 100, false);
        mMoveBaseClient = MoveBaseClientPtr(new MoveBaseClient("move_base", true));
        mSetInitRobotStateClient = mNodeHandle.serviceClient<SetInitRobotState>("/nbv/set_init_robot_state");
        setPointCloudClient = mNodeHandle.serviceClient<SetAttributedPointCloud>("/nbv/set_point_cloud");
        getPointCloudClient = mNodeHandle.serviceClient<GetAttributedPointCloud>("/nbv/get_point_cloud");
        getNextBestViewClient = mNodeHandle.serviceClient<GetNextBestView>("/nbv/next_best_view");
        updatePointCloudClient = mNodeHandle.serviceClient<UpdatePointCloud>("/nbv/update_point_cloud");
    }

    BaseTest::~BaseTest() {}

    void BaseTest::visualizeSingleObjectWithNormals() {
        ros::Publisher pubOne = mNodeHandle.advertise<visualization_msgs::Marker>("/visualization_marker", 100, false);
        ros::Publisher pub = mNodeHandle.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 100, false);

        visualization_msgs::MarkerArray markerArray;
        ObjectHelper objectHelper;
        ObjectMetaDataResponsePtr objectMetaDataResPtr = objectHelper.getObjectMetaData("Smacks");

        ros::Duration(5).sleep();

        visualization_msgs::Marker deleteMarker = MarkerHelper::getBasicMarker(0);

        deleteMarker.action = 3u;
        pubOne.publish(deleteMarker);


        ros::Duration(5).sleep();
        int id = 0;
        SimpleVector3 zero(0, 0, 0);
        SimpleQuaternion rotation = MathHelper::getQuaternionByAngles(0, 0, -M_PI / 2.0);

        Indices ind;
        for (std::size_t idx = 0; idx < objectMetaDataResPtr->normal_vectors.size(); idx++) {
            geometry_msgs::Point pt = objectMetaDataResPtr->normal_vectors.at(idx);
            SimpleVector3 pos = TypeHelper::getSimpleVector3(pt);
            bool rebel = false;
            BOOST_FOREACH(std::size_t index, ind) {
                geometry_msgs::Point pt2 = objectMetaDataResPtr->normal_vectors.at(index);
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
            geometry_msgs::Point pt = objectMetaDataResPtr->normal_vectors.at(index);
            SimpleVector3 pos = TypeHelper::getSimpleVector3(pt);
            pos = rotation.toRotationMatrix() * pos;
            visualization_msgs::Marker arrowMarker = MarkerHelper::getArrowMarker(id, zero, pos, SimpleVector3(0.025, 0.05, 0.05), SimpleVector4(0.3, 0.3, 1.0, 1.0));
            markerArray.markers.push_back(arrowMarker);
            id++;

        }

        visualization_msgs::Marker objectMarker = MarkerHelper::getMeshMarker(id, objectMetaDataResPtr->object_mesh_resource, zero, rotation);
        markerArray.markers.push_back(objectMarker);

        pub.publish(markerArray);

        ros::spinOnce();
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

    void BaseTest::moveToPose(const geometry_msgs::Pose &pose) {

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

    void BaseTest::visualizeSpaceSampling() {
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

