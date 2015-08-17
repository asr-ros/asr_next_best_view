
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
    }

    BaseTest::~BaseTest() {}

    /*!
     * \brief evaluates the correctness of the Sphere To Cartesian and Cartesian To Sphere Methods.
     */
    void BaseTest::evaluateS2CandC2S() {
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
                //BOOST_REQUIRE(cerror <= tolerance);
            }
        }
    }

    void BaseTest::solveLinearProblem() {
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

    void BaseTest::visualizeSingleObjectWithNormals() {
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

   SimpleQuaternion  BaseTest::euler2Quaternion( const Precision roll,
                  const Precision pitch,
                  const Precision yaw)
    {

        Eigen::AngleAxis<Precision> rollAngle(M_PI*roll/180.0,SimpleVector3::UnitX());
        Eigen::AngleAxis<Precision> pitchAngle(M_PI*pitch/180.0,SimpleVector3::UnitY());
        Eigen::AngleAxis<Precision> yawAngle(M_PI*yaw/180.0,SimpleVector3::UnitZ());

        SimpleQuaternion q = rollAngle*pitchAngle*yawAngle;
        return q;
    }


