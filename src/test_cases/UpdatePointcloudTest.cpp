
/*
 * UpdatePointCloudTest.cpp
 *
 *  Created on: Jun 10, 2016
 *      Author: Daniel Stroh
 */
#define BOOST_TEST_STATIC_LINK

#include <boost/test/included/unit_test.hpp>
#include "next_best_view/test_cases/BaseTest.h"
#include "next_best_view/GetAttributedPointCloud.h"

using namespace next_best_view;
using namespace boost::unit_test;

class UpdatePointCloudTest : public BaseTest{
public:
    UpdatePointCloudTest(bool silent) : BaseTest (true, silent) { }

    virtual ~UpdatePointCloudTest() {}

    int test(SetAttributedPointCloud apc, std::vector<std::string> object_type_name_list) {
        ROS_INFO("Setze initiale Pose");
        geometry_msgs::Pose initialPose;
        initialPose.position.x = -0.383223;
        initialPose.position.y = 0.157997;
        initialPose.position.z = 1.32;
        initialPose.orientation.x = -0.17266;
        initialPose.orientation.y = 0.2115588;
        initialPose.orientation.z = 0.60825979;
        initialPose.orientation.w = 0.74528563;
        this->setInitialPose(initialPose);

        // Setze PointCloud
        if (!mSetPointCloudClient.call(apc.request, apc.response)) {
            ROS_ERROR("Could not set initial point cloud.");
        }

        GetAttributedPointCloud gpc;
        if (!mGetPointCloudClient.call(gpc)) {
            ROS_ERROR("Could not get initial point cloud.");
        }

        GetNextBestView nbv;
        nbv.request.current_pose = initialPose;

        int i = 1;
        int deactivatedNormals = 0;
        while(ros::ok() && i <= 3) {
            ROS_INFO_STREAM("Kalkuliere NBV " << i);
            if (!mGetNextBestViewClient.call(nbv.request, nbv.response)) {
                ROS_ERROR("Something went wrong in next best view");
                break;
            }

            UpdatePointCloud upc_req;
            upc_req.request.object_type_name_list = object_type_name_list;
            upc_req.request.pose_for_update = nbv.response.resulting_pose;
            if(!mUpdatePointCloudClient.call(upc_req)) {
                ROS_ERROR("Update Point Cloud failed!");
                break;
            }
            deactivatedNormals += upc_req.response.deactivated_object_normals;
            this->waitForEnter();
            i++;
        }
        return deactivatedNormals;
    }

    void iterationTest() {
        SetAttributedPointCloud apc;

        ROS_INFO("Generiere Häufungspunkte");
        // anzahl Häufungspunkte
        int nHp = 2;
        SimpleVector3* location = new SimpleVector3[nHp];
        location[0] = SimpleVector3(-0.43232, 0.6958, 0.7211);
        location[1] = SimpleVector3(-0.283309, 0.710407,  0.718543);

        SimpleQuaternion* orientation = new SimpleQuaternion[nHp];
        orientation[0] = euler2Quaternion(-90, 0.0, 0.0);
        orientation[1] = euler2Quaternion(-90, 0.0, 0.0);

        std::string* types = new std::string[nHp];
        types[0] = "Smacks";
        types[1] = "PlateDeep";

        int elementsPerHp[2] = { 3, 1 };

        for (std::size_t idx = 0; idx < (std::size_t)nHp; idx++) {
            for (std::size_t cnt = 0; cnt < (std::size_t)elementsPerHp[idx]; cnt++) {
                SimpleVector3 curLocation = location[idx];

                pbd_msgs::PbdAttributedPoint element;

                geometry_msgs::Pose pose;
                pose.orientation.w = orientation[idx].w();
                pose.orientation.x = orientation[idx].x();
                pose.orientation.y = orientation[idx].y();
                pose.orientation.z = orientation[idx].z();
                pose.position.x = curLocation[0] - 0.1 * cnt; // put smacks in row next to each other
                pose.position.y = curLocation[1];
                pose.position.z = curLocation[2];

                element.type = types[idx];
                element.identifier = std::to_string(idx);
                element.pose = pose;
                apc.request.point_cloud.elements.push_back(element);
            }
        }

        int deactivatedSmacksNormals = test(apc, {"Smacks"});
        int deactivatedSmacksPlateDeepNormals = test(apc, {"Smacks", "PlateDeep"});
        BOOST_CHECK_MESSAGE(deactivatedSmacksNormals < deactivatedSmacksPlateDeepNormals, "Update deactivated the same or more normals without PlateDeep");
        ROS_INFO("deactivated normals [Smacks, PlateDeep]: %d", deactivatedSmacksPlateDeepNormals);
        ROS_INFO("deactivated normals [Smacks]: %d", deactivatedSmacksNormals);
    }
};

test_suite* init_unit_test_suite( int argc, char* argv[] ) {
    ros::init(argc, argv, "nbv_test");
    ros::start();

    ros::Duration(5).sleep();

    test_suite* evaluation = BOOST_TEST_SUITE("Evaluation NBV");

    bool silent = false;
    if (argc >= 2) {
        cout << argv[1];
        // caseinsensitive comparison
        if (boost::iequals(argv[1], "silent") || boost::iequals(argv[1], "s")) {
            silent = true;
        }
    }
    boost::shared_ptr<UpdatePointCloudTest> testPtr(new UpdatePointCloudTest(silent));

    evaluation->add(BOOST_CLASS_TEST_CASE(&UpdatePointCloudTest::iterationTest, testPtr));

    framework::master_test_suite().add(evaluation);

    return 0;
}




