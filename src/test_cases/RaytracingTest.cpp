#define BOOST_TEST_STATIC_LINK

#include <boost/test/included/unit_test.hpp>

#include "next_best_view/test_cases/BaseTest.h"

using namespace next_best_view;
using namespace boost::unit_test;

class RaytracingTest : public BaseTest {

public:
    RaytracingTest() : BaseTest (){
    }

    virtual ~RaytracingTest() {}

    void raytracingTest()
    {
        // get parameters
        std::vector<double> positionList, orientationList;
        if (!this->getParameter("position", positionList))
            return;

        SimpleVector3 position = TypeHelper::getSimpleVector3(positionList);

        if (!this->getParameter("orientation", orientationList))
            return;

        SimpleQuaternion orientation = TypeHelper::getSimpleQuaternion(orientationList);

        ROS_INFO("Setze initiale Pose");
        geometry_msgs::Pose initialPose;
        initialPose.position.x = 0.0;
        initialPose.position.y = 0.0;
        initialPose.position.z = 1.32;

        initialPose.orientation.x = 0.0;
        initialPose.orientation.y = 0.0;
        initialPose.orientation.z = 0.0;
        initialPose.orientation.w = 1.0;
        this->setInitialPose(initialPose);

        ROS_INFO("Setze Objekt");

        pbd_msgs::PbdAttributedPoint element;

        SimpleMatrix3 rotation;
        rotation = Eigen::AngleAxis<Precision>(180.0 * (M_PI / 180.0), SimpleVector3::UnitX())
                    * Eigen::AngleAxis<Precision>(0.0 * (M_PI / 180.0), SimpleVector3::UnitY())
                    * Eigen::AngleAxis<Precision>(0.0, SimpleVector3::UnitZ());

        rotation = orientation.toRotationMatrix() * rotation;

        orientation = rotation;

        geometry_msgs::Pose pose;
        pose.position.x = position[0];
        pose.position.y = position[1];
        pose.position.z = position[2];
        pose.orientation.x = orientation.x();
        pose.orientation.y = orientation.y();
        pose.orientation.z = orientation.z();
        pose.orientation.w = orientation.w();

        element.type = "CeylonTea";
        element.pose = pose;
        element.identifier = "0";

        SetAttributedPointCloud apc;
        apc.request.point_cloud.elements.push_back(element);

        if (!mSetPointCloudClient.call(apc.request, apc.response))
        {
            ROS_ERROR("Could not set point cloud");
            return;
        }

        GetNextBestView nbv;
        nbv.request.current_pose = initialPose;

        if (!mGetNextBestViewClient.call(nbv.request, nbv.response)) {
            ROS_ERROR("Something went wrong in next best view");
            return;
        }

        if (nbv.response.found)
        {
            ROS_INFO("Found NBV");
        }
        else
        {
            ROS_ERROR("No NBV found");
        }
    }
};

test_suite* init_unit_test_suite( int argc, char* argv[] ) {
    ros::init(argc, argv, "nbv_test");
    ros::start();

    ros::Duration(5).sleep();

    test_suite* evaluation = BOOST_TEST_SUITE("Evaluation NBV");

    boost::shared_ptr<RaytracingTest> testPtr(new RaytracingTest());

    evaluation->add(BOOST_CLASS_TEST_CASE(&RaytracingTest::raytracingTest, testPtr));

    framework::master_test_suite().add(evaluation);

    return 0;
}


