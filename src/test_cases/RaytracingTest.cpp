#define BOOST_TEST_STATIC_LINK

#include <boost/test/included/unit_test.hpp>

#include "next_best_view/test_cases/BaseTest.h"

using namespace asr_next_best_view;
using namespace boost::unit_test;

class RaytracingTest : public BaseTest {

public:
    RaytracingTest() : BaseTest (){
    }

    virtual ~RaytracingTest() {}

    void raytracingTest()
    {
        // get parameters
        std::vector<double> robotPositionList, robotOrientationList, objectPositionList, objectOrientationList;
        if (!this->getParameter("robot_position", robotPositionList))
            return;

        SimpleVector3 robotPosition = TypeHelper::getSimpleVector3(robotPositionList);

        if (!this->getParameter("robot_orientation", robotOrientationList))
            return;

        SimpleQuaternion robotOrientation = TypeHelper::getSimpleQuaternion(robotOrientationList);

        if (!this->getParameter("object_position", objectPositionList))
            return;

        SimpleVector3 objectPosition = TypeHelper::getSimpleVector3(objectPositionList);

        if (!this->getParameter("object_orientation", objectOrientationList))
            return;

        SimpleQuaternion objectOrientation = TypeHelper::getSimpleQuaternion(objectOrientationList);

        ROS_INFO("Setze initiale Pose");
        geometry_msgs::Pose initialPose;
        initialPose.position.x = robotPosition[0];
        initialPose.position.y = robotPosition[1];
        initialPose.position.z = robotPosition[2];

        initialPose.orientation.x = robotOrientation.x();
        initialPose.orientation.y = robotOrientation.y();
        initialPose.orientation.z = robotOrientation.z();
        initialPose.orientation.w = robotOrientation.w();
        this->setInitialPose(initialPose);

        ROS_INFO("Setze Objekt");

        asr_msgs::AsrAttributedPoint element;

        SimpleMatrix3 rotation;
        rotation = Eigen::AngleAxis<Precision>(-90.0 * (M_PI / 180.0), SimpleVector3::UnitX())
                    * Eigen::AngleAxis<Precision>(-90.0 * (M_PI / 180.0), SimpleVector3::UnitY())
                    * Eigen::AngleAxis<Precision>(0.0, SimpleVector3::UnitZ());

        rotation = objectOrientation.toRotationMatrix() * rotation;

        objectOrientation = rotation;

        geometry_msgs::Pose pose;
        pose.position.x = objectPosition[0];
        pose.position.y = objectPosition[1];
        pose.position.z = objectPosition[2];
        pose.orientation.x = objectOrientation.x();
        pose.orientation.y = objectOrientation.y();
        pose.orientation.z = objectOrientation.z();
        pose.orientation.w = objectOrientation.w();

        element.type = "Cup";
        element.pose = pose;
        element.identifier = "0";

        SetAttributedPointCloud apc;
        apc.request.point_cloud.elements.push_back(element);

        if (!mSetPointCloudClient.call(apc.request, apc.response))
        {
            ROS_ERROR("Could not set point cloud");
            return;
        }

        asr_next_best_view::GetNextBestView nbv;
        nbv.request.current_pose = initialPose;

        if (!mGetNextBestViewClient.call(nbv.request, nbv.response)) {
            ROS_ERROR("Something went wrong in next best view");
            return;
        }

        if (nbv.response.found)
        {
            asr_next_best_view::TriggerFrustumVisualization tfv;
            tfv.request.current_pose = nbv.response.resulting_pose;
            mTriggerFrustumVisClient.call(tfv.request, tfv.response);

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


