/*
 * TestRecognitionManager.h
 *
 *  Created on: Mar 15, 2014
 *      Author: ralfschleicher
 */
#define BOOST_TEST_STATIC_LINK

#include <boost/test/included/unit_test.hpp>

#include "next_best_view/test_cases/BaseTest.h"

using namespace next_best_view;
using namespace boost::unit_test;

class Test : public BaseTest {
public:
    Test() : BaseTest (){
	}

	virtual ~Test() {}

    void iterationTest() {

		SetAttributedPointCloud apc;

		ROS_INFO("Generiere Häufungspunkte");
		// Häufungspunkte
        std::size_t hpSize = 3;
		SimpleVector3* hp = new SimpleVector3[hpSize];
		hp[0] = SimpleVector3(16.0, 16.0, 0.98);
		hp[1] = SimpleVector3(25.0, 11.0, 1.32);
		hp[2] = SimpleVector3(18.6, 7.3, 1.80);

		std::vector<std::string> objectTypeNames;
		objectTypeNames.push_back("Smacks");
		objectTypeNames.push_back("CoffeeBox");
		objectTypeNames.push_back("Vitalis");

        std::size_t sampleSize = 200;

		MapHelper mapHelper;

		for (std::size_t idx = 0; idx < hpSize; idx++) {
			for (std::size_t cnt = 0; cnt < sampleSize; cnt++) {
				SimpleVector3 randomVector;
				int8_t occupancyValue;
				do {
					randomVector = MathHelper::getRandomVector(hp[idx], SimpleVector3(.5, .5, 0.1));
					occupancyValue = mapHelper.getRaytracingMapOccupancyValue(randomVector);
				} while(!mapHelper.isOccupancyValueAcceptable(occupancyValue));

				SimpleQuaternion randomQuat = next_best_view::MathHelper::getRandomQuaternion();

				pbd_msgs::PbdAttributedPoint element;

				geometry_msgs::Pose pose;
				pose.orientation.w = randomQuat.w();
				pose.orientation.x = randomQuat.x();
				pose.orientation.y = randomQuat.y();
				pose.orientation.z = randomQuat.z();
				pose.position.x = randomVector[0];
				pose.position.y = randomVector[1];
				pose.position.z = randomVector[2];

                element.type = objectTypeNames[next_best_view::MathHelper::getRandomInteger(0, objectTypeNames.size() - 1)];
				element.pose = pose;

				apc.request.point_cloud.elements.push_back(element);
			}
		}

		ROS_INFO("Setze initiale Pose");
		geometry_msgs::Pose initialPose;
		initialPose.position.x = 2.45717;
		initialPose.position.y = 22.276;
		initialPose.position.z = 1.32;

		initialPose.orientation.w = 0.971467;
		initialPose.orientation.x = 0.0;
		initialPose.orientation.y = 0.0;
		initialPose.orientation.z = -0.237177;
		this->setInitialPose(initialPose);

		// Setze PointCloud
        mSetPointCloudClient.call(apc.request, apc.response);

		GetNextBestView nbv;
        nbv.request.current_pose = initialPose;
		int x = 1;
		while(ros::ok()) {
            ROS_INFO_STREAM("Kalkuliere NBV " << x);
            if (!mGetNextBestViewClient.call(nbv.request, nbv.response)) {
				ROS_ERROR("Something went wrong in next best view");
				break;
			}

			if (!nbv.response.found) {
                ROS_ERROR("No NBV found");
				break;
			}

            UpdatePointCloud upc_req;
            upc_req.request.object_type_name_list = nbv.response.object_type_name_list;
            upc_req.request.pose_for_update = nbv.response.resulting_pose;

            if(!mUpdatePointCloudClient.call(upc_req)) {
                ROS_ERROR("Update Point Cloud failed!");
                break;
            }

			x++;

			nbv.request.current_pose = nbv.response.resulting_pose;

			ros::spinOnce();
			waitForEnter();
			ros::Duration(2).sleep();
        }
	}


};

test_suite* init_unit_test_suite( int argc, char* argv[] )
{
	ros::init(argc, argv, "nbv_test");
	ros::start();

	ros::Duration(5).sleep();

	test_suite* evaluation = BOOST_TEST_SUITE("Evaluation NBV");

	boost::shared_ptr<Test> testPtr(new Test());

	evaluation->add(BOOST_CLASS_TEST_CASE(&Test::iterationTest, testPtr));

    framework::master_test_suite().add(evaluation);

    return 0;
}

