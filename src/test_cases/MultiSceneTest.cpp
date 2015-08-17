
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

class MultiSceneTest : public BaseTest{
public:
    MultiSceneTest() : BaseTest (){
    }

    virtual ~MultiSceneTest() {}

	void iterationTest() {
		ros::ServiceClient setPointCloudClient = mNodeHandle.serviceClient<SetAttributedPointCloud>("/nbv/set_point_cloud");
		ros::ServiceClient getPointCloud2Client = mNodeHandle.serviceClient<GetPointCloud2>("/nbv/get_point_cloud2");
		ros::ServiceClient getNextBestViewClient = mNodeHandle.serviceClient<GetNextBestView>("/nbv/next_best_view");
		ros::ServiceClient updatePointCloudClient = mNodeHandle.serviceClient<UpdatePointCloud>("/nbv/update_point_cloud");
		ros::ServiceClient getSpaceSamplingClient = mNodeHandle.serviceClient<GetSpaceSampling>("/nbv/get_space_sampling");

		SetAttributedPointCloud apc;

		ROS_INFO("Generiere Häufungspunkte");
		// Häufungspunkte
        int hpSize = 9;
        SimpleVector3* hp = new SimpleVector3[hpSize];
        hp[0] = SimpleVector3(0.725892364979, 1.57344818115, 0.8);
        hp[1] = SimpleVector3(1.0500099659, 1.63358879089,  0.8);
        hp[2] = SimpleVector3(1.74442279339,-0.276624619961,  0.8);
        hp[3] = SimpleVector3(1.64329886436, -0.350318551064,  0.8);
        hp[4] = SimpleVector3(-0.998491704464, 0.463356167078,  0.8);
        hp[5] = SimpleVector3(-0.972244024277,  0.759688913822,  0.8);
        hp[6] = SimpleVector3(-0.516409695148,-0.863412082195, 1.3);
        hp[7] = SimpleVector3(-0.516409695148,-0.863412082195, 1.3);
        hp[8] = SimpleVector3(-0.516409695148,-0.863412082195, 1.7);
	

        SimpleQuaternion* orientation = new SimpleQuaternion[hpSize];
        orientation[0] = SimpleQuaternion(0.0, 0.0, 0.0, 1.0);
        orientation[1] = SimpleQuaternion(0.0, 0.0, 0.0, 1.0);
        orientation[2] = SimpleQuaternion(0.0, 0.0, 0.0, 1.0);
        orientation[3] = SimpleQuaternion(0.0, 0.0, 0.0, 1.0);
        orientation[4] = SimpleQuaternion(0.0, 0.0, 0.0, 1.0);
        orientation[5] = SimpleQuaternion(0.0, 0.0, 0.0, 1.0);
        orientation[6] = SimpleQuaternion(0.0, 0.0, 0.0, 1.0);
        orientation[7] = SimpleQuaternion(0.70711,0.0, 0.0, -0.70711);
        orientation[8] = SimpleQuaternion(0.0, 0.0, 0.0, 1.0);
        /*orientation[0] = SimpleQuaternion(0.658806053033,0.0, 0.0, 0.752312823556);
        orientation[1] = SimpleQuaternion(0.658806053033, 0.0, 0.0, 0.752312823556);
        orientation[2] = SimpleQuaternion(0.999847878074, 0.0, 0.0, -0.0174419239707);
        orientation[3] = SimpleQuaternion(0.695335180664, 0.0, 0.0,-0.718685596441);*/
        std::string* types = new std::string[hpSize];
        types[0] = "Cup";
        types[1] = "PlateDeep";
        types[2] = "Coffeebox";
        types[3] = "CoffeeFilters2";
        types[4] = "Smacks";
        types[5] = "VitalisSchoko";
        types[6] = "CeylonTea";
        types[7] = "CeylonTea";
        types[8] = "CeylonTea";

       int sampleSize = 10;
       std::map<std::string, std::vector<AttributedPoint>* > objectPointCloudsMap;

		for (std::size_t idx = 0; idx < hpSize; idx++) {

            if(objectPointCloudsMap.find(types[idx]) == objectPointCloudsMap.end())
            {
                objectPointCloudsMap[types[idx]]= new std::vector<AttributedPoint>();
            }
			for (std::size_t cnt = 0; cnt < sampleSize; cnt++) {
				SimpleVector3 randomVector;
                randomVector = MathHelper::getRandomVector(hp[idx], SimpleVector3(.05, .05, 0.01));

                AttributedPoint element;

				geometry_msgs::Pose pose;
                pose.orientation.w = orientation[idx].w();
                pose.orientation.x = orientation[idx].x();
                pose.orientation.y = orientation[idx].y();
                pose.orientation.z = orientation[idx].z();
				pose.position.x = randomVector[0];
				pose.position.y = randomVector[1];
				pose.position.z = randomVector[2];

                element.object_type = types[idx];
                element.pose = pose;
                objectPointCloudsMap[types[idx]]->push_back(element);
				apc.request.point_cloud.elements.push_back(element);
			}
		}

		ROS_INFO("Setze initiale Pose");
		geometry_msgs::Pose initialPose;
        initialPose.position.x = -0.974955737591;
        initialPose.position.y = -0.157173499465;
		initialPose.position.z = 1.32;

        initialPose.orientation.w = 0.718685498907;
		initialPose.orientation.x = 0.0;
        initialPose.orientation.y = 0.0;
        initialPose.orientation.z = 0.695335281472;
		this->setInitialPose(initialPose);

		apc.request.pose = initialPose;

		// Setze PointCloud
		setPointCloudClient.call(apc.request, apc.response);
        ros::Rate r(2);
		GetNextBestView nbv;
		nbv.request.current_pose = initialPose;
		ViewportPointCloudPtr viewportPointCloudPtr(new ViewportPointCloud());
		int x = 1;
		while(ros::ok()) {
            std::string yolo;
            std::cin >> yolo;
            if(apc.request.point_cloud.elements.size() == 0)
            {
              break;
            }
			ROS_INFO("Kalkuliere NBV (%d)", x);
			if (!getNextBestViewClient.call(nbv.request, nbv.response)) {
				ROS_ERROR("Something went wrong in next best view");
				break;
			}

            if (nbv.response.object_name_list.size() > 0)
            {
                apc.request.point_cloud.elements.clear();
                for(int i=0;i<nbv.response.object_name_list.size();i++)
                {
                    if (nbv.response.object_name_list[i] != "CeylonTea") objectPointCloudsMap.erase(nbv.response.object_name_list[i]);
                }
                for (std::map<std::string, std::vector<AttributedPoint>* >::iterator it = objectPointCloudsMap.begin() ; it != objectPointCloudsMap.end(); ++it)
                {
                    apc.request.point_cloud.elements.insert(apc.request.point_cloud.elements.end(),(it->second)->begin(),(it->second)->end());
                }
                setPointCloudClient.call(apc.request, apc.response);
            }
            ROS_INFO_STREAM("nbv.response.resulting_pose:" << nbv.response.resulting_pose);
            //ROS_INFO_STREAM("nbv.response.robot_state:" << nbv.response.robot_state);
            ROS_INFO_STREAM("nbv.response.found:" << nbv.response.found);

			if (!nbv.response.found) {
                ROS_ERROR("No NBV found");
				break;
			}

            UpdatePointCloud upc_req;
            upc_req.request.update_pose = nbv.response.resulting_pose;

            if(!updatePointCloudClient.call(upc_req)) {
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

test_suite* init_unit_test_suite( int argc, char* argv[] ) {
	ros::init(argc, argv, "nbv_test");
	ros::start();

	ros::Duration(5).sleep();

	test_suite* evaluation = BOOST_TEST_SUITE("Evaluation NBV");

    boost::shared_ptr<MultiSceneTest> testPtr(new MultiSceneTest());

    evaluation->add(BOOST_CLASS_TEST_CASE(&MultiSceneTest::iterationTest, testPtr));

    framework::master_test_suite().add(evaluation);

    return 0;
}

