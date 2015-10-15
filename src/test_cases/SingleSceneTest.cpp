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

class SingleSceneTest : public BaseTest {

public:
    SingleSceneTest() : BaseTest (){
    }

    virtual ~SingleSceneTest() {}

	void iterationTest() {
		ros::ServiceClient setPointCloudClient = mNodeHandle.serviceClient<SetAttributedPointCloud>("/nbv/set_point_cloud");
        ros::ServiceClient getPointCloudClient = mNodeHandle.serviceClient<GetAttributedPointCloud>("/nbv/get_point_cloud");
		ros::ServiceClient getPointCloud2Client = mNodeHandle.serviceClient<GetPointCloud2>("/nbv/get_point_cloud2");
		ros::ServiceClient getNextBestViewClient = mNodeHandle.serviceClient<GetNextBestView>("/nbv/next_best_view");
		ros::ServiceClient updatePointCloudClient = mNodeHandle.serviceClient<UpdatePointCloud>("/nbv/update_point_cloud");
		ros::ServiceClient getSpaceSamplingClient = mNodeHandle.serviceClient<GetSpaceSampling>("/nbv/get_space_sampling");

        GetAttributedPointCloud gpc;
		SetAttributedPointCloud apc;

		ROS_INFO("Generiere Häufungspunkte");
		// Häufungspunkte
        int hpSize = 4;
		SimpleVector3* hp = new SimpleVector3[hpSize];
        hp[0] = SimpleVector3(-1.35357105732, 1.06068396568,0.8);
        hp[1] = SimpleVector3(-1.05733001232, 1.06068396568, 0.8);
        hp[2] = SimpleVector3(-1.03866374493, 0.772554755211, 0.8);
        hp[3] = SimpleVector3(-1.35400700569, 0.420550823212, 0.8);

        SimpleQuaternion* orientation = new SimpleQuaternion[hpSize];
        orientation[0] = euler2Quaternion(-90, 180.0, 0.0);
        orientation[1] = euler2Quaternion(-90, 180.0, 0.0);
        orientation[2] = euler2Quaternion(-90, -90.0, 0.0);
        orientation[3] = euler2Quaternion(-90, 0.0, 0.0);
        /*orientation[0] = SimpleQuaternion(0.658806053033,0.0, 0.0, 0.752312823556);
        orientation[1] = SimpleQuaternion(0.658806053033, 0.0, 0.0, 0.752312823556);
        orientation[2] = SimpleQuaternion(0.999847878074, 0.0, 0.0, -0.0174419239707);
        orientation[3] = SimpleQuaternion(0.695335180664, 0.0, 0.0,-0.718685596441);*/
        std::string* types = new std::string[hpSize];
        types[0] = "Knaeckebrot";
        types[1] = "VitalisSchoko";
        types[2] = "Coffeebox";
        types[3] = "Smacks";

       int sampleSize = 2;
       std::map<std::string, std::vector<pbd_msgs::PbdAttributedPoint>* > objectPointCloudsMap;

		for (std::size_t idx = 0; idx < hpSize; idx++) {

            if(objectPointCloudsMap.find(types[idx]) == objectPointCloudsMap.end())
            {
	      objectPointCloudsMap[types[idx]]= new std::vector<pbd_msgs::PbdAttributedPoint>();
            }
			for (std::size_t cnt = 0; cnt < sampleSize; cnt++) {
				SimpleVector3 randomVector;
                randomVector = MathHelper::getRandomVector(hp[idx], SimpleVector3(.05, .05, 0.01));

                pbd_msgs::PbdAttributedPoint element;

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
        bool setPointCloud = false;
		int x = 1;
		while(ros::ok()) {
            if(apc.request.point_cloud.elements.size() == 0)
            {
                ROS_ERROR("No elements were found");
                break;
            }
            else if(setPointCloud)
            {
                setPointCloud = false;
                if (!setPointCloudClient.call(apc.request, apc.response))
                {
                    ROS_ERROR("Could not set point cloud");
                    break;
                }
            }

            ROS_INFO_STREAM("Kalkuliere NBV "<< x);
			if (!getNextBestViewClient.call(nbv.request, nbv.response)) {
				ROS_ERROR("Something went wrong in next best view");
				break;
			}

            if (nbv.response.object_name_list.size() > 0)
            {
                getPointCloudClient.call(gpc);
                apc.request.point_cloud.elements.clear();
                apc.request.point_cloud.elements.insert(apc.request.point_cloud.elements.end(), gpc.response.point_cloud.elements.begin(), gpc.response.point_cloud.elements.end());

                for(int i=0;i<nbv.response.object_name_list.size();i++)
                {
                    std::vector<pbd_msgs::PbdAttributedPoint> temp;
                    for (std::vector<pbd_msgs::PbdAttributedPoint>::iterator it = apc.request.point_cloud.elements.begin(); it != apc.request.point_cloud.elements.end(); ++it)
                    {
                        if ((nbv.response.object_name_list[i].compare(it->object_type)) != 0)
                        {
                            temp.push_back(*it);
                        }
                    }
                    apc.request.point_cloud.elements.clear();
                    apc.request.point_cloud.elements.insert(apc.request.point_cloud.elements.end(), temp.begin(), temp.end());
                    setPointCloud = true;
                }
            }
            else
            {
                break;
            }

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

    boost::shared_ptr<SingleSceneTest> testPtr(new SingleSceneTest());

    evaluation->add(BOOST_CLASS_TEST_CASE(&SingleSceneTest::iterationTest, testPtr));

    framework::master_test_suite().add(evaluation);

    return 0;
}

