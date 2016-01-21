
/*
 * MultiIdSceneTest.cpp
 *
 *  Created on: Nov 23, 2015
 *      Author: Jeremias Trautmann
 */
#define BOOST_TEST_STATIC_LINK

#include <boost/test/included/unit_test.hpp>
#include "next_best_view/test_cases/BaseTest.h"
#include "next_best_view/GetAttributedPointCloud.h"

using namespace next_best_view;
using namespace boost::unit_test;

class MultiIdSceneTest : public BaseTest{
public:
    MultiIdSceneTest() : BaseTest (){
    }

    virtual ~MultiIdSceneTest() {}

    void iterationTest() {
        ros::ServiceClient setPointCloudClient = mNodeHandle.serviceClient<SetAttributedPointCloud>("/nbv/set_point_cloud");
        ros::ServiceClient getPointCloudClient = mNodeHandle.serviceClient<GetAttributedPointCloud>("/nbv/get_point_cloud");
        ros::ServiceClient getNextBestViewClient = mNodeHandle.serviceClient<GetNextBestView>("/nbv/next_best_view");
        ros::ServiceClient updatePointCloudClient = mNodeHandle.serviceClient<UpdatePointCloud>("/nbv/update_point_cloud");

        GetAttributedPointCloud gpc;
        SetAttributedPointCloud apc;

        ROS_INFO("Generiere Häufungspunkte");
        // Häufungspunkte
        int hpSize_scene1 = 6;
        int hpSize_scene2 = 3;
        SimpleVector3* hp_scene1 = new SimpleVector3[hpSize_scene1];
        SimpleVector3* hp_scene2 = new SimpleVector3[hpSize_scene2];
        hp_scene1[0] = SimpleVector3(0.725892364979, 1.57344818115, 0.8);
        hp_scene1[1] = SimpleVector3(1.0500099659, 1.63358879089,  0.8);
        hp_scene1[2] = SimpleVector3(1.66446590424, -0.549417376518,  0.8);
        hp_scene1[3] = SimpleVector3(1.92009782791,-0.298665702343,  0.8);
        hp_scene1[4] = SimpleVector3(-0.978491704464, 0.583356167078,  0.8);
        hp_scene1[5] = SimpleVector3(-0.972244024277,  0.759688913822,  0.8);
        hp_scene2[0] = SimpleVector3(-0.404876768589,-0.50940537452, 1.3);
        hp_scene2[1] = SimpleVector3(-0.404876768589,-0.50940537452, 1.3);
        hp_scene2[2] = SimpleVector3(-0.404876768589,-0.50940537452, 1.7);

        SimpleQuaternion* orientation_scene1 = new SimpleQuaternion[hpSize_scene1];
        SimpleQuaternion* orientation_scene2 = new SimpleQuaternion[hpSize_scene2];
        orientation_scene1[0] = euler2Quaternion(-90, 0.0, 0.0);
        orientation_scene1[1] = euler2Quaternion(-90, 0.0, 0.0);
        orientation_scene1[2] = euler2Quaternion(-90, 100.0, 0.0);
        orientation_scene1[3] = euler2Quaternion(-90, 170.0, 0.0);
        orientation_scene1[4] = euler2Quaternion(-90, -90.0, 0.0);
        orientation_scene1[5] = euler2Quaternion(-90, -90.0, 0.0);
        orientation_scene2[0] = euler2Quaternion(-90, 180.0, 0.0);
        orientation_scene2[1] = euler2Quaternion(-90, -90.0, 0.0);
        orientation_scene2[2] = euler2Quaternion(-90, 180, 0.0);

        std::string* types_scene1 = new std::string[hpSize_scene1];
        std::string* types_scene2 = new std::string[hpSize_scene2];
        types_scene1[0] = "Cup";
        types_scene1[1] = "PlateDeep";
        types_scene1[2] = "CoffeeBox";
        types_scene1[3] = "CoffeeFilters2";
        types_scene1[4] = "Smacks";
        types_scene1[5] = "VitalisSchoko";
        types_scene2[0] = "CeylonTea";
        types_scene2[1] = "CeylonTea";
        types_scene2[2] = "CeylonTea";


       int sampleSize = 10;
       std::map<std::string, std::vector<pbd_msgs::PbdAttributedPoint>* > objectPointCloudsMap;

       for (std::size_t idx = 0; idx < (std::size_t)hpSize_scene1; idx++) {

            if(objectPointCloudsMap.find(types_scene1[idx]) == objectPointCloudsMap.end())
            {
                objectPointCloudsMap[types_scene1[idx]]= new std::vector<pbd_msgs::PbdAttributedPoint>();
            }
            for (std::size_t cnt = 0; cnt < (std::size_t)sampleSize; cnt++) {
                SimpleVector3 randomVector;
                randomVector = MathHelper::getRandomVector(hp_scene1[idx], SimpleVector3(.05, .05, 0.01));

                pbd_msgs::PbdAttributedPoint element;

                geometry_msgs::Pose pose;
                pose.orientation.w = orientation_scene1[idx].w();
                pose.orientation.x = orientation_scene1[idx].x();
                pose.orientation.y = orientation_scene1[idx].y();
                pose.orientation.z = orientation_scene1[idx].z();
                pose.position.x = randomVector[0];
                pose.position.y = randomVector[1];
                pose.position.z = randomVector[2];

                element.type = types_scene1[idx];
                element.identifier = std::to_string(idx);
                element.pose = pose;
                objectPointCloudsMap[types_scene1[idx]]->push_back(element);
                apc.request.point_cloud.elements.push_back(element);
            }
        }

        ROS_INFO("Setze initiale Pose");
        geometry_msgs::Pose initialPose;
        initialPose.position.x = 1.04865884781;
        initialPose.position.y = 0.846048951149;
        initialPose.position.z = 1.32;

        initialPose.orientation.w = 0.668036938496;
        initialPose.orientation.x = 0.0;
        initialPose.orientation.y = 0.0;
        initialPose.orientation.z = 0.744128113166;
        this->setInitialPose(initialPose);

        apc.request.pose = initialPose;
        // Setze PointCloud
        if (!setPointCloudClient.call(apc.request, apc.response)) {
            ROS_ERROR("Could not set initial point cloud.");
        }

        if (!getPointCloudClient.call(gpc)) {
            ROS_ERROR("Could not get initial point cloud.");
        }

        ros::Rate r(2);
        GetNextBestView nbv;
        nbv.request.current_pose = initialPose;

        int x = 1;
        bool scene2_isInitialized = false;
        bool setPointCloud = false;
        while(ros::ok()) {

            if(apc.request.point_cloud.elements.size() == 0)
            {
                ROS_INFO("No elements were found");
                break;
            }
            else if(setPointCloud)
            {
                setPointCloud = false;
                if (!setPointCloudClient.call(apc.request, apc.response)) {
                    ROS_ERROR("Could not set point cloud.");
                    break;
                }
            }

            ROS_INFO_STREAM("Kalkuliere NBV " << x);
            if (!getNextBestViewClient.call(nbv.request, nbv.response)) {
                ROS_ERROR("Something went wrong in next best view");
                break;
            }
            if (nbv.response.object_name_list.size() > 0)
            {
                if (!getPointCloudClient.call(gpc)) {
                    ROS_ERROR("Could not get point cloud.");
                    break;
                }
                apc.request.point_cloud.elements.clear();
                apc.request.point_cloud.elements.insert(apc.request.point_cloud.elements.end(), gpc.response.point_cloud.elements.begin(), gpc.response.point_cloud.elements.end());

                for(unsigned int i=0;i<nbv.response.object_name_list.size();i++)
                {
                    if (nbv.response.object_name_list[i] != "CeylonTea")
                    {
                        std::vector<pbd_msgs::PbdAttributedPoint> temp;
                        for (std::vector<pbd_msgs::PbdAttributedPoint>::iterator it = apc.request.point_cloud.elements.begin(); it != apc.request.point_cloud.elements.end(); ++it)
                        {
                            if ((nbv.response.object_name_list[i].compare(it->type)) != 0)
                            {
                                temp.push_back(*it);
                            }
                        }
                        apc.request.point_cloud.elements.clear();
                        apc.request.point_cloud.elements.insert(apc.request.point_cloud.elements.end(), temp.begin(), temp.end());
                        setPointCloud = true;
                    }
                    if (nbv.response.object_name_list[i] == "Smacks" && !scene2_isInitialized)
                    {
                        for (std::size_t idx = 0; idx < (std::size_t)hpSize_scene2; idx++) {

                             if(objectPointCloudsMap.find(types_scene2[idx]) == objectPointCloudsMap.end())
                             {
                                 objectPointCloudsMap[types_scene2[idx]]= new std::vector<pbd_msgs::PbdAttributedPoint>();
                             }
                             for (std::size_t cnt = 0; cnt < (std::size_t)sampleSize; cnt++) {
                                 SimpleVector3 randomVector;
                                 randomVector = MathHelper::getRandomVector(hp_scene2[idx], SimpleVector3(.05, .05, 0.01));
                                 pbd_msgs::PbdAttributedPoint element;
                                 geometry_msgs::Pose pose;
                                 pose.orientation.w = orientation_scene2[idx].w();
                                 pose.orientation.x = orientation_scene2[idx].x();
                                 pose.orientation.y = orientation_scene2[idx].y();
                                 pose.orientation.z = orientation_scene2[idx].z();
                                 pose.position.x = randomVector[0];
                                 pose.position.y = randomVector[1];
                                 pose.position.z = randomVector[2];
                                 element.type = types_scene2[idx];
                                 element.identifier = std::to_string(idx + hpSize_scene1);
                                 element.pose = pose;
                                 apc.request.point_cloud.elements.push_back(element);
                             }
                         }
                        scene2_isInitialized = true;
                        setPointCloud = true;
                    }
                }
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

    boost::shared_ptr<MultiIdSceneTest> testPtr(new MultiIdSceneTest());

    //evaluation->add(BOOST_CLASS_TEST_CASE(&MultiSceneTest::visualizeSingleObjectWithNormals, testPtr));
    evaluation->add(BOOST_CLASS_TEST_CASE(&MultiIdSceneTest::iterationTest, testPtr));

    framework::master_test_suite().add(evaluation);

    return 0;
}


