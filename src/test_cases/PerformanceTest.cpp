/*
 * TestRecognitionManager.h
 *
 *  Created on: Mar 15, 2014
 *      Author: ralfschleicher
 */
#define BOOST_TEST_STATIC_LINK

#include <boost/test/included/unit_test.hpp>

#include "next_best_view/test_cases/BaseTest.h"
#include <chrono>
#include <fstream>

using namespace next_best_view;
using namespace boost::unit_test;

class PerformanceTest : public BaseTest {

public:
    PerformanceTest() : BaseTest (){
    }

    virtual ~PerformanceTest() {}

    void iterationTest() {
        ros::NodeHandle nh;
        std::string testResultPath;
        nh.getParam("/test/testResultFilePath", testResultPath);

        std::ofstream file(testResultPath.c_str(), std::ios::out | std::ios::trunc);
        file << "HP,SampleSize,TotalNBV,AvgNBV,TotalUpd,AvgUpd" << std::endl;
        ROS_INFO_STREAM("Writing to " << testResultPath);

        ResetCalculator reca;
        std::chrono::time_point<std::chrono::system_clock> start, end;
        unsigned int numberOfTestRuns = 10;
	
        double totalTimeNBV, totalTimeUpdate;
        SimpleVector3* hp = new SimpleVector3[15];
        hp[0] = SimpleVector3(2.2320508103, 22.2942286316,0.8);
        hp[1] = SimpleVector3(3.09807621388, 21.7942286312,0.8);
        hp[2] = SimpleVector3(3.96410161747, 21.2942286309,0.8);
        hp[3] = SimpleVector3(4.83012702105, 20.7942286305,0.8);
        hp[4] = SimpleVector3(5.69615242463, 20.2942286302,0.8);
        hp[5] = SimpleVector3(6.56217782822, 19.7942286298,0.8);
        hp[6] = SimpleVector3(7.4282032318, 19.2942286295,0.8);
        hp[7] = SimpleVector3(8.29422863538, 18.7942286291,0.8);
        hp[8] = SimpleVector3(9.16025403897, 18.2942286288,0.8);
        hp[9] = SimpleVector3(10.0262794426, 17.7942286284,0.8);
        hp[10] = SimpleVector3(10.8923048461, 17.2942286281,0.8);
        hp[11] = SimpleVector3(11.7583302497, 16.7942286277,0.8);
        hp[12] = SimpleVector3(12.6243556533, 16.2942286274,0.8);
        hp[13] = SimpleVector3(13.4903810569, 15.794228627,0.8);
        hp[14] = SimpleVector3(14.3564064605, 15.2942286267,0.8);

        ROS_INFO("Generiere Häufungspunkte");
		// Häufungspunkte
        for(unsigned int sampleSize = 50; sampleSize<=400; sampleSize+=50)
        {

            for(unsigned int hpSize = 1; hpSize <= 15; hpSize++)
            {
                int countNBV = 0;
                int countUpdate = 0;
                totalTimeNBV = 0;
                totalTimeUpdate = 0;
                for (unsigned int testRun = 0; testRun < numberOfTestRuns; testRun++)
                {
                    std::cout << "Starting iteration " << testRun + 1 << std::endl;
                    SetAttributedPointCloud apc;

                    SimpleQuaternion* orientation = new SimpleQuaternion[hpSize];
                    for(unsigned int i=0; i<hpSize; i++){ orientation[i] = euler2Quaternion(-90, 100.0, 0.0);}

                    std::string* types = new std::string[hpSize];
                    for(unsigned int i=0; i<hpSize; i++){types[i] = "Smacks";}


                    std::map<std::string, std::vector<pbd_msgs::PbdAttributedPoint>* > objectPointCloudsMap;

                    for (std::size_t idx = 0; idx < hpSize; idx++) {

                        if(objectPointCloudsMap.find(types[idx]) == objectPointCloudsMap.end())
                        {
                            objectPointCloudsMap[types[idx]]= new std::vector<pbd_msgs::PbdAttributedPoint>();
                        }
                        for (std::size_t cnt = 0; cnt < sampleSize; cnt++)
                        {
                            SimpleVector3 randomVector;
                            randomVector = MathHelper::getRandomVector(hp[idx], SimpleVector3(.1, .1, 0.01));

                            pbd_msgs::PbdAttributedPoint element;

                            geometry_msgs::Pose pose;
                            pose.orientation.w = orientation[idx].w();
                            pose.orientation.x = orientation[idx].x();
                            pose.orientation.y = orientation[idx].y();
                            pose.orientation.z = orientation[idx].z();
                            pose.position.x = randomVector[0];
                            pose.position.y = randomVector[1];
                            pose.position.z = randomVector[2];

                            element.type = types[idx];
                            element.pose = pose;
                            objectPointCloudsMap[types[idx]]->push_back(element);
                            apc.request.point_cloud.elements.push_back(element);
                        }
                    }
                    std::cout << "point cloud size " << apc.request.point_cloud.elements.size() << std::endl;

                    ROS_INFO("Setze initiale Pose");
                    geometry_msgs::Pose initialPose;
                    initialPose.position.x = 1.64;
                    initialPose.position.y = 22.73;
                    initialPose.position.z = 1.32;

                    initialPose.orientation.w = 0.964072111801;
                    initialPose.orientation.x = 0.0;
                    initialPose.orientation.y = 0.0;
                    initialPose.orientation.z = -0.265640665651;
                    this->setInitialPose(initialPose);

                    //Resete den calculator vor jedem test
                    mResetCalculatorClient.call(reca.request, reca.response);
                    // Setze PointCloud
                    if (!mSetPointCloudClient.call(apc.request, apc.response))
                    {
                        ROS_ERROR("Could not set initial point cloud");
                    }
                    ros::Rate r(2);
                    GetNextBestView nbv;
                    nbv.request.current_pose = initialPose;
                    //ViewportPointCloudPtr viewportPointCloudPtr(new ViewportPointCloud());
                    bool setPointCloud = false;
                    int x = 1;
                    std::cout << "HaufungsPunkt : " << hpSize << ", SamplingSize " << sampleSize << std::endl;
                    while(ros::ok()) {
                        if(apc.request.point_cloud.elements.size() == 0)
                        {
                            ROS_ERROR("No elements were found");
                            break;
                        }
                        else if(setPointCloud)
                        {
                            setPointCloud = false;
                            if (!mSetPointCloudClient.call(apc.request, apc.response))
                            {
                                ROS_ERROR("Could not set point cloud");
                                break;
                            }
                        }

                        ROS_INFO_STREAM("Kalkuliere NBV "<< x);
                        start = std::chrono::system_clock::now();
                        if (!mGetNextBestViewClient.call(nbv.request, nbv.response)) {
                            ROS_ERROR("Something went wrong in next best view");
                            break;
                        }
                        end = std::chrono::system_clock::now();
                        countNBV ++;
                        std::chrono::duration<double> elapsed_seconds = end-start;
                        totalTimeNBV += elapsed_seconds.count();

                        if (!nbv.response.found) {
                            ROS_ERROR("No NBV found");
                            break;
                        }

                        UpdatePointCloud upc_req;
                        upc_req.request.object_type_name_list = nbv.response.object_type_name_list;
                        upc_req.request.pose_for_update = nbv.response.resulting_pose;

                        start = std::chrono::system_clock::now();
                        if(!mUpdatePointCloudClient.call(upc_req)) {
                            ROS_ERROR("Update Point Cloud failed!");
                            break;
                        }
                        end = std::chrono::system_clock::now();
                        countUpdate++;
                        elapsed_seconds = end-start;
                        totalTimeUpdate += elapsed_seconds.count();
                        x++;
                        if (x > 10)
                        {
                            break;
                        }

                        nbv.request.current_pose = nbv.response.resulting_pose;
                        ros::spinOnce();
                        ros::Duration(0.5).sleep();
                    }
                }
                file << hpSize << "," << sampleSize << ",";
                if (countNBV > 0)
                {
                    file << totalTimeNBV << "," << totalTimeNBV /(double)countNBV << ",";
                    std::cout << "Total time NBV : " << totalTimeNBV << std::endl;
                    std::cout << "Average time NBV : " << totalTimeNBV /(double)countNBV << std::endl;
                }
                else
                {
                    file << "0,0,";
                }
                if (countUpdate > 0)
                {
                    file << totalTimeUpdate << "," << totalTimeUpdate / (double)countUpdate << std::endl;
                    std::cout << "Total time update : " << totalTimeUpdate << std::endl;
                    std::cout << "Average time update : " << totalTimeUpdate / (double)countUpdate << std::endl;
                }
                else
                {
                    file << "0,0" << std::endl;
                }
                std::cout << "Iteration count : " << countNBV << " / " << countUpdate << std::endl;
            }
        }
      file.close();
    }
};

test_suite* init_unit_test_suite( int argc, char* argv[] ) {
	ros::init(argc, argv, "nbv_test");
	ros::start();

	ros::Duration(5).sleep();

	test_suite* evaluation = BOOST_TEST_SUITE("Evaluation NBV");

    boost::shared_ptr<PerformanceTest> testPtr(new PerformanceTest());

    evaluation->add(BOOST_CLASS_TEST_CASE(&PerformanceTest::iterationTest, testPtr));

    framework::master_test_suite().add(evaluation);

    return 0;
}

