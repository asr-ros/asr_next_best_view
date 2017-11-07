/**

Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Heller Florian, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Stroh Daniel, Trautmann Jeremias, Walter Milena, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

4. The use is explicitly not permitted to any application which deliberately try to kill or do harm to any living creature.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
        asr_next_best_view::GetAttributedPointCloud gpc;
		asr_next_best_view::SetAttributedPointCloud apc;

        ROS_INFO("Generating point cloud");

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

        std::string* types = new std::string[hpSize];
        types[0] = "Knaeckebrot";
        types[1] = "VitalisSchoko";
        types[2] = "Cup";
        types[3] = "Cup";

        std::string* ids = new std::string[hpSize];
        ids[0] = "1";
        ids[1] = "2";
        ids[2] = "000100000100";
        ids[3] = "100000000100";

       int sampleSize = 1;
       std::map<std::string, std::vector<asr_msgs::AsrAttributedPoint>* > objectPointCloudsMap;

        for (std::size_t idx = 0; idx < (std::size_t)hpSize; idx++) {

            if(objectPointCloudsMap.find(types[idx]) == objectPointCloudsMap.end())
            {
          objectPointCloudsMap[types[idx]]= new std::vector<asr_msgs::AsrAttributedPoint>();
            }
            for (std::size_t cnt = 0; cnt < (std::size_t)sampleSize; cnt++) {
				SimpleVector3 randomVector;
                randomVector = MathHelper::getRandomVector(hp[idx], SimpleVector3(.05, .05, 0.01));

                asr_msgs::AsrAttributedPoint element;

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
                element.identifier = ids[idx];
                objectPointCloudsMap[types[idx]]->push_back(element);
				apc.request.point_cloud.elements.push_back(element);
			}
		}

        ROS_INFO("Setting initial pose");
		geometry_msgs::Pose initialPose;
        initialPose.position.x = -0.974955737591;
        initialPose.position.y = -0.157173499465;
		initialPose.position.z = 1.32;

        initialPose.orientation.w = 0.718685498907;
		initialPose.orientation.x = 0.0;
        initialPose.orientation.y = 0.0;
        initialPose.orientation.z = 0.695335281472;
		this->setInitialPose(initialPose);

        // Set point cloud
        mSetPointCloudClient.call(apc.request, apc.response);
        ros::Rate r(2);
		asr_next_best_view::GetNextBestView nbv;
		nbv.request.current_pose = initialPose;
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
                if (!mSetPointCloudClient.call(apc.request, apc.response))
                {
                    ROS_ERROR("Could not set point cloud");
                    break;
                }
            }

            ROS_INFO_STREAM("Calculating NBV #"<< x);
            if (!mGetNextBestViewClient.call(nbv.request, nbv.response)) {
				ROS_ERROR("Something went wrong in next best view");
				break;
			}

            ROS_INFO_STREAM("Name list size " << nbv.response.viewport.object_type_name_list.size());

            if (nbv.response.viewport.object_type_name_list.size() > 0)
            {
                mGetPointCloudClient.call(gpc);
                apc.request.point_cloud.elements.clear();
                apc.request.point_cloud.elements.insert(apc.request.point_cloud.elements.end(), gpc.response.point_cloud.elements.begin(), gpc.response.point_cloud.elements.end());

                for(unsigned int i=0;i<nbv.response.viewport.object_type_name_list.size();i++)
                {
                    std::vector<asr_msgs::AsrAttributedPoint> temp;
                    ROS_INFO_STREAM("Type: " << nbv.response.viewport.object_type_name_list[i]);
                    for (std::vector<asr_msgs::AsrAttributedPoint>::iterator it = apc.request.point_cloud.elements.begin(); it != apc.request.point_cloud.elements.end(); ++it)
                    {

                        if ((nbv.response.viewport.object_type_name_list[i].compare(it->type)) != 0)
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

            asr_next_best_view::UpdatePointCloud upc_req;
            upc_req.request.object_type_name_list = nbv.response.viewport.object_type_name_list;
            upc_req.request.pose_for_update = nbv.response.viewport.pose;

            if(!mUpdatePointCloudClient.call(upc_req)) {
                ROS_ERROR("Update Point Cloud failed!");
                break;
            }

			x++;

                        nbv.request.current_pose = nbv.response.viewport.pose;

			ros::spinOnce();
			waitForEnter();
			ros::Duration(2).sleep();
		}
    }
};

test_suite* init_unit_test_suite( int argc, char* argv[] ) {
	ros::init(argc, argv, "nbv_test");
    ros::start();

	test_suite* evaluation = BOOST_TEST_SUITE("Evaluation NBV");

    boost::shared_ptr<SingleSceneTest> testPtr(new SingleSceneTest());

    evaluation->add(BOOST_CLASS_TEST_CASE(&SingleSceneTest::iterationTest, testPtr));

    framework::master_test_suite().add(evaluation);

    return 0;
}

