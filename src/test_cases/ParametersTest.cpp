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
#include "world_model/EmptyViewportList.h"
#include "next_best_view/NextBestView.hpp"
#include "next_best_view/test_cases/BaseTest.h"
#include "next_best_view/robot_model/impl/MILDRobotModel.hpp"
#include "next_best_view/robot_model/impl/MILDRobotModelWithExactIK.hpp"
#include "next_best_view/robot_model/impl/MILDRobotState.hpp"
#include "next_best_view/helper/MathHelper.hpp"
#include "next_best_view/rating/impl/DefaultRatingModule.hpp"
#include "typedef.hpp"

using namespace next_best_view;
using namespace boost::unit_test;

/*!
 * \brief The ParametersTest class is a test class to check the NBV parameters.
 * It works with the map "map_flur".
 */
class ParametersTest : public BaseTest
{
private:
    ros::NodeHandle mNodeHandle;
    ros::NodeHandle mGlobalNodeHandle;
    boost::shared_ptr<NextBestView> NBV;
    MapHelper mMapHelper;

    std::string mOutputPath;

public:
    ParametersTest() : BaseTest(false, false), mNodeHandle("~") {
        NBV = boost::shared_ptr<NextBestView>(new NextBestView());
        mNodeHandle.param("output_path", mOutputPath, std::string(""));
        ROS_INFO_STREAM("output_path: " << mOutputPath);
    }

    virtual ~ParametersTest() {}

    void configurablePointCloudsTest() {
        bool doTest = true;
        mNodeHandle.param("configurable_point_clouds/test", doTest, true);

        if (!doTest)
            return;

        ROS_INFO("Running configurable point cloud test");

        // get parameters
        std::vector<std::string> objectTypeNames1, objectTypeNames2;
        if (!this->getParameter("configurable_point_clouds/object_types_1", objectTypeNames1))
            return;
        if (!this->getParameter("configurable_point_clouds/object_types_2", objectTypeNames2))
            return;

        int elements1, elements2;
        if (!this->getParameter("configurable_point_clouds/elements_1", elements1))
            return;
        if (!this->getParameter("configurable_point_clouds/elements_2", elements2))
            return;

        // between 0.0 (at first point cloud) and 1.0 (at second point cloud)
        double distance;
        if (!this->getParameter("configurable_point_clouds/distance", distance))
            return;

        bool continueTest = true;
        while (continueTest)
        {
            ROS_INFO_STREAM("Running test with distance at " << distance);
            std::map<std::string, std::vector<double>> typeToOrientation;
            for (unsigned int i = 0; i < objectTypeNames1.size(); i++)
            {
                std::string objectType = objectTypeNames1[i];
                std::string parameter = "configurable_point_clouds/" + objectType + "_orientation";
                typeToOrientation[objectType] = std::vector<double>();
                if (!this->getParameter(parameter, typeToOrientation[objectType]))
                    return;
            }
            for (unsigned int i = 0; i < objectTypeNames2.size(); i++)
            {
                std::string objectType = objectTypeNames2[i];
                if (std::find(objectTypeNames1.begin(), objectTypeNames1.end(), objectType) == objectTypeNames1.end()) {
                    std::string parameter = "configurable_point_clouds/" + objectType + "_orientation";
                    typeToOrientation[objectType] = std::vector<double>();
                    if (!this->getParameter(parameter, typeToOrientation[objectType]))
                        return;
                }
            }

            std::vector<double> posDeviation1List, posDeviation2List;
            if (!this->getParameter("configurable_point_clouds/position_deviation_1", posDeviation1List))
                return;
            if (!this->getParameter("configurable_point_clouds/position_deviation_2", posDeviation2List))
                return;
            SimpleVector3 posDeviation1(posDeviation1List[0], posDeviation1List[1], posDeviation1List[2]);
            SimpleVector3 posDeviation2(posDeviation2List[0], posDeviation2List[1], posDeviation2List[2]);

            float orientationDeviation1, orientationDeviation2;
            if (!this->getParameter("configurable_point_clouds/orientation_deviation_1", orientationDeviation1))
                return;
            if (!this->getParameter("configurable_point_clouds/orientation_deviation_2", orientationDeviation2))
                return;

            world_model::EmptyViewportList empty;
            ros::service::waitForService("/env/world_model/empty_viewport_list", -1);
            ros::ServiceClient emptyViewportsClient = mGlobalNodeHandle.serviceClient<world_model::EmptyViewportList>("/env/world_model/empty_viewport_list");

            ROS_INFO("Generiere Häufungspunkte");
            // Häufungspunkte
            int hpSize = 2;
            SimpleVector3* hp = new SimpleVector3[hpSize];
            hp[0] = SimpleVector3(30.5, 5.8, 1.32);
            hp[1] = SimpleVector3(32.6, 4.5, 1.32);

            SimpleQuaternion* orientations = new SimpleQuaternion[hpSize];
            orientations[1] = SimpleQuaternion(0.967, 0.000, 0.000, -0.256);
            orientations[0] = this->getOrientation(orientations[1], 0, 0, 180);

            SetAttributedPointCloud apc;

            //create first point cloud
            for (int i = 0; i < elements1; i++) {
                asr_msgs::AsrAttributedPoint element = this->getAttributedPoint(hp[0], posDeviation1, orientations[0], orientationDeviation1,
                                                                                    objectTypeNames1, typeToOrientation, i);

                apc.request.point_cloud.elements.push_back(element);
            }

            //create second point cloud
            for (int i = 0; i < elements2; i++) {
                asr_msgs::AsrAttributedPoint element = this->getAttributedPoint(hp[1], posDeviation2, orientations[1], orientationDeviation2,
                                                                                    objectTypeNames2, typeToOrientation, i);

                apc.request.point_cloud.elements.push_back(element);
            }

            ROS_INFO("Setze initiale Pose");
            // interpoliere zwischen Roboter Position und Punktwolke
            SimpleVector3 position;
            position[0] = hp[0][0] + distance * (hp[1][0] - hp[0][0]) + 0.2;
            position[1] = hp[0][1] + distance * (hp[1][1] - hp[0][1]) + 0.3;
            position[2] = 1.32;

            SimpleQuaternion orientation = this->getOrientation(orientations[1], 0, 0, -90);

            geometry_msgs::Pose initialPose;

            //Pose in der Mitte von CeylonTea und CoffeeFilters
            initialPose.position.x = position[0];
            initialPose.position.y = position[1];
            initialPose.position.z = position[2];

            initialPose.orientation.w = orientation.w();
            initialPose.orientation.x = orientation.x();
            initialPose.orientation.y = orientation.y();
            initialPose.orientation.z = orientation.z();

            GetNextBestView getNBV;

            this->setInitialPose(initialPose, NBV);

            emptyViewportsClient.call(empty);

            NBV->processSetPointCloudServiceCall(apc.request, apc.response);

            getNBV.request.current_pose = initialPose;
            NBV->processGetNextBestViewServiceCall(getNBV.request, getNBV.response);

            TriggerFrustumVisualization tfv;
            tfv.request.current_pose = getNBV.response.resulting_pose;
            NBV->processTriggerFrustumVisualization(tfv.request, tfv.response);

            float robotX = getNBV.response.resulting_pose.position.x;
            float robotY = getNBV.response.resulting_pose.position.y;

            float distanceToPC = sqrt(pow(robotX - hp[1](0), 2) + pow(robotY - hp[1](1), 2));
            if (sqrt(pow(robotX - hp[0](0), 2) + pow(robotY - hp[0](1), 2)) < sqrt(pow(robotX - hp[1](0), 2) + pow(robotY - hp[1](1), 2))) {
                ROS_INFO("Roboter bewegt sich zur 1. Objekt-Punktwolke.");
            }
            else {
                ROS_INFO("Roboter bewegt sich zur 2. Objekt-Punktwolke.");
            }
            ROS_INFO("Abstand zur Punktwolke: %f", distanceToPC);

            ROS_INFO("To run another test, enter a distance factor between 0 an 1 or press ENTER to run the test again with the same distance factor.");
            ROS_INFO("Type <quit> or whatever to exit.");
            std::string input;
            std::getline (std::cin,input);
            if( input.compare("\n") != 0 ){
                try {
                  double tempDistance;
                  tempDistance = boost::lexical_cast<double>(input);
                  if (tempDistance >= 0.0 && tempDistance <= 1.0)
                  {
                      distance = tempDistance;
                  }
                  else
                  {
                      ROS_ERROR("Distance must be between 0.0 and 1.0!");
                      continueTest = false;
                  }
                } catch(boost::bad_lexical_cast &) {
                  ROS_ERROR("Invalid number!");
                  continueTest = false;
                }
            }
        }
    }

    void positionsTest() {
        bool doTest = true;
        mNodeHandle.param("positions/test", doTest, true);

        if (!doTest)
            return;

        ROS_INFO("Running positions test");

        ROS_INFO("Getting parameters");
        std::vector<std::string> elementTypes;
        if (!this->getParameter("positions/element_types", elementTypes))
            return;

        std::vector<double> elementOrientationList;
        if (!this->getParameter("positions/element_orientation", elementOrientationList))
            return;

        world_model::EmptyViewportList empty;
        ros::service::waitForService("/env/world_model/empty_viewport_list", -1);
        ros::ServiceClient emptyViewportsClient = mGlobalNodeHandle.serviceClient<world_model::EmptyViewportList>("/env/world_model/empty_viewport_list");

        // object poses
        unsigned int pcSize = 3;

        if (pcSize != elementTypes.size()) {
            ROS_ERROR_STREAM(pcSize << "element types needed");
            return;
        }

        SimpleVector3* hp = new SimpleVector3[pcSize];
        hp[0] = SimpleVector3(25.742, 8.607, 1.32);
        hp[1] = SimpleVector3(24.041, 10.947, 1.32);
        hp[2] = SimpleVector3(25.902, 11.025, 1.32);

        SimpleQuaternion* pcOrientations = new SimpleQuaternion[pcSize];
        pcOrientations[0] = SimpleQuaternion(0.256, 0.000, 0.000, 0.967);
        pcOrientations[1] = SimpleQuaternion(0.960, 0.000, 0.000, -0.281);
        pcOrientations[2] = SimpleQuaternion(-0.483, 0.000, 0.000, 0.875);

        // robot poses
        unsigned int robotPoses = 3;

        SimpleVector3* rp = new SimpleVector3[robotPoses];
        rp[2] = SimpleVector3(24.288, 9.430, 1.32);
        rp[1] = SimpleVector3(24.991, 10.441, 1.32);
        rp[0] = SimpleVector3(25.294, 9.906, 1.32);

        SimpleQuaternion* robotOrientations = new SimpleQuaternion[robotPoses];
        robotOrientations[2] = this->getOrientation(pcOrientations[0], 0, 0, 180);
        robotOrientations[1] = this->getOrientation(pcOrientations[1], 0, 0, 180);
        robotOrientations[0] = this->getOrientation(pcOrientations[2], 0, 0, 180);

        for (unsigned int i = 0; i < robotPoses; i++) {
            SetAttributedPointCloud apc;

            // create point cloud
            ROS_INFO("Generate point cloud");
            for (unsigned int j = 0; j < pcSize; j++) {
                SimpleQuaternion q = this->getOrientation(pcOrientations[j], elementOrientationList[0],
                                                            elementOrientationList[1], elementOrientationList[2]);

                asr_msgs::AsrAttributedPoint element;

                geometry_msgs::Pose pose;
                pose.position.x = hp[j][0];
                pose.position.y = hp[j][1];
                pose.position.z = hp[j][2];
                pose.orientation.w = q.w();
                pose.orientation.x = q.x();
                pose.orientation.y = q.y();
                pose.orientation.z = q.z();

                element.type = elementTypes[j];
                element.pose = pose;

                apc.request.point_cloud.elements.push_back(element);
            }

            // set robot pose
            ROS_INFO("Set robot pose");
            geometry_msgs::Pose initialPose;

            //Pose in der Mitte von CeylonTea und CoffeeFilters
            initialPose.position.x = rp[i][0];
            initialPose.position.y = rp[i][1];
            initialPose.position.z = rp[i][2];

            initialPose.orientation.w = robotOrientations[i].w();
            initialPose.orientation.x = robotOrientations[i].x();
            initialPose.orientation.y = robotOrientations[i].y();
            initialPose.orientation.z = robotOrientations[i].z();

            this->setInitialPose(initialPose, NBV);

            // calculate NBV
            GetNextBestView getNBV;

            emptyViewportsClient.call(empty);

            NBV->processSetPointCloudServiceCall(apc.request, apc.response);

            getNBV.request.current_pose = initialPose;
            NBV->processGetNextBestViewServiceCall(getNBV.request, getNBV.response);

            if (!getNBV.response.found) {
                ROS_ERROR("No NBV found!");
            } else {
                TriggerFrustumVisualization tfv;
                tfv.request.current_pose = getNBV.response.resulting_pose;
                NBV->processTriggerFrustumVisualization(tfv.request, tfv.response);
            }

            ros::spinOnce();
            waitForEnter();
            ros::Duration(2).sleep();
        }
    }

    void sideObjectTest()
    {
        bool doTest = true;
        mNodeHandle.param("side_objects/test", doTest, true);

        if (!doTest)
            return;

        ROS_INFO("Running side object test");

        GetNextBestView getNBV;
        world_model::EmptyViewportList empty;
        ros::ServiceClient emptyViewportsClient = mGlobalNodeHandle.serviceClient<world_model::EmptyViewportList>("/env/world_model/empty_viewport_list");

        ofstream outputFile(mOutputPath + "testSideObject.dat");
        if(outputFile.is_open()) {
            outputFile << "#Objekt steht zuerst gegenüber vom Roboter und wird dann seitlich verschoben \n",
            outputFile << "#Verschiebung des Objekts\tVeränderung Pan\t\tVeränderung Rotation\tVeränderung x\tVeränderung y \n";
        }

        ROS_INFO("Getting parameters");
        std::string elementType;
        if (!this->getParameter("side_objects/element_type", elementType))
            return;
        ObjectTypeSetPtr typeSet = ObjectTypeSetPtr(new ObjectTypeSet);
        typeSet->insert(elementType);

        int cloudSize;
        if (!this->getParameter("side_objects/cloud_size", cloudSize))
            return;

        std::vector<double> initialPoseList;
        if (!this->getParameter("side_objects/initial_pose", initialPoseList))
            return;

        std::vector<double> elementOrientationList;
        if (!this->getParameter("side_objects/element_orientation", elementOrientationList))
            return;

        std::vector<double> elementDeviationList;
        if (!this->getParameter("side_objects/element_deviation", elementDeviationList))
            return;
        SimpleVector3 elementDeviation(elementDeviationList[0], elementDeviationList[1], elementDeviationList[2]);

        int stepSize;
        if (!this->getParameter("side_objects/step_size", stepSize))
            return;

        int stepNumber;
        if (!this->getParameter("side_objects/step_number", stepNumber))
            return;

        double ncp;
        if (!this->getParameter("ncp", ncp))
            return;

        double fcp;
        if (!this->getParameter("fcp", fcp))
            return;

        ROS_INFO("Set initial pose");


        geometry_msgs::Pose initialPose;

        initialPose.position.x = initialPoseList[0];
        initialPose.position.y = initialPoseList[1];
        initialPose.position.z = initialPoseList[2];
        initialPose.orientation.x = initialPoseList[3];
        initialPose.orientation.y = initialPoseList[4];
        initialPose.orientation.z = initialPoseList[5];
        initialPose.orientation.w = initialPoseList[6];

        this->setInitialPose(initialPose, NBV);

        for (int i = 0; i < stepNumber; i++)
        {
            ROS_INFO_STREAM("Step " << i << " of side object test");

            int stepOffset = i * stepSize;

            SetAttributedPointCloud apc;

            MILDRobotModelWithExactIK robot;
            SimpleVector3 position(initialPose.position.x, initialPose.position.y, initialPose.position.z);
            SimpleQuaternion orientation(initialPose.orientation.w, initialPose.orientation.x, initialPose.orientation.y, initialPose.orientation.z);
            MILDRobotStatePtr robotStatePtr(new MILDRobotState());
            robotStatePtr = boost::static_pointer_cast<MILDRobotState>(robot.calculateRobotState(robotStatePtr, position, orientation));

            ROS_INFO_STREAM("Set point cloud consisting of one object type with " << cloudSize << " occurances.");

            ObjectPointCloudPtr objectPointCloudPtr = ObjectPointCloudPtr(new ObjectPointCloud());

            for (int j = 0; j < cloudSize; j++)
            {
                // set element position
                Eigen::Matrix<Precision, 3, 3> stepRotation;
                stepRotation = Eigen::AngleAxis<Precision>(0.0, SimpleVector3::UnitX())
                            * Eigen::AngleAxis<Precision>(0.0, SimpleVector3::UnitY())
                            * Eigen::AngleAxis<Precision>(stepOffset * M_PI / 180, SimpleVector3::UnitZ());
                SimpleVector3 elementPosition = position + (ncp + fcp) / 2 * stepRotation * MathHelper::getVisualAxis(orientation);

                SimpleVector3 randomVector;
                if (elementDeviation[0] == 0 && elementDeviation[1] == 0 && elementDeviation[2] == 0)
                {
                    randomVector = elementPosition;
                }
                else
                {
                    int8_t occupancyValue;
                    do {
                        randomVector = MathHelper::getRandomVector(elementPosition, elementDeviation);
                        occupancyValue = mMapHelper.getRaytracingMapOccupancyValue(randomVector);
                    } while(!mMapHelper.isOccupancyValueAcceptable(occupancyValue));
                }

                // set element orientation
                Eigen::Matrix<Precision, 3, 3> elementRotation;
                elementRotation = Eigen::AngleAxis<Precision>(elementOrientationList[0] * M_PI / 180, SimpleVector3::UnitX())
                            * Eigen::AngleAxis<Precision>(elementOrientationList[1] * M_PI / 180, SimpleVector3::UnitY())
                            * Eigen::AngleAxis<Precision>(elementOrientationList[2] * M_PI / 180, SimpleVector3::UnitZ());
                SimpleQuaternion elementOrientation(orientation.toRotationMatrix() * stepRotation * elementRotation);

                geometry_msgs::Pose pose;
                pose.position.x = randomVector[0];
                pose.position.y = randomVector[1];
                pose.position.z = randomVector[2];
                pose.orientation.x = elementOrientation.x();
                pose.orientation.y = elementOrientation.y();
                pose.orientation.z = elementOrientation.z();
                pose.orientation.w = elementOrientation.w();

                asr_msgs::AsrAttributedPoint element;
                element.type = elementType;
                element.pose = pose;

                apc.request.point_cloud.elements.push_back(element);


                ObjectPoint objectPoint(pose);
                objectPoint.r = 0;
                objectPoint.g = 255;
                objectPoint.b = 0;
                objectPoint.type = elementType;

                objectPointCloudPtr->push_back(objectPoint);
            }

            emptyViewportsClient.call(empty);

            NBV->processSetPointCloudServiceCall(apc.request, apc.response);

            getNBV.request.current_pose = initialPose;

            ROS_INFO("Asking for next best view");
            NBV->processGetNextBestViewServiceCall(getNBV.request, getNBV.response);

            if (!getNBV.response.found)
            {
                ROS_ERROR("No next best view found!");
                continue;
            }

            ROS_INFO("Got next best view");

            TriggerFrustumVisualization tfv;
            tfv.request.current_pose = getNBV.response.resulting_pose;
            NBV->processTriggerFrustumVisualization(tfv.request, tfv.response);

            SimpleVector3 pos(initialPose.position.x, initialPose.position.y, initialPose.position.z);
            SimpleQuaternion q(initialPose.orientation.w, initialPose.orientation.x, initialPose.orientation.y, initialPose.orientation.z);

            NextBestViewCalculator nbvCalc = NBV->getCalculator();
            ViewportPoint viewport;
            nbvCalc.doFrustumCulling(pos, q, nbvCalc.getActiveIndices(), viewport);

            viewport.object_type_set = typeSet;

            ROS_INFO_STREAM("Original state: pan " << robotStatePtr->pan << " tilt " << robotStatePtr->tilt
                                        << " rotation " << robotStatePtr->rotation
                                        << " x " << robotStatePtr->x << " y " << robotStatePtr->y);
            if (nbvCalc.getRatingModule()->setSingleScoreContainer(viewport, viewport))
                ROS_INFO_STREAM("Original viewport: utility " << viewport.score->getWeightedNormalizedUtility() << " inverseCosts " << viewport.score->getWeightedInverseCosts()
                                    << " rating " << viewport.score->getWeightedNormalizedUtility() * viewport.score->getWeightedInverseCosts());
            else
                ROS_INFO("Original viewport utility <= 0");

            TriggerFrustumVisualization triggerFrustumVis;
            triggerFrustumVis.request.current_pose = viewport.getPose();
            NBV->processTriggerOldFrustumVisualization(triggerFrustumVis.request, triggerFrustumVis.response);

            ROS_INFO_STREAM("Changes: pan " << getNBV.response.robot_state.pan << " tilt " << getNBV.response.robot_state.tilt
                                       << " rotation " << getNBV.response.robot_state.rotation
                                       << " x " << getNBV.response.robot_state.x << " y " << getNBV.response.robot_state.y);
            ROS_INFO_STREAM("Resulting viewport: utility " << getNBV.response.utility << " inverseCosts " << getNBV.response.inverse_costs
                                << " rating " << getNBV.response.utility * getNBV.response.inverse_costs);

            double panDiff = std::abs(robotStatePtr->pan - getNBV.response.robot_state.pan);
            double rotDiff = std::abs(robotStatePtr->rotation - getNBV.response.robot_state.rotation);
            double xDiff = std::abs(robotStatePtr->x - getNBV.response.robot_state.x);
            double yDiff = std::abs(robotStatePtr->y - getNBV.response.robot_state.y);

            outputFile << stepOffset << "\t\t\t\t" << panDiff << "\t\t" << rotDiff <<  "\t\t" << xDiff << "\t\t" << yDiff << " \n";

        }


        outputFile.close();

    }

    template<typename T> bool getParameter(const std::string &key, T &parameter)
    {
        if (!mNodeHandle.getParam(key, parameter))
        {
            ROS_ERROR_STREAM(key << " parameter not set!");
            return false;
        }
        else
        {
            return true;
        }
    }

    SimpleQuaternion getOrientation(SimpleQuaternion originalOrientation, double alpha, double beta, double gamma)
    {
        Eigen::Matrix<Precision, 3, 3> rotation;
        rotation = Eigen::AngleAxis<Precision>(alpha* M_PI / 180, SimpleVector3::UnitX())
                    * Eigen::AngleAxis<Precision>(beta * M_PI / 180, SimpleVector3::UnitY())
                    * Eigen::AngleAxis<Precision>(gamma * M_PI / 180, SimpleVector3::UnitZ());
        SimpleQuaternion result;
        result = originalOrientation.toRotationMatrix() * rotation;
        return result;
    }

    asr_msgs::AsrAttributedPoint getAttributedPoint(SimpleVector3 position, SimpleVector3 positionDeviation,
                                                        SimpleQuaternion orientation, float orientationDeviation,
                                                        std::vector<std::string> objectTypeNames,
                                                        std::map<std::string, std::vector<double>> typeToOrientation,
                                                        int iteration) {
        SimpleVector3 randomVector;
        int8_t occupancyValue;
        do {
            randomVector = MathHelper::getRandomVector(position, positionDeviation);
            occupancyValue = mMapHelper.getRaytracingMapOccupancyValue(randomVector);
        } while(!mMapHelper.isOccupancyValueAcceptable(occupancyValue));

        vector<double> typeOrientation = typeToOrientation[objectTypeNames[iteration % objectTypeNames.size()]];
        float randomDeviation = MathHelper::getRandomNumber(0, orientationDeviation);
        SimpleQuaternion randomOrientation = this->getOrientation(orientation, 0.0, 0.0, randomDeviation);
        SimpleQuaternion quaternion = this->getOrientation(randomOrientation, typeOrientation[0], typeOrientation[1], typeOrientation[2]);

        asr_msgs::AsrAttributedPoint element;

        geometry_msgs::Pose pose;
        pose.orientation.w = quaternion.w();
        pose.orientation.x = quaternion.x();
        pose.orientation.y = quaternion.y();
        pose.orientation.z = quaternion.z();
        pose.position.x = randomVector[0];
        pose.position.y = randomVector[1];
        pose.position.z = randomVector[2];

        element.type = objectTypeNames[iteration % objectTypeNames.size()];
        element.pose = pose;

        return element;
    }

};

test_suite* init_unit_test_suite( int argc, char* argv[] )
{
    ros::init(argc, argv, "nbv");
    ros::start();

    test_suite* evaluation = BOOST_TEST_SUITE("Evaluation NBV with different scenes");

    boost::shared_ptr<ParametersTest> testPtr(new ParametersTest());

    evaluation->add(BOOST_CLASS_TEST_CASE(&ParametersTest::configurablePointCloudsTest, testPtr));
    evaluation->add(BOOST_CLASS_TEST_CASE(&ParametersTest::positionsTest, testPtr));
    evaluation->add(BOOST_CLASS_TEST_CASE(&ParametersTest::sideObjectTest, testPtr));

    framework::master_test_suite().add(evaluation);

    return 0;
}
