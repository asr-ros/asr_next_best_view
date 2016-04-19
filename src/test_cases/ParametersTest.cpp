#define BOOST_TEST_STATIC_LINK

#include <boost/test/included/unit_test.hpp>
#include "world_model/EmptyViewportList.h"
#include "next_best_view/NextBestView.hpp"
#include "next_best_view/test_cases/BaseTest.h"
#include "next_best_view/robot_model/impl/MILDRobotModel.hpp"
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
    NextBestView NBV;

    std::string mOutputPath;

public:
    ParametersTest() : BaseTest(), mNodeHandle("~") {
        mNodeHandle.param("output_path", mOutputPath, std::string(""));
        ROS_INFO_STREAM("output_path: " << mOutputPath);
    }

    virtual ~ParametersTest() {}

    /*
     * this test case uses a scene with 2 far apart point clouds
     * the robot will be positioned between them, it has to drive to look at a point cloud
     * the first point cloud contains one object type, the second contains two
     * the first point cloud is set and then the second one is set bigger until the robot chooses it
     *
     */
    void pointCloudOneTypeSizeTest() {
        bool doTest = true;
        mNodeHandle.param("point_cloud_one_type_size/test", doTest, true);

        if (!doTest)
            return;

        ROS_INFO("Running point cloud one type size test");

        // get parameters
        int stepSize;
        if (!this->getParameter("point_cloud_one_type_size/step_size", stepSize))
            return;

        int iterations;
        if (!this->getParameter("point_cloud_one_type_size/iterations", iterations))
            return;

        double distance;
        if (!this->getParameter("point_cloud_one_type_size/distance", distance))
            return;

        std::vector<std::string> objectTypeNames;
        objectTypeNames.push_back("CeylonTea");
        objectTypeNames.push_back("CoffeeFilters2");

        std::map<std::string, std::vector<double>> typeToOrientation;
        for (unsigned int i = 0; i < objectTypeNames.size(); i++)
        {
            std::string parameter = "point_cloud_one_type_size/" + objectTypeNames[i] + "_orientation";
            typeToOrientation[objectTypeNames[i]] = std::vector<double>();
            if (!this->getParameter(parameter, typeToOrientation[objectTypeNames[i]]))
                return;
        }

        std::vector<double> elementDeviationList;
        if (!this->getParameter("point_cloud_one_type_size/element_deviation", elementDeviationList))
            return;
        SimpleVector3 elementDeviation(elementDeviationList[0], elementDeviationList[1], elementDeviationList[2]);

        world_model::EmptyViewportList empty;
        ros::ServiceClient emptyViewportsClient = mGlobalNodeHandle.serviceClient<world_model::EmptyViewportList>("/env/world_model/empty_viewport_list");


        ROS_INFO("Generiere Häufungspunkte");
        // Häufungspunkte
        int hpSize = 2;
        SimpleVector3* hp = new SimpleVector3[hpSize];
        hp[0] = SimpleVector3(21.1, 11.4, 1.32);
        hp[1] = SimpleVector3(37.0, 2.0, 1.32);

        SimpleQuaternion* quaternions = new SimpleQuaternion[hpSize];
        quaternions[0] = SimpleQuaternion(0.260, 0.000, 0.000, 0.966);
        quaternions[1] = SimpleQuaternion(0.967, 0.000, 0.000, -0.256);

        MapHelper mapHelper;

        ofstream outputFile(mOutputPath + "testPointCloudOneTypeSize.dat");
        if(outputFile.is_open()) {
            outputFile << "#Robot stands between the point clouds with distance parameter " << distance << "\n",
            outputFile << "#Size of the cloud with 2 object types\t\tSize of the cloud with 1 object type when it is first chosen \n";
        }

        for (int i = 1; i < iterations + 1; i += stepSize) {
            SetAttributedPointCloud apc;

            //create second point cloud that contains multiple object types
            for (int cnt = 0; cnt < i; cnt++) {
                SimpleVector3 randomVector;
                int8_t occupancyValue;
                do {
                    randomVector = MathHelper::getRandomVector(hp[1], elementDeviation);
                    occupancyValue = mapHelper.getRaytracingMapOccupancyValue(randomVector);
                } while(!mapHelper.isOccupancyValueAcceptable(occupancyValue));

                vector<double> typeOrientation = typeToOrientation[objectTypeNames[cnt % objectTypeNames.size()]];
                SimpleQuaternion orientation = this->getOrientation(quaternions[1], typeOrientation[0], typeOrientation[1], typeOrientation[2]);

                pbd_msgs::PbdAttributedPoint element;

                geometry_msgs::Pose pose;
                pose.orientation.w = orientation.w();
                pose.orientation.x = orientation.x();
                pose.orientation.y = orientation.y();
                pose.orientation.z = orientation.z();
                pose.position.x = randomVector[0];
                pose.position.y = randomVector[1];
                pose.position.z = randomVector[2];

                element.type = objectTypeNames[cnt % objectTypeNames.size()];
                element.pose = pose;

                apc.request.point_cloud.elements.push_back(element);
            }

            ROS_INFO("Setze initiale Pose");
            // interpoliere zwischen Roboter Position und Punktwolke
            SimpleVector3 position;
            position[0] = 29.4 + (1.0 - distance) * (hp[1][0] - 29.4);
            position[1] = 7.3 + (1.0 - distance) * (hp[1][1] - 7.3);
            position[2] = 1.32;

            geometry_msgs::Pose initialPose;

            //Pose in der Mitte von CeylonTea und CoffeeFilters
            initialPose.position.x = position[0];
            initialPose.position.y = position[1];
            initialPose.position.z = position[2];

            initialPose.orientation.w = 0.5;
            initialPose.orientation.x = 0.0;
            initialPose.orientation.y = 0.0;
            initialPose.orientation.z = -0.86;

            this->setInitialPose(initialPose);
            apc.request.pose = initialPose;

            GetNextBestView getNBV;

            int pcSize = 0;
            bool stop = false;
            while (!stop) {
                //make point cloud with one object type bigger by adding one element until robot chooses it
                SimpleVector3 randomVector;
                int8_t occupancyValue;
                do {
                    randomVector = MathHelper::getRandomVector(hp[0], elementDeviation);
                    occupancyValue = mapHelper.getRaytracingMapOccupancyValue(randomVector);
                } while(!mapHelper.isOccupancyValueAcceptable(occupancyValue));

                vector<double> typeOrientation = typeToOrientation[objectTypeNames[0]];
                SimpleQuaternion orientation = this->getOrientation(quaternions[0], typeOrientation[0], typeOrientation[1], typeOrientation[2]);

                pbd_msgs::PbdAttributedPoint element;

                geometry_msgs::Pose pose;
                pose.orientation.w = orientation.w();
                pose.orientation.x = orientation.x();
                pose.orientation.y = orientation.y();
                pose.orientation.z = orientation.z();
                pose.position.x = randomVector[0];
                pose.position.y = randomVector[1];
                pose.position.z = randomVector[2];

                element.type = objectTypeNames[0];
                element.pose = pose;

                pcSize++;
                apc.request.point_cloud.elements.push_back(element);

                emptyViewportsClient.call(empty);

                NBV.processSetPointCloudServiceCall(apc.request, apc.response);

                getNBV.request.current_pose = initialPose;
                NBV.processGetNextBestViewServiceCall(getNBV.request, getNBV.response);

                float robotX = getNBV.response.resulting_pose.position.x;
                float robotY = getNBV.response.resulting_pose.position.y;

                float distanceToPC = sqrt(pow(robotX - hp[1](0), 2) + pow(robotY - hp[1](1), 2));
                if (sqrt(pow(robotX - hp[0](0), 2) + pow(robotY - hp[0](1), 2)) < sqrt(pow(robotX - hp[1](0), 2) + pow(robotY - hp[1](1), 2))) {
                    if(outputFile.is_open()) {
                        // i: size of point cloud with 2 object types
                        // pcSize: size of point cloud with 1 object type when it is chosen first
                        const char* line = (boost::to_string(i) + "\t\t\t\t\t\t" + boost::to_string(pcSize) + "\n").c_str();
                        outputFile << line;
                    }
                    ROS_INFO("Roboter bewegt sich zur 1-Objekttypen-Punktwolke, die %d Hypothesen enthält, wobei die 2-Objekttyp-Punktwolke %d Hypothesen enthält",
                        pcSize, i);
                    distanceToPC = sqrt(pow(robotX - hp[0](0), 2) + pow(robotY - hp[0](1), 2));

                    stop = true;
                }
                ROS_INFO("Abstand zur Punktwolke: %f", distanceToPC);
            }
        }

        if(outputFile.is_open()) {
            outputFile << "\n\n";
        }

        outputFile.close();
    }

    /*
     * this test case uses a scene with 2 far apart point clouds
     * the robot will be positioned between them, it has to drive to look at a point cloud
     * the first point cloud contains one object type, the second contains two
     * the second point cloud is set and then the first one is set bigger until the robot chooses it
     *
     */
    void pointCloudTwoTypesSizeTest() {
        bool doTest = true;
        mNodeHandle.param("point_cloud_two_types_size/test", doTest, true);

        if (!doTest)
            return;

        ROS_INFO("Running point cloud two types size test");

        // get parameters
        int stepSize;
        if (!this->getParameter("point_cloud_two_types_size/step_size", stepSize))
            return;

        int iterations;
        if (!this->getParameter("point_cloud_two_types_size/iterations", iterations))
            return;

        double distance;
        if (!this->getParameter("point_cloud_two_types_size/distance", distance))
            return;

        std::vector<std::string> objectTypeNames;
        objectTypeNames.push_back("CeylonTea");
        objectTypeNames.push_back("CoffeeFilters2");

        std::map<std::string, std::vector<double>> typeToOrientation;
        for (unsigned int i = 0; i < objectTypeNames.size(); i++)
        {
            std::string parameter = "point_cloud_two_types_size/" + objectTypeNames[i] + "_orientation";
            typeToOrientation[objectTypeNames[i]] = std::vector<double>();
            if (!this->getParameter(parameter, typeToOrientation[objectTypeNames[i]]))
                return;
        }

        std::vector<double> elementDeviationList;
        if (!this->getParameter("point_cloud_two_types_size/element_deviation", elementDeviationList))
            return;
        SimpleVector3 elementDeviation(elementDeviationList[0], elementDeviationList[1], elementDeviationList[2]);

        world_model::EmptyViewportList empty;
        ros::ServiceClient emptyViewportsClient = mGlobalNodeHandle.serviceClient<world_model::EmptyViewportList>("/env/world_model/empty_viewport_list");

        ROS_INFO("Generiere Häufungspunkte");
        // Häufungspunkte
        int hpSize = 2;
        SimpleVector3* hp = new SimpleVector3[hpSize];
        hp[0] = SimpleVector3(21.1, 11.4, 1.32);
        hp[1] = SimpleVector3(37.0, 2.0, 1.32);

        SimpleQuaternion* quaternions = new SimpleQuaternion[hpSize];
        quaternions[0] = SimpleQuaternion(0.260, 0.000, 0.000, 0.966);
        quaternions[1] = SimpleQuaternion(0.967, 0.000, 0.000, -0.256);

        MapHelper mapHelper;

        ofstream outputFile(mOutputPath + "testPointCloudTwoTypesSize.dat");
        if(outputFile.is_open()) {
            outputFile << "#Robot stands between the point clouds with distance parameter " << distance << "\n",
            outputFile << "#Size of the cloud with 1 object type\tSize of the cloud with 2 object types when it is first chosen \n";
        }


        for (int i = 1; i < iterations + 1; i += stepSize) {
            SetAttributedPointCloud apc;

            //create first point cloud that contains only one object type
            for (int cnt = 0; cnt < i; cnt++) {
                SimpleVector3 randomVector;
                int8_t occupancyValue;
                do {
                    randomVector = MathHelper::getRandomVector(hp[0], elementDeviation);
                    occupancyValue = mapHelper.getRaytracingMapOccupancyValue(randomVector);
                } while(!mapHelper.isOccupancyValueAcceptable(occupancyValue));

                vector<double> typeOrientation = typeToOrientation[objectTypeNames[0]];
                SimpleQuaternion orientation = this->getOrientation(quaternions[0], typeOrientation[0], typeOrientation[1], typeOrientation[2]);

                pbd_msgs::PbdAttributedPoint element;

                geometry_msgs::Pose pose;
                pose.orientation.w = orientation.w();
                pose.orientation.x = orientation.x();
                pose.orientation.y = orientation.y();
                pose.orientation.z = orientation.z();
                pose.position.x = randomVector[0];
                pose.position.y = randomVector[1];
                pose.position.z = randomVector[2];

                element.type = objectTypeNames[0];
                element.pose = pose;

                apc.request.point_cloud.elements.push_back(element);
            }

            ROS_INFO("Setze initiale Pose");
            // interpoliere zwischen Roboter Position und Punktwolke
            SimpleVector3 position;
            position[0] = 29.4 + (1.0 - distance) * (hp[0][0] - 29.4);
            position[1] = 7.3 + (1.0 - distance) * (hp[0][1] - 7.3);
            position[2] = 1.32;

            geometry_msgs::Pose initialPose;

            //Pose in der Mitte von CeylonTea und CoffeeFilters
            initialPose.position.x = position[0];
            initialPose.position.y = position[1];
            initialPose.position.z = position[2];

            initialPose.orientation.w = 0.5;
            initialPose.orientation.x = 0.0;
            initialPose.orientation.y = 0.0;
            initialPose.orientation.z = -0.86;

            this->setInitialPose(initialPose);
            apc.request.pose = initialPose;

            GetNextBestView getNBV;

            int pcSize = 0;
            bool stop = false;
            while (!stop) {
                //make point cloud with one object type bigger by adding one element until robot chooses it
                SimpleVector3 randomVector;
                int8_t occupancyValue;
                do {
                    randomVector = MathHelper::getRandomVector(hp[1], elementDeviation);
                    occupancyValue = mapHelper.getRaytracingMapOccupancyValue(randomVector);
                } while(!mapHelper.isOccupancyValueAcceptable(occupancyValue));

                vector<double> typeOrientation = typeToOrientation[objectTypeNames[pcSize % objectTypeNames.size()]];
                SimpleQuaternion orientation = this->getOrientation(quaternions[1], typeOrientation[0], typeOrientation[1], typeOrientation[2]);

                pbd_msgs::PbdAttributedPoint element;

                geometry_msgs::Pose pose;
                pose.orientation.w = orientation.w();
                pose.orientation.x = orientation.x();
                pose.orientation.y = orientation.y();
                pose.orientation.z = orientation.z();
                pose.position.x = randomVector[0];
                pose.position.y = randomVector[1];
                pose.position.z = randomVector[2];

                element.type = objectTypeNames[pcSize % objectTypeNames.size()];
                element.pose = pose;

                pcSize++;
                apc.request.point_cloud.elements.push_back(element);

                emptyViewportsClient.call(empty);

                NBV.processSetPointCloudServiceCall(apc.request, apc.response);

                getNBV.request.current_pose = initialPose;
                NBV.processGetNextBestViewServiceCall(getNBV.request, getNBV.response);

                float robotX = getNBV.response.resulting_pose.position.x;
                float robotY = getNBV.response.resulting_pose.position.y;

                float distanceToPC = sqrt(pow(robotX - hp[0](0), 2) + pow(robotY - hp[0](1), 2));
                if (sqrt(pow(robotX - hp[1](0), 2) + pow(robotY - hp[1](1), 2)) < sqrt(pow(robotX - hp[0](0), 2) + pow(robotY - hp[0](1), 2))) {
                    if(outputFile.is_open()) {
                        // i: size of point cloud with 1 object type
                        // pcSize: size of point cloud with 2 object types when it is chosen first
                        const char* line = (boost::to_string(i) + "\t\t\t\t\t" + boost::to_string(pcSize) + "\n").c_str();
                        outputFile << line;
                    }
                    ROS_INFO("Roboter bewegt sich zur 2-Objekttypen-Punktwolke, die %d Hypothesen enthält, wobei die 1-Objekttyp-Punktwolke %d Hypothesen enthält",
                        pcSize, i);
                    distanceToPC = sqrt(pow(robotX - hp[1](0), 2) + pow(robotY - hp[1](1), 2));

                    stop = true;
                }
                ROS_INFO("Abstand zur Punktwolke: %f", distanceToPC);
            }
        }

        if(outputFile.is_open()) {
            outputFile << "\n\n";
        }

        outputFile.close();
    }

    void sideObjectTest()
    {
        bool doTest = true;
        mNodeHandle.param("side_objects/test", doTest, true);

        if (!doTest)
            return;

        ROS_INFO("Running side object test");

        MapHelper mapHelper;

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

        this->setInitialPose(initialPose);

        for (int i = 0; i < stepNumber; i++)
        {
            ROS_INFO_STREAM("Step " << i << " of side object test");

            int stepOffset = i * stepSize;

            SetAttributedPointCloud apc;

            MILDRobotModel robot;
            SimpleVector3 position(initialPose.position.x, initialPose.position.y, initialPose.position.z);
            SimpleQuaternion orientation(initialPose.orientation.w, initialPose.orientation.x, initialPose.orientation.y, initialPose.orientation.z);
            MILDRobotStatePtr robotStatePtr(new MILDRobotState());
            robotStatePtr = boost::static_pointer_cast<MILDRobotState>(robot.calculateRobotState(robotStatePtr, position, orientation));

            apc.request.pose = initialPose;

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
                        occupancyValue = mapHelper.getRaytracingMapOccupancyValue(randomVector);
                    } while(!mapHelper.isOccupancyValueAcceptable(occupancyValue));
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

                pbd_msgs::PbdAttributedPoint element;
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

            NBV.processSetPointCloudServiceCall(apc.request, apc.response);

            getNBV.request.current_pose = initialPose;

            ROS_INFO("Asking for next best view");
            NBV.processGetNextBestViewServiceCall(getNBV.request, getNBV.response);

            if (!getNBV.response.found)
            {
                ROS_ERROR("No next best view found!");
                continue;
            }

            ROS_INFO("Got next best view");

            SimpleVector3 pos(initialPose.position.x, initialPose.position.y, initialPose.position.z);
            SimpleQuaternion q(initialPose.orientation.w, initialPose.orientation.x, initialPose.orientation.y, initialPose.orientation.z);

            NextBestViewCalculator nbvCalc = NBV.getCalculator();
            ViewportPoint viewport;
            nbvCalc.doFrustumCulling(pos, q, nbvCalc.getActiveIndices(), viewport);

            viewport.object_type_set = typeSet;

            ROS_INFO_STREAM("Original state: pan " << robotStatePtr->pan << " tilt " << robotStatePtr->tilt
                                        << " rotation " << robotStatePtr->rotation
                                        << " x " << robotStatePtr->x << " y " << robotStatePtr->y);
            if (nbvCalc.getRatingModule()->setSingleScoreContainer(viewport, viewport))
                ROS_INFO_STREAM("Original viewport: utility " << viewport.score->getUtility() << " inverseCosts " << viewport.score->getInverseCosts()
                                    << " rating " << viewport.score->getUtility() * viewport.score->getInverseCosts());
            else
                ROS_INFO("Original viewport utility <= 0");

            TriggerFrustumVisualization triggerFrustumVis;
            triggerFrustumVis.request.current_pose = viewport.getPose();
            NBV.processTriggerOldFrustumVisualization(triggerFrustumVis.request, triggerFrustumVis.response);

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

};

test_suite* init_unit_test_suite( int argc, char* argv[] )
{
    ros::init(argc, argv, "nbv");
    ros::start();

    ros::Duration(5).sleep();

    test_suite* evaluation = BOOST_TEST_SUITE("Evaluation NBV with different scenes");

    boost::shared_ptr<ParametersTest> testPtr(new ParametersTest());

    evaluation->add(BOOST_CLASS_TEST_CASE(&ParametersTest::pointCloudOneTypeSizeTest, testPtr));
    evaluation->add(BOOST_CLASS_TEST_CASE(&ParametersTest::pointCloudTwoTypesSizeTest, testPtr));
    evaluation->add(BOOST_CLASS_TEST_CASE(&ParametersTest::sideObjectTest, testPtr));

    framework::master_test_suite().add(evaluation);

    return 0;
}
