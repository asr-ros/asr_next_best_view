
/*
 * AbstractTest.h
 *
 *  Created on: Mar 17, 2015
 *      Author: florianaumann
 */
#define BOOST_TEST_STATIC_LINK

#include <ros/ros.h>
#include <boost/array.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <glpk.h>
#include <map>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/PointCloud2.h>

#include "object_database/ObjectType.h"
#include "object_database/ObjectManager.h"
#include "next_best_view/NextBestView.hpp"
#include "next_best_view/GetNextBestView.h"
#include "next_best_view/GetPointCloud2.h"
#include "next_best_view/SetAttributedPointCloud.h"
#include "next_best_view/ResetCalculator.h"
#include "next_best_view/UpdatePointCloud.h"
#include "pbd_msgs/PbdAttributedPoint.h"
#include "next_best_view/helper/MathHelper.hpp"
#include "next_best_view/helper/MarkerHelper.hpp"
#include "next_best_view/helper/MapHelper.hpp"
#include "next_best_view/helper/TypeHelper.hpp"
#include "next_best_view/robot_model/impl/MILDRobotModel_with_IK.hpp"
#include "next_best_view/robot_model/impl/MILDRobotState.hpp"
#include <tf/tf.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "next_best_view/GetSpaceSampling.h"

using namespace next_best_view;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef boost::shared_ptr<MoveBaseClient> MoveBaseClientPtr;

class BaseTest {
protected:
    ros::NodeHandle mNodeHandle;
    MoveBaseClientPtr mMoveBaseClient;
    ros::Publisher mInitPosePub;
public:
    BaseTest();

    ~BaseTest();

    void evaluateS2CandC2S();

    void solveLinearProblem();

    void visualizeSingleObjectWithNormals();

    void setInitialPose(const geometry_msgs::Pose &initialPose);

    void moveToPose(const geometry_msgs::Pose &pose);

    void visualizeSpaceSampling();

    void waitForEnter();

    SimpleQuaternion euler2Quaternion( const Precision roll, const Precision pitch, const Precision yaw);

    SimpleQuaternion ZXZ2Quaternion( const Precision roll, const Precision pitch, const Precision yaw);
};

