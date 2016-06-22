/*
 * PTURoboterState.hpp
 *
 *  Created on: Sep 23, 2014
 *      Author: ralfschleicher
 */

#include <glpk.h>
#include <limits>
#include <ros/ros.h>
#include <ros/node_handle.h>

#include "next_best_view/robot_model/impl/MILDRobotModel.hpp"
#include "next_best_view/robot_model/impl/MILDRobotState.hpp"
#include "next_best_view/helper/MathHelper.hpp"

#include "nav_msgs/GetPlan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"

#include "urdf/model.h"
#include "urdf_model/joint.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
//#include <robot_state_publisher/robot_state_publisher.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

namespace next_best_view {
    MILDRobotModel::MILDRobotModel() : RobotModel(), listener() {
        mDebugHelperPtr = DebugHelper::getInstance();
        mDebugHelperPtr->write(std::stringstream() << "STARTING MILD ROBOT MODEL", DebugHelper::ROBOT_MODEL);

        ros::NodeHandle n("nbv_robot_model");
        navigationCostClient = n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
        double mOmegaPan_, mOmegaTilt_, mOmegaRot_, tolerance_, speedFactorPTU_,speedFactorBaseMove_,speedFactorBaseRot_, mSigma_;
        bool useGlobalPlanner_;
        n.getParam("mOmegaPan", mOmegaPan_);
        n.getParam("mOmegaTilt", mOmegaTilt_);
        n.getParam("mOmegaRot", mOmegaRot_);
        n.getParam("speedFactorPTU", speedFactorPTU_);
        n.getParam("speedFactorBaseMove", speedFactorBaseMove_);
        n.getParam("speedFactorBaseRot", speedFactorBaseRot_);
        n.getParam("tolerance", tolerance_);
        n.getParam("useGlobalPlanner", useGlobalPlanner_);
        n.getParam("mSigma", mSigma_);

        useGlobalPlanner = useGlobalPlanner_;
        if (useGlobalPlanner_)
        {
            mDebugHelperPtr->write("Use of global planner ENABLED", DebugHelper::PARAMETERS);
        }
        else
        {
            mDebugHelperPtr->write("Use of global planner DISABLED. Using simplified calculation instead", DebugHelper::PARAMETERS);
        }

        mDebugHelperPtr->write(std::stringstream() << "speedFactorPTU: " << speedFactorPTU_, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "speedFactorBaseMove: " << speedFactorBaseMove_, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "speedFactorBaseRot: " << speedFactorBaseRot_, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "tolerance: " << tolerance_, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "mOmegaPan: " << mOmegaPan_, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "mOmegaTilt: " << mOmegaTilt_, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "mOmegaRot: " << mOmegaRot_, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "mSigma: " << mSigma_, DebugHelper::PARAMETERS);
        mOmegaPan = mOmegaPan_;
        mOmegaTilt = mOmegaTilt_;
        mOmegaRot = mOmegaRot_;
        speedFactorPTU = speedFactorPTU_;
        speedFactorBaseMove = speedFactorBaseMove_;
        speedFactorBaseRot = speedFactorBaseRot_;
        tolerance = tolerance_;
        mSigma = mSigma_;
        this->setPanAngleLimits(0, 0);
        this->setTiltAngleLimits(0, 0);
        this->setRotationAngleLimits(0, 0);
	}

	MILDRobotModel::~MILDRobotModel() {}

    geometry_msgs::Pose MILDRobotModel::getRobotPose()
    {
        geometry_msgs::Pose robotPose;
        tf::StampedTransform transform;
        listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
        robotPose.position.x = transform.getOrigin()[0];
        robotPose.position.y = transform.getOrigin()[1];
        robotPose.position.z = transform.getOrigin()[2];
        tf::quaternionTFToMsg(transform.getRotation(), robotPose.orientation);
        return robotPose;
    }

    geometry_msgs::Pose MILDRobotModel::getCameraPose()
    {
        geometry_msgs::Pose cameraPose;
        tf::StampedTransform transform;
        listener.lookupTransform("/map", "/ptu_mount_link", ros::Time(0), transform);
        cameraPose.position.x = transform.getOrigin()[0];
        cameraPose.position.y = transform.getOrigin()[1];
        cameraPose.position.z = transform.getOrigin()[2];
        tf::quaternionTFToMsg(transform.getRotation(), cameraPose.orientation);
        return cameraPose;
    }

	void MILDRobotModel::setPanAngleLimits(float minAngleDegrees, float maxAngleDegrees) {
        mDebugHelperPtr->write(std::stringstream() << "Setting PAN angle limits to (" << minAngleDegrees << ", " << maxAngleDegrees << ")", DebugHelper::PARAMETERS);
		mPanLimits.get<0>() = MathHelper::degToRad(minAngleDegrees);
		mPanLimits.get<1>() = MathHelper::degToRad(maxAngleDegrees);
	}

	void MILDRobotModel::setTiltAngleLimits(float minAngleDegrees, float maxAngleDegrees) {
        mDebugHelperPtr->write(std::stringstream() << "Setting TILT angle limits to (" << minAngleDegrees << ", " << maxAngleDegrees << ")", DebugHelper::PARAMETERS);
		mTiltLimits.get<0>() = MathHelper::degToRad(minAngleDegrees);
		mTiltLimits.get<1>() = MathHelper::degToRad(maxAngleDegrees);
	}

	void MILDRobotModel::setRotationAngleLimits(float minAngleDegrees, float maxAngleDegrees) {
        mDebugHelperPtr->write(std::stringstream() << "Setting rotation angle limits to (" << minAngleDegrees << ", " << maxAngleDegrees << ")", DebugHelper::PARAMETERS);
        mRotationLimits.get<0>() = MathHelper::degToRad(minAngleDegrees);
		mRotationLimits.get<1>() = MathHelper::degToRad(maxAngleDegrees);
	}

    bool MILDRobotModel::isPositionAllowed(const geometry_msgs::Point &position)
    {
        mDebugHelperPtr->writeNoticeably("STARTING IS-POSITION-ALLOWED METHOD", DebugHelper::ROBOT_MODEL);
        MILDRobotModel::initMapHelper();
        SimpleVector3 pos(position.x, position.y, position.z);
        int8_t occupancyValue = mMapHelperPtr->getRaytracingMapOccupancyValue(pos);
        mDebugHelperPtr->writeNoticeably("ENDING IS-POSITION-ALLOWED METHOD", DebugHelper::ROBOT_MODEL);
        return mMapHelperPtr->isOccupancyValueAcceptable(occupancyValue);
    }

    void MILDRobotModel::initMapHelper() {
        if (mMapHelperPtr == nullptr)
        {
            mDebugHelperPtr->writeNoticeably("INIT MAP_HELPER IN MILDRobotModel", DebugHelper::ROBOT_MODEL);
            mMapHelperPtr = MapHelperPtr(new MapHelper());
            ros::NodeHandle n("nbv_robot_model");
            double colThresh;
            n.param("colThresh", colThresh, 45.0);
            mDebugHelperPtr->write(std::stringstream() << "colThresh: " << colThresh, DebugHelper::PARAMETERS);
            mMapHelperPtr->setCollisionThreshold(colThresh);
            mDebugHelperPtr->writeNoticeably("INIT MAP_HELPER IN MILDRobotModel DONE", DebugHelper::ROBOT_MODEL);
        }
    }

    bool MILDRobotModel::isPoseReachable(const SimpleVector3 &position, const SimpleQuaternion &orientation)
    {
        SimpleVector3 visualAxis = MathHelper::getVisualAxis(orientation);
		SimpleSphereCoordinates sphereCoords = MathHelper::convertC2S(visualAxis);

		return (mTiltLimits.get<0>() <= sphereCoords[1] && sphereCoords[1] <= mTiltLimits.get<1>());
	}

    bool MILDRobotModel::isPositionReachable(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition)
    {
        mDebugHelperPtr->writeNoticeably("STARTING IS-POSITION-REACHABLE METHOD", DebugHelper::ROBOT_MODEL);

        nav_msgs::Path path = getNavigationPath(sourcePosition, targetPosition);

        if (path.poses.empty())
        {
            mDebugHelperPtr->writeNoticeably("ENDING IS-POSITION-REACHABLE METHOD", DebugHelper::ROBOT_MODEL);
            return false;
        }

        int lastPose = path.poses.size()-1;
        float distanceToLastPoint = sqrt(pow(targetPosition.x - path.poses[lastPose].pose.position.x, 2) + pow(targetPosition.y  - path.poses[lastPose].pose.position.y, 2));
        mDebugHelperPtr->write(std::stringstream() << "Target: " << targetPosition.x << ", " << targetPosition.y, DebugHelper::ROBOT_MODEL);
        mDebugHelperPtr->write(std::stringstream() << "Actual position: " << path.poses[lastPose].pose.position.x << ", " << path.poses[lastPose].pose.position.y,
                    DebugHelper::ROBOT_MODEL);
        mDebugHelperPtr->writeNoticeably("ENDING IS-POSITION-REACHABLE METHOD", DebugHelper::ROBOT_MODEL);
        return distanceToLastPoint < 0.01f;
    }

    float MILDRobotModel::getBase_TranslationalMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState)
    {
        MILDRobotStatePtr sourceMILDRobotState = boost::static_pointer_cast<MILDRobotState>(sourceRobotState);
        MILDRobotStatePtr targetMILDRobotState = boost::static_pointer_cast<MILDRobotState>(targetRobotState);

        float distance ;
        geometry_msgs::Point sourcePoint, targetPoint;
        sourcePoint.x = sourceMILDRobotState->x;
        sourcePoint.y = sourceMILDRobotState->y;
        sourcePoint.z = 0;
        targetPoint.x = targetMILDRobotState->x;
        targetPoint.y = targetMILDRobotState->y;
        targetPoint.z = 0;

        distance = getDistance(sourcePoint, targetPoint);

        float movementCosts = std::exp(-pow((distance-0.0), 2.0)/(2.0*pow(mSigma, 2.0))); // [0, 1]
        return movementCosts;
    }

    float MILDRobotModel::getPTU_PanMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState)
    {
        MILDRobotStatePtr sourceMILDRobotState = boost::static_pointer_cast<MILDRobotState>(sourceRobotState);
        MILDRobotStatePtr targetMILDRobotState = boost::static_pointer_cast<MILDRobotState>(targetRobotState);

        float panDiff = targetMILDRobotState->pan - sourceMILDRobotState->pan;

        float panSpan = mPanLimits.get<1>() - mPanLimits.get<0>();

        if (panSpan == 0)
            return 1.0;

        float ptuPanCosts = fabs(panDiff)/ panSpan;

        return 1.0 - ptuPanCosts;
    }

    float MILDRobotModel::getPTU_TiltMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState)
    {
        MILDRobotStatePtr sourceMILDRobotState = boost::static_pointer_cast<MILDRobotState>(sourceRobotState);
        MILDRobotStatePtr targetMILDRobotState = boost::static_pointer_cast<MILDRobotState>(targetRobotState);
        float tiltDiff = targetMILDRobotState->tilt - sourceMILDRobotState->tilt;

        float tiltSpan = mTiltLimits.get<1>() - mTiltLimits.get<0>();

        if (tiltSpan == 0)
            return 1.0;

        float ptuTiltCosts = fabs(tiltDiff)/ tiltSpan;

        return 1.0 - ptuTiltCosts;
    }

    float MILDRobotModel::getBase_RotationalMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState)
    {
        MILDRobotStatePtr sourceMILDRobotState = boost::static_pointer_cast<MILDRobotState>(sourceRobotState);
        MILDRobotStatePtr targetMILDRobotState = boost::static_pointer_cast<MILDRobotState>(targetRobotState);

        float rotationCosts;

        if (fabs(targetMILDRobotState->y - sourceMILDRobotState->y) < 0.00001 && fabs(targetMILDRobotState->x - sourceMILDRobotState->x) < 0.00001) {
            float rotDiff = targetMILDRobotState->rotation - sourceMILDRobotState->rotation;

            rotationCosts = std::min(fabs(rotDiff), (float)(2.0f*M_PI-fabs(rotDiff)))/ (2.0 * M_PI);

        }
        else {
            float angleBetweenPoints = std::atan2(targetMILDRobotState->y - sourceMILDRobotState->y, targetMILDRobotState->x - sourceMILDRobotState->x);
            if(angleBetweenPoints < 0.0) angleBetweenPoints += 2.0 * M_PI;

            float sourceRotDiff = sourceMILDRobotState->rotation - angleBetweenPoints;
            float targetRotDiff = targetMILDRobotState->rotation - angleBetweenPoints;

            rotationCosts = std::min(fabs(sourceRotDiff), (float)(2.0f*M_PI-fabs(sourceRotDiff)))
                                    + std::min(fabs(targetRotDiff), (float)(2.0f*M_PI-fabs(targetRotDiff)));
            rotationCosts /= 2.0 * M_PI;

        }
        return 1.0 - rotationCosts;
    }

    float MILDRobotModel::getDistance(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition)
    {
        mDebugHelperPtr->writeNoticeably("STARTING GET-DISTANCE METHOD", DebugHelper::ROBOT_MODEL);
        double distance;
        if (useGlobalPlanner) //Use global planner to calculate distance
        {
            nav_msgs::Path path;
            mDebugHelperPtr->write(std::stringstream() << "Calculate path from (" << sourcePosition.x << ", "
                                    << sourcePosition.y << ") to (" << targetPosition.x << ", "<< targetPosition.y << ")",
                        DebugHelper::ROBOT_MODEL);

            path = getNavigationPath(sourcePosition, targetPosition);

            if (!path.poses.empty())
            {
                unsigned int size = path.poses.size();
                distance = 0;
                for (unsigned int i = 0; i < size ; i++)
                {
                    mDebugHelperPtr->write(std::stringstream() << "Path (" << path.poses[i].pose.position.x << ", " << path.poses[i].pose.position.y << ")",
                                DebugHelper::ROBOT_MODEL);
                }
                //Calculate distance from source point to first point in path
                distance += sqrt(pow(sourcePosition.x - path.poses[0].pose.position.x, 2) + pow(sourcePosition.y - path.poses[0].pose.position.y, 2));
                //Calculate path length
                for (unsigned int i = 0; i < size - 1 ; i++)
                {
                    distance += sqrt(pow(path.poses[i].pose.position.x - path.poses[i+1].pose.position.x, 2) + pow(path.poses[i].pose.position.y - path.poses[i+1].pose.position.y, 2));
                }
            }
            else
            {
                distance = -1;
                ROS_ERROR("Could not get navigation path..");
            }
        }
        else //Use euclidean distance
        {
            distance = sqrt(pow(sourcePosition.x -targetPosition.x, 2) + pow(sourcePosition.y - targetPosition.y, 2));
        }
        mDebugHelperPtr->write(std::stringstream() << "Calculated distance: " << distance, DebugHelper::ROBOT_MODEL);
        mDebugHelperPtr->write(std::stringstream() << "Euclidian distance: "
                                << sqrt(pow(sourcePosition.x -targetPosition.x, 2) + pow(sourcePosition.y - targetPosition.y, 2)),
                    DebugHelper::ROBOT_MODEL);
        mDebugHelperPtr->writeNoticeably("ENDING GET-DISTANCE METHOD", DebugHelper::ROBOT_MODEL);
        return distance;
    }

    nav_msgs::Path MILDRobotModel::getNavigationPath(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition)
    {
        return getNavigationPath(sourcePosition, targetPosition, 0, 0);
    }

    nav_msgs::Path MILDRobotModel::getNavigationPath(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition, double sourceRotationBase, double targetRotationBase)
    {
        mDebugHelperPtr->writeNoticeably("STARTING GET-NAVIGATION-PATH METHOD", DebugHelper::ROBOT_MODEL);

        nav_msgs::GetPlan srv;
        srv.request.start.header.frame_id = "map";
        srv.request.goal.header.frame_id = "map";
        srv.request.start.pose.position = sourcePosition;
        srv.request.goal.pose.position = targetPosition;
        Eigen::Quaterniond sourceRotationEigen(Eigen::AngleAxisd(sourceRotationBase,Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond targetRotationEigen(Eigen::AngleAxisd(targetRotationBase,Eigen::Vector3d::UnitZ()));
        geometry_msgs::Quaternion sourceRotation;
        sourceRotation.w = sourceRotationEigen.w();
        sourceRotation.x = sourceRotationEigen.x();
        sourceRotation.y = sourceRotationEigen.y();
        sourceRotation.z = sourceRotationEigen.z();
        geometry_msgs::Quaternion targetRotation;
        targetRotation.w = targetRotationEigen.w();
        targetRotation.x = targetRotationEigen.x();
        targetRotation.y = targetRotationEigen.y();
        targetRotation.z = targetRotationEigen.z();
        srv.request.start.pose.orientation = sourceRotation;
        srv.request.goal.pose.orientation = targetRotation;
        srv.request.tolerance = tolerance;

        nav_msgs::Path path;
        if (navigationCostClient.call(srv))
        {
            path = srv.response.plan;
            mDebugHelperPtr->write(std::stringstream() << "Path size:" << path.poses.size(), DebugHelper::ROBOT_MODEL);
        }
        else
        {
            ROS_ERROR("Failed to call the global planner.");
        }

        mDebugHelperPtr->writeNoticeably("ENDING GET-NAVIGATION-PATH METHOD", DebugHelper::ROBOT_MODEL);
        return path;
    }

    // Not fully implemented
    // Guideline: http://www.orocos.org/kdl/examples
    geometry_msgs::Pose MILDRobotModel::calculateCameraPose(const RobotStatePtr &sourceRobotState)
    {
        /*MILDRobotStatePtr sourceMILDRobotState = boost::static_pointer_cast<MILDRobotState>(sourceRobotState);

        urdf::Model * myModel;
        geometry_msgs::Pose cameraPose;

        tf::Transform pan(tf::Quaternion(tf::Vector3(0,0,1), sourceMILDRobotState->pan));
        tf::Transform tilt(tf::Quaternion(tf::Vector3(0,-1,0), sourceMILDRobotState->tilt));

        urdf::Model myModel;
        ros::Rate loop_rate(30);
        //geometry_msgs::Pose cameraPose;

        if(!myModel.initParam("/robot_description"))
        {
            ROS_ERROR("Could not get robot description.");
        }
        else
        {
            KDL::Tree my_tree;
            if (!kdl_parser::treeFromUrdfModel(myModel, my_tree))
            {
                   ROS_ERROR("Failed to construct kdl tree");
            }
            else
            {
                 KDL::Chain chain;
                 my_tree.getChain(myModel.getRoot()->name, "mount_to_camera_left", chain);
                 ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);


                 //robot_state_publisher::RobotStatePublisher* publisher = new robot_state_publisher::RobotStatePublisher(my_tree);
                 //my_tree.getSegment()

                 typedef std::map<std::string, boost::shared_ptr<urdf::Joint> > joint_map;

                 std::map<std::string, double> positions;
                 joint_map joints = myModel.joints_;

                 for(joint_map::const_iterator it =  joints.begin(); it !=  joints.end(); ++it)
                 {
                    std::string name = it->first;
                    urdf::Joint* joint = it->second.get();
                    //joint->dynamics.
                    mDebugHelperPtr->write(name, DebugHelper::ROBOT_MODEL);
                    if(joint->type==urdf::Joint::REVOLUTE || joint->type==urdf::Joint::CONTINUOUS || joint->type==urdf::Joint::PRISMATIC) positions[name] = 0;
                 }
                 while (ros::ok())
                 {
                     //publisher->publishTransforms(positions, ros::Time::now());

                     // Process a round of subscription messages
                     ros::spinOnce();

                     // This will adjust as needed per iteration
                     loop_rate.sleep();
                 }
             }
        } */
        geometry_msgs::Pose myPose;
        return myPose;
    }
}

