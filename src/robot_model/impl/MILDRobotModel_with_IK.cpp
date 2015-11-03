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

#include "next_best_view/robot_model/impl/MILDRobotModel_with_IK.hpp"
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
    MILDRobotModelWithIK::MILDRobotModelWithIK() : RobotModel() {
        ros::NodeHandle n("nbv_srv");
        navigationCostClient = n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
        double mOmegaPan_, mOmegaTilt_, mOmegaUseBase_, tolerance_, speedFactorPTU_,speedFactorBaseMove_,speedFactorBaseRot_, mOmegaGlobal_;
        bool useGlobalPlanner_;
        n.getParam("mOmegaPan", mOmegaPan_);
        n.getParam("mOmegaTilt", mOmegaTilt_);
        n.getParam("mOmegaUseBase", mOmegaUseBase_);
        n.getParam("mOmegaGlobal", mOmegaGlobal_);
        n.getParam("speedFactorPTU", speedFactorPTU_);
        n.getParam("speedFactorBaseMove", speedFactorBaseMove_);
        n.getParam("speedFactorBaseRot", speedFactorBaseRot_);
        n.getParam("tolerance", tolerance_);
        n.getParam("useGlobalPlanner", useGlobalPlanner_);
        useGlobalPlanner = useGlobalPlanner_;
        if (useGlobalPlanner_)
        {
            ROS_DEBUG("Use of global planner ENABLED");
        }
        else
        {
            ROS_DEBUG("Use of global planner DISABLED. Using simplified calculation instead...");
        }

        ROS_DEBUG_STREAM("mOmegaUseBase: " << mOmegaUseBase_);
        ROS_DEBUG_STREAM("speedFactorPTU: " << speedFactorPTU_);
        ROS_DEBUG_STREAM("speedFactorBaseMove: " << speedFactorBaseMove_);
        ROS_DEBUG_STREAM("speedFactorBaseRot: " << speedFactorBaseRot_);
        ROS_DEBUG_STREAM("tolerance: " << tolerance_);
        ROS_DEBUG_STREAM("mOmegaPan: " << mOmegaPan_);
        ROS_DEBUG_STREAM("mOmegaTilt: " << mOmegaTilt_);
        ROS_DEBUG_STREAM("mOmegaGlobal: " << mOmegaGlobal_);
        mOmegaPan = mOmegaPan_;
        mOmegaTilt = mOmegaTilt_;
        mOmegaUseBase = mOmegaUseBase_;
        speedFactorPTU = speedFactorPTU_;
        speedFactorBaseMove = speedFactorBaseMove_;
        speedFactorBaseRot = speedFactorBaseRot_;
        mOmegaGlobal = mOmegaGlobal_;
        tolerance = tolerance_;
		this->setPanAngleLimits(0, 0);
		this->setTiltAngleLimits(0, 0);
        this->setRotationAngleLimits(0, 0);
	}

    MILDRobotModelWithIK::~MILDRobotModelWithIK() {}

    geometry_msgs::Pose MILDRobotModelWithIK::getRobotPose()
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

    geometry_msgs::Pose MILDRobotModelWithIK::getCameraPose()
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

    void MILDRobotModelWithIK::setPanAngleLimits(float minAngleDegrees, float maxAngleDegrees) {
		mPanLimits.get<0>() = MathHelper::degToRad(minAngleDegrees);
		mPanLimits.get<1>() = MathHelper::degToRad(maxAngleDegrees);
	}

    void MILDRobotModelWithIK::setTiltAngleLimits(float minAngleDegrees, float maxAngleDegrees) {
		mTiltLimits.get<0>() = MathHelper::degToRad(minAngleDegrees);
		mTiltLimits.get<1>() = MathHelper::degToRad(maxAngleDegrees);
	}

    void MILDRobotModelWithIK::setRotationAngleLimits(float minAngleDegrees, float maxAngleDegrees) {
        mRotationLimits.get<0>() = MathHelper::degToRad(minAngleDegrees);
		mRotationLimits.get<1>() = MathHelper::degToRad(maxAngleDegrees);
	}

    void MILDRobotModelWithIK::setViewPointDistance(float viewPointDistance) {
        this->viewPointDistance = viewPointDistance;
    }

    bool MILDRobotModelWithIK::isPoseReachable(const SimpleVector3 &position, const SimpleQuaternion &orientation)
    {
        SimpleVector3 visualAxis = MathHelper::getVisualAxis(orientation);
		SimpleSphereCoordinates sphereCoords = MathHelper::convertC2S(visualAxis);

		return (mTiltLimits.get<0>() <= sphereCoords[1] && sphereCoords[1] <= mTiltLimits.get<1>());
	}

    bool MILDRobotModelWithIK::isPositionReachable(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition)
    {
        nav_msgs::Path path = getNavigationPath(sourcePosition, targetPosition);

        if (path.poses.empty())
        {
            return false;
        }
        else
        {
            int lastPose = path.poses.size()-1;
            float distanceToLastPoint = sqrt(pow(targetPosition.x - path.poses[lastPose].pose.position.x, 2) + pow(targetPosition.y  - path.poses[lastPose].pose.position.y, 2));
            ROS_DEBUG_STREAM("Target: " << targetPosition.x << ", " << targetPosition.y);
            ROS_DEBUG_STREAM("Actual position: " << path.poses[lastPose].pose.position.x << ", " << path.poses[lastPose].pose.position.y);
            return distanceToLastPoint < 0.01f;
        }
    }

  //Comment?
    //Solves the inverse kinematical problem for an given robot state and a pose for the camera

    RobotStatePtr MILDRobotModelWithIK::calculateRobotState(const RobotStatePtr &sourceRobotState, const SimpleVector3 &position, const SimpleQuaternion &orientation)
    {
		MILDRobotStatePtr sourceMILDRobotState = boost::static_pointer_cast<MILDRobotState>(sourceRobotState);
		MILDRobotStatePtr targetMILDRobotState(new MILDRobotState());

		SimpleVector3 visualAxis = MathHelper::getVisualAxis(orientation);
		SimpleSphereCoordinates sphereCoords = MathHelper::convertC2S(visualAxis);
		while (sphereCoords[2] < 0) { sphereCoords[2] += 2 * M_PI; }
		while (sphereCoords[2] > 2 * M_PI) { sphereCoords[2] -= 2 * M_PI; }

		double phiMin = mPanLimits.get<0>();
		double phiMax = mPanLimits.get<1>();
		double currentPhi = sourceMILDRobotState->pan;
		double currentRho = sourceMILDRobotState->rotation;

        ROS_DEBUG_STREAM("Calculate state for: (Pan: " << sourceMILDRobotState->pan << ", Tilt: " << sourceMILDRobotState->tilt << ", Rotation " << sourceMILDRobotState->rotation << ", X:" << sourceMILDRobotState->x << ", Y:" << sourceMILDRobotState->y << ")");
        ROS_DEBUG_STREAM("Position: " << position[0] << ", " << position[1] << ", " << position[2]);

		double alpha = sphereCoords[2] - currentPhi - currentRho;
		alpha = alpha > M_PI ? alpha - 2 * M_PI : alpha;
		alpha = alpha < -M_PI ? alpha + 2 * M_PI : alpha;

        //BEGIN: Calculate ViewCenterPoint
        //**************************************
        Eigen::Vector3d

        //**************************************
        //END: Calculate ViewCenterPoint

        //BEGIN:Get Parameters from TF-Publishers
        //**************************************
        double h_tilt;  //Height of the tilt axis from ground
        Eigen::Affine3d t_tilt_cam; //Transform between tilted frame and camera frame


        //**************************************
        //END:Get Parameters from TF-Publishers

		// set pan
		targetMILDRobotState->pan = sourceMILDRobotState->pan + x_pan_plus - x_pan_minus;

		// set rotation
		targetMILDRobotState->rotation = sourceMILDRobotState->rotation + x_rot_plus - x_rot_minus;
		while (targetMILDRobotState->rotation < 0) { targetMILDRobotState->rotation += 2 * M_PI; };
		while (targetMILDRobotState->rotation > 2 * M_PI) { targetMILDRobotState->rotation -= 2 * M_PI; };

		// set tilt
		targetMILDRobotState->tilt = sphereCoords[1];

		// set x, y
		targetMILDRobotState->x = position[0];
		targetMILDRobotState->y = position[1];
        ROS_DEBUG_STREAM("Targetstate: (Pan: " << targetMILDRobotState->pan << ", Tilt: " << targetMILDRobotState->tilt << ", Rotation " << targetMILDRobotState->rotation << ", X:" << targetMILDRobotState->x << ", Y:" << targetMILDRobotState->y << ")");
		return targetMILDRobotState;
	}

    float MILDRobotModelWithIK::getMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState)
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

        float mu = 0.0;
        float sigma = 0.5;
        float movementCosts = std::exp(-pow((distance-mu), 2.0)/(2.0*pow(sigma, 2.0))); // [0, 1]
        return movementCosts;
    }

    float MILDRobotModelWithIK::getPTU_PanMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState)
    {
        MILDRobotStatePtr sourceMILDRobotState = boost::static_pointer_cast<MILDRobotState>(sourceRobotState);
        MILDRobotStatePtr targetMILDRobotState = boost::static_pointer_cast<MILDRobotState>(targetRobotState);

        float panDiff = targetMILDRobotState->pan - sourceMILDRobotState->pan;

        float panSpan = mPanLimits.get<1>() - mPanLimits.get<0>();

        float ptuPanCosts = fabs(panDiff)/ panSpan;

        return 1.0 - ptuPanCosts;
    }

    float MILDRobotModelWithIK::getPTU_TiltMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState)
    {
        MILDRobotStatePtr sourceMILDRobotState = boost::static_pointer_cast<MILDRobotState>(sourceRobotState);
        MILDRobotStatePtr targetMILDRobotState = boost::static_pointer_cast<MILDRobotState>(targetRobotState);
        float tiltDiff = targetMILDRobotState->tilt - sourceMILDRobotState->tilt;

        float tiltSpan = mTiltLimits.get<1>() - mTiltLimits.get<0>();

        float ptuTiltCosts = fabs(tiltDiff)/ tiltSpan;

        return 1.0 - ptuTiltCosts;
    }

    float MILDRobotModelWithIK::getRotationCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState)
    {
        MILDRobotStatePtr sourceMILDRobotState = boost::static_pointer_cast<MILDRobotState>(sourceRobotState);
        MILDRobotStatePtr targetMILDRobotState = boost::static_pointer_cast<MILDRobotState>(targetRobotState);

        float rotDiff = targetMILDRobotState->rotation - sourceMILDRobotState->rotation;

        geometry_msgs::Point sourcePoint, targetPoint;
        sourcePoint.x = sourceMILDRobotState->x;
        sourcePoint.y = sourceMILDRobotState->y;
        sourcePoint.z = 0;
        targetPoint.x = targetMILDRobotState->x;
        targetPoint.y = targetMILDRobotState->y;
        targetPoint.z = 0;


        float rotationCosts = std::min(fabs(rotDiff), (float)(2.0f*M_PI-fabs(rotDiff)))/M_PI;
        return 1.0 - rotationCosts;
    }

    float MILDRobotModelWithIK::getDistance(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition)
    {
        float distance, costs ;
        if (useGlobalPlanner) //Use global planner to calculate distance
        {
            nav_msgs::Path path;
            ROS_DEBUG_STREAM("Calculate path from (" << sourcePosition.x << ", " << sourcePosition.y << ") to (" << targetPosition.x << ", "<< targetPosition.y << ")");

            path = getNavigationPath(sourcePosition, targetPosition);

            if (!path.poses.empty())
            {
                unsigned int size = path.poses.size();
                distance = 0;
                for (unsigned int i = 0; i < size ; i++)
                {
                    ROS_DEBUG_STREAM("Path (" << path.poses[i].pose.position.x << ", " << path.poses[i].pose.position.y << ")");
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
                costs = -1;
                ROS_ERROR("Could not get navigation path..");
            }
        }
        else //Use euclidean distance
        {
            distance = sqrt(pow(sourcePosition.x -targetPosition.x, 2) + pow(sourcePosition.y - targetPosition.y, 2));
        }
        ROS_DEBUG_STREAM("Global planner distance: " << distance);
        ROS_DEBUG_STREAM("Euclidian distance: " << sqrt(pow(sourcePosition.x -targetPosition.x, 2) + pow(sourcePosition.y - targetPosition.y, 2)));
        return distance;
    }

    nav_msgs::Path MILDRobotModelWithIK::getNavigationPath(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition)
    {
        nav_msgs::GetPlan srv;
        srv.request.start.header.frame_id = "map";
        srv.request.goal.header.frame_id = "map";
        srv.request.start.pose.position = sourcePosition;
        srv.request.goal.pose.position = targetPosition;
        srv.request.tolerance = tolerance;

        nav_msgs::Path path;
        if (navigationCostClient.call(srv))
        {
            path = srv.response.plan;
            ROS_DEBUG_STREAM("Path size:" << path.poses.size());
        }
        else
        {
            ROS_ERROR("Failed to call the global planner.");
        }
        return path;
    }

    // Not fully implemented
    // Guideline: http://www.orocos.org/kdl/examples
    geometry_msgs::Pose MILDRobotModelWithIK::calculateCameraPose(const RobotStatePtr &sourceRobotState)
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
                    ROS_DEBUG_STREAM(name);
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
    }
}

