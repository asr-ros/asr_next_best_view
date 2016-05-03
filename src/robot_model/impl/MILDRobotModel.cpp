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
    MILDRobotModel::MILDRobotModel() : RobotModel() {
        ros::NodeHandle n("nbv_robot_model");
        navigationCostClient = n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
        mDebugHelperPtr = DebugHelper::getInstance();
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
		mPanLimits.get<0>() = MathHelper::degToRad(minAngleDegrees);
		mPanLimits.get<1>() = MathHelper::degToRad(maxAngleDegrees);
	}

	void MILDRobotModel::setTiltAngleLimits(float minAngleDegrees, float maxAngleDegrees) {
		mTiltLimits.get<0>() = MathHelper::degToRad(minAngleDegrees);
		mTiltLimits.get<1>() = MathHelper::degToRad(maxAngleDegrees);
	}

	void MILDRobotModel::setRotationAngleLimits(float minAngleDegrees, float maxAngleDegrees) {
        mRotationLimits.get<0>() = MathHelper::degToRad(minAngleDegrees);
		mRotationLimits.get<1>() = MathHelper::degToRad(maxAngleDegrees);
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

  //Comment?
    //Solves the inverse kinematical problem for an given robot state and a pose for the camera

    RobotStatePtr MILDRobotModel::calculateRobotState(const RobotStatePtr &sourceRobotState, const SimpleVector3 &position, const SimpleQuaternion &orientation)
    {
        mDebugHelperPtr->writeNoticeably("STARTING CALCULATE-ROBOT-STATE METHOD", DebugHelper::ROBOT_MODEL);
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

        mDebugHelperPtr->write(std::stringstream() << "Source robot state: (Pan: " << sourceMILDRobotState->pan
                                << ", Tilt: " << sourceMILDRobotState->tilt
                                << ", Rotation " << sourceMILDRobotState->rotation
                                << ", X:" << sourceMILDRobotState->x
                                << ", Y:" << sourceMILDRobotState->y << ")",
                    DebugHelper::ROBOT_MODEL);
        mDebugHelperPtr->write(std::stringstream() << "Target View Position: " << position[0] << ", " << position[1] << ", " << position[2],
                    DebugHelper::ROBOT_MODEL);
        mDebugHelperPtr->write(std::stringstream() << "Target View Orientation: " << orientation.w() << ", " << orientation.x() << ", " << orientation.y()<< ", " << orientation.z(),
                    DebugHelper::ROBOT_MODEL);

		double alpha = sphereCoords[2] - currentPhi - currentRho;
		alpha = alpha > M_PI ? alpha - 2 * M_PI : alpha;
		alpha = alpha < -M_PI ? alpha + 2 * M_PI : alpha;

		glp_prob *lp;

		int rows = 1;
		int cols = 4;
		int glIdx[1+cols*rows];
		int glColIdx[1+cols*rows];
		double glCoef[1+cols*rows];

		lp = glp_create_prob();
		glp_set_prob_name(lp, "angle_prob");
		glp_set_obj_dir(lp, GLP_MIN);
		// omega_pan * x_pan + omega_rot * x_rot
		glp_add_cols(lp, cols);

		int colIdx = 1;
		glp_set_col_name(lp, colIdx, "x_pan+");
		double maxPanPlus = abs(phiMax - currentPhi);
		glp_set_col_bnds(lp, colIdx, (maxPanPlus == 0.0 ? GLP_FX : GLP_DB), 0.0, maxPanPlus);
		glp_set_obj_coef(lp, colIdx, mOmegaPan);

		colIdx++;
		glp_set_col_name(lp, colIdx, "x_pan-");
		double maxPanMinus = abs(currentPhi - phiMin);
		glp_set_col_bnds(lp, colIdx, (maxPanMinus == 0.0 ? GLP_FX : GLP_DB), 0.0, maxPanMinus);
		glp_set_obj_coef(lp, colIdx, mOmegaPan);

		colIdx++;
		glp_set_col_name(lp, colIdx, "x_rot+");
		glp_set_col_bnds(lp, colIdx, GLP_DB, 0, M_PI);
		glp_set_obj_coef(lp, colIdx, mOmegaRot);

		colIdx++;
		glp_set_col_name(lp, colIdx, "x_rot-");
		glp_set_col_bnds(lp, colIdx, GLP_DB, 0.0, M_PI);
		glp_set_obj_coef(lp, colIdx, mOmegaRot);

		// constraint alpha
		int rowIdx = 1;
		glp_add_rows(lp, rowIdx);
		glp_set_row_name(lp, rowIdx, "alpha");
		glp_set_row_bnds(lp, rowIdx, GLP_FX, alpha, alpha);

		// alpha = x_pan + x_rot
		glIdx[1] = 1; glColIdx[1] = 1; glCoef[1] = 1.0;
		glIdx[2] = 1; glColIdx[2] = 2; glCoef[2] = -1.0;
		glIdx[3] = 1; glColIdx[3] = 3; glCoef[3] = 1.0;
		glIdx[4] = 1; glColIdx[4] = 4; glCoef[4] = -1.0;

		// constraint
		glp_load_matrix(lp, 4, glIdx, glColIdx, glCoef);

		glp_smcp *param = new glp_smcp();
		glp_init_smcp(param);
		param->msg_lev = GLP_MSG_OFF;
		glp_simplex(lp, param);

		int statusCode = glp_get_status(lp);
		if (statusCode != GLP_OPT) {
			ROS_ERROR("No optimal solution found - inverse kinematics");
		}

		double cost_value = glp_get_obj_val(lp);
		double x_pan_plus = glp_get_col_prim(lp, 1);
		double x_pan_minus = glp_get_col_prim(lp, 2);
		double x_rot_plus = glp_get_col_prim(lp, 3);
		double x_rot_minus = glp_get_col_prim(lp, 4);

		// free memory
		glp_delete_prob(lp);

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
        mDebugHelperPtr->write(std::stringstream() << "Target state: (Pan: " << targetMILDRobotState->pan
                                << ", Tilt: " << targetMILDRobotState->tilt
                                << ", Rotation " << targetMILDRobotState->rotation
                                << ", X:" << targetMILDRobotState->x
                                << ", Y:" << targetMILDRobotState->y << ")",
                    DebugHelper::ROBOT_MODEL);
        mDebugHelperPtr->writeNoticeably("ENDING CALCULATE-ROBOT-STATE METHOD", DebugHelper::ROBOT_MODEL);
		return targetMILDRobotState;
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
        double distance, costs;
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
                costs = -1;
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

