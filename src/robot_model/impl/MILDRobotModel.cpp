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
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

namespace next_best_view {
    MILDRobotModel::MILDRobotModel() : RobotModel() {

        ros::NodeHandle n;
        navigationCostClient = n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
        double mOmegaPan_, mOmegaTilt_, mOmegaRot_;
        n.param("mOmegaPan", mOmegaPan_, 1.0);
        n.param("mOmegaTilt", mOmegaTilt_, 1.0);
        n.param("mOmegaRot", mOmegaRot_, 2.0);
        mOmegaPan = mOmegaPan_;
        mOmegaTilt = mOmegaTilt_;
        mOmegaRot = mOmegaRot_;
		this->setPanAngleLimits(0, 0);
		this->setTiltAngleLimits(0, 0);
		this->setRotationAngleLimits(0, 0);
	}

	MILDRobotModel::~MILDRobotModel() {}

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
        nav_msgs::Path path = getNavigationPath(sourcePosition, targetPosition);
        return !path.poses.empty();
    }

  //Comment?
    RobotStatePtr MILDRobotModel::calculateRobotState(const RobotStatePtr &sourceRobotState, const SimpleVector3 &position, const SimpleQuaternion &orientation)
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

		return targetMILDRobotState;
	}

    float MILDRobotModel::getMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState)
    {
        MILDRobotStatePtr sourceMILDRobotState = boost::static_pointer_cast<MILDRobotState>(sourceRobotState);
        MILDRobotStatePtr targetMILDRobotState = boost::static_pointer_cast<MILDRobotState>(targetRobotState);

        float panDiff = targetMILDRobotState->pan - sourceMILDRobotState->pan;
        float tiltDiff = targetMILDRobotState->tilt - sourceMILDRobotState->tilt;
        float rotDiff = targetMILDRobotState->rotation - sourceMILDRobotState->rotation;

        float distance, costs ;
        nav_msgs::Path path;
        ROS_INFO_STREAM("Calculate path from (" << sourceMILDRobotState->x << ", " << sourceMILDRobotState->y << ") to (" << targetMILDRobotState->x << ", "<< targetMILDRobotState->y << ")");

        geometry_msgs::Point sourcePoint, targetPoint;
        sourcePoint.x = sourceMILDRobotState->x;
        sourcePoint.y = sourceMILDRobotState->y;
        sourcePoint.z = 0;
        targetPoint.x = targetMILDRobotState->x;
        targetPoint.y = targetMILDRobotState->y;
        targetPoint.z = 0;
        path = getNavigationPath(sourcePoint, targetPoint);

        if (!path.poses.empty())
        {
            unsigned int size = path.poses.size();
            distance = 0;
            for (unsigned int i = 0; i < size ; i++)
            {
                ROS_INFO_STREAM("Path (" << path.poses[i].pose.position.x << ", " << path.poses[i].pose.position.y << ")");
            }
            //Calculate distance from source point to firste point in path
            distance += sqrt(pow(sourcePoint.x - path.poses[0].pose.position.x, 2) + pow(sourcePoint.y - path.poses[0].pose.position.y, 2));
            //Calculate path length
            for (unsigned int i = 0; i < size - 1 ; i++)
            {
                distance += sqrt(pow(path.poses[i].pose.position.x - path.poses[i+1].pose.position.x, 2) + pow(path.poses[i].pose.position.y - path.poses[i+1].pose.position.y, 2));
            }

            float panSpan = mPanLimits.get<1>() - mPanLimits.get<0>();
            float tiltSpan = mTiltLimits.get<1>() - mTiltLimits.get<0>();
            float rotationCosts = (mOmegaTilt * abs(panDiff) + mOmegaPan * abs(tiltDiff) + mOmegaRot * fminf(abs(rotDiff), (2 * M_PI - abs(rotDiff)))) / (mOmegaTilt * tiltSpan + mOmegaPan * panSpan + mOmegaRot * M_PI);
            costs = rotationCosts + (distance < 1E-7 ? 0.0 : 10.0 * distance);
        }
        else
        {
            costs = -1;
            ROS_ERROR("Could not get navigation path..");
        }
        return costs;
    }

    nav_msgs::Path MILDRobotModel::getNavigationPath(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition)
    {
        nav_msgs::GetPlan srv;
        srv.request.start.header.frame_id = "map";
        srv.request.goal.header.frame_id = "map";
        srv.request.start.pose.position = sourcePosition;
        srv.request.goal.pose.position = targetPosition;

        srv.request.tolerance = 1.0f;
        nav_msgs::Path path;
        if (navigationCostClient.call(srv))
        {
            path = srv.response.plan;
            ROS_INFO_STREAM("Path size:" << path.poses.size());
        }
        else
        {
            ROS_ERROR("Failed to call the global planner.");
        }
        return path;
    }
}

