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
#include "next_best_view/robot_model/impl/MILDRobotModelWithApproximatedIK.hpp"
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
    MILDRobotModelWithApproximatedIK::MILDRobotModelWithApproximatedIK() : MILDRobotModel() {
        mDebugHelperPtr = DebugHelper::getInstance();
        mDebugHelperPtr->write(std::stringstream() << "STARTING MILD ROBOT MODEL WITH APPROXIMATED IK", DebugHelper::ROBOT_MODEL);

        ros::NodeHandle n("nbv_robot_model");
        navigationCostClient = n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
        double speedFactorPTU_,speedFactorBaseMove_,speedFactorBaseRot_;
        n.getParam("speedFactorPTU", speedFactorPTU_);
        n.getParam("speedFactorBaseMove", speedFactorBaseMove_);
        n.getParam("speedFactorBaseRot", speedFactorBaseRot_);
        mDebugHelperPtr->write(std::stringstream() << "speedFactorPTU: " << speedFactorPTU_, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "speedFactorBaseMove: " << speedFactorBaseMove_, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "speedFactorBaseRot: " << speedFactorBaseRot_, DebugHelper::PARAMETERS);
        speedFactorPTU = speedFactorPTU_;
        speedFactorBaseMove = speedFactorBaseMove_;
        speedFactorBaseRot = speedFactorBaseRot_;
	}

    MILDRobotModelWithApproximatedIK::~MILDRobotModelWithApproximatedIK() {}

  //Comment?
    //Solves the inverse kinematical problem for an given robot state and a pose for the camera

    RobotStatePtr MILDRobotModelWithApproximatedIK::calculateRobotState(const RobotStatePtr &sourceRobotState, const SimpleVector3 &position, const SimpleQuaternion &orientation)
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

        // Truncate pan angle to valid range
        if (currentPhi < phiMin) {
            ROS_WARN_STREAM("Initial Pan-Angle (" << currentPhi * (180/M_PI) << ") was too small.");
            currentPhi = phiMin;
        }
        if (currentPhi > phiMax) {
            ROS_WARN_STREAM("Initial Pan-Angle (" << currentPhi * (180/M_PI) << ") was too large.");
            currentPhi = phiMax;
        }

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

        double x_pan_plus = glp_get_col_prim(lp, 1);
		double x_pan_minus = glp_get_col_prim(lp, 2);
		double x_rot_plus = glp_get_col_prim(lp, 3);
		double x_rot_minus = glp_get_col_prim(lp, 4);

		// free memory
		glp_delete_prob(lp);

		// set pan
		targetMILDRobotState->pan = sourceMILDRobotState->pan + x_pan_plus - x_pan_minus;

        // Truncate pan angle to valid range
        if (targetMILDRobotState->pan < phiMin) {
            ROS_ERROR_STREAM("Calculated Pan-Angle (" << currentPhi * (180/M_PI) << ") is too small.");
//            targetMILDRobotState->pan = phiMin;
        }
        if (targetMILDRobotState->pan > phiMax) {
            ROS_ERROR_STREAM("Calculated Pan-Angle (" << currentPhi * (180/M_PI) << ") is too large.");
//            targetMILDRobotState->pan = phiMax;
        }

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
}

