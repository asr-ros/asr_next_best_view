/*
 * PTURoboterState.hpp
 *
 *  Created on: Sep 23, 2014
 *      Author: ralfschleicher
 */

#pragma once

#include <boost/tuple/tuple.hpp>
#include "next_best_view/robot_model/RobotModel.hpp"
#include "typedef.hpp"
#include "geometry_msgs/Pose.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "next_best_view/helper/DebugHelper.hpp"

namespace next_best_view {
	/*!
     * \brief MILDRobotModel implements a model of pan tilt unit robot.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
    class MILDRobotModel : public RobotModel {
    protected:

        DebugHelperPtr mDebugHelperPtr;

		// weighting values
		float mOmegaPan;
		float mOmegaTilt;
        float mOmegaRot;
		float speedFactorPTU;
		float speedFactorBaseMove;
		float speedFactorBaseRot;
		float tolerance;
        float mSigma;
		bool useGlobalPlanner;
        tf::TransformListener listener;

		/*!
		 * contains the lower and upper limit of pan
		 */
		boost::tuple<float, float> mPanLimits;

		/*!
		 * contains the lower and upper limit of tilt
		 */
		boost::tuple<float, float> mTiltLimits;

		/*!
		 * contains the lower and upper limit of rotation
		 */
		boost::tuple<float, float> mRotationLimits;
		
		/*!
		 * Client used for communication with the global_planner to calculate movement costs
		 */
		ros::ServiceClient navigationCostClient;
		
	public:
		/*!
         * \brief constructor of the MILDRobotModel
		 */
        MILDRobotModel();

		/*!
		 * \brief destructor of the MILDRobotModel
		 */
        virtual ~MILDRobotModel();

        /*!
         * Client used for communication with the global_planner to calculate movement costs
         */
         nav_msgs::Path getNavigationPath(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition, double sourceRotationBase, double targetRotationBase);
         nav_msgs::Path getNavigationPath(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition);

		/*!
		 * \brief sets the angle limits of the pan angle.
		 * \param minAngleDegrees the minimum angle in degrees
		 * \param maxAngleDegrees the maximum angle in degrees
		 */
		void setPanAngleLimits(float minAngleDegrees, float maxAngleDegrees);

		/*!
		 * \brief sets the angle limits of the tilt angle.
		 * \param minAngleDegrees the minimum angle in degrees
		 * \param maxAngleDegrees the maximum angle in degrees
		 */
		void setTiltAngleLimits(float minAngleDegrees, float maxAngleDegrees);

		/*!
		 * \brief sets the angle limits of the rotation angle.
		 * \param minAngleDegrees the minimum angle in degrees
		 * \param maxAngleDegrees the maximum angle in degrees
		 */
		void setRotationAngleLimits(float minAngleDegrees, float maxAngleDegrees);

		bool isPoseReachable(const SimpleVector3 &position, const SimpleQuaternion &orientation);
		
		bool isPositionReachable(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition);

		/*!
		 * \brief Calculates the movement costs from sourceRobotState to targetRobotState. Returns -1 if pose is not reachable
		 */
        float getBase_TranslationalMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState);

        float getPTU_TiltMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState);

        float getPTU_PanMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState);

        float getBase_RotationalMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState);
		
		float getDistance(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition);

        //Returns the current pose of the robots base from tf
        geometry_msgs::Pose getRobotPose();
        //Returns the current pose of the robots camera from tf
        geometry_msgs::Pose getCameraPose();

        /*!
         * \brief Uses a given RobotState to calculate the camera frame
         */
        virtual RobotStatePtr calculateRobotState(const RobotStatePtr &sourceRobotState, const SimpleVector3 &position, const SimpleQuaternion &orientation);

        /*!
         * \brief Uses a given RobotState to calculate the camera frame
         */
        geometry_msgs::Pose calculateCameraPose(const RobotStatePtr &sourceRobotState);

	};

    typedef boost::shared_ptr<MILDRobotModel> MILDRobotModelPtr;
}
