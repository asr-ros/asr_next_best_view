/*
 * PTURoboterState.hpp
 *
 *  Created on: Sep 23, 2014
 *      Author: ralfschleicher
 */

#ifndef MILDROBOTMODEL_HPP_
#define MILDROBOTMODEL_HPP_

#include <boost/tuple/tuple.hpp>
#include "next_best_view/robot_model/RobotModel.hpp"
#include "typedef.hpp"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Path.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace next_best_view {
	/*!
     * \brief PTURobotModel implements a model of pan tilt unit robot. This is an experimental class used to test a new concept for the inverse kinematics
     * \author Florian Aumann
     * \date 10.2015
	 * \version 1.0
	 * \copyright GNU Public License
	 */
    class MILDRobotModelWithIK : public RobotModel {
	private:
		// weighting values
		float mOmegaPan;
		float mOmegaTilt;
		float mOmegaRot;
		float mOmegaBase;
		float mOmegaUseBase;
        float mOmegaGlobal;
		float speedFactorPTU;
		float speedFactorBaseMove;
		float speedFactorBaseRot;
        float viewPointDistance;
		float tolerance;
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
		
		/*!
		 * Client used for communication with the global_planner to calculate movement costs
		 */
		 nav_msgs::Path getNavigationPath(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition);
	public:
		/*!
		 * \brief constructor of the PTURobotModel
		 */
        MILDRobotModelWithIK();

		/*!
		 * \brief destructor of the MILDRobotModel
		 */
        virtual ~MILDRobotModelWithIK();

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

        /*!
         * \brief sets distance bewtween the center of the view frustum and the camera
         * \param viewPointDistance distance bewtween the center of the view frustum and the camera in milimeters
         */
        void setViewPointDistance(float viewPointDistance);

		bool isPoseReachable(const SimpleVector3 &position, const SimpleQuaternion &orientation);
		
		bool isPositionReachable(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition);

		RobotStatePtr calculateRobotState(const RobotStatePtr &sourceRobotState, const SimpleVector3 &position, const SimpleQuaternion &orientation);
		
		/*!
		 * \brief Uses a given RobotState to calculate the camera frame
		 */
		geometry_msgs::Pose calculateCameraPose(const RobotStatePtr &sourceRobotState);

		/*!
		 * \brief Calculates the movement costs from sourceRobotState to targetRobotState. Returns -1 if pose is not reachable
		 */
        float getMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState);

        float getPTU_TiltMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState);

        float getPTU_PanMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState);

        float getRotationCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState);
		
		float getDistance(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition);

        //Returns the current pose of the robots base from tf
        geometry_msgs::Pose getRobotPose();
        //Returns the current pose of the robots camera from tf
        geometry_msgs::Pose getCameraPose();

	};

    typedef boost::shared_ptr<MILDRobotModelWithIK> MILDRobotModelWithIKPtr;
}


#endif /* MILDROBOTMODEL_HPP_ */
