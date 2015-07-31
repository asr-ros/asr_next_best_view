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
#include "nav_msgs/Path.h"
#include <ros/ros.h>

namespace next_best_view {
	/*!
	 * \brief PTURobotModel implements a model of pan tilt unit robot.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
	class MILDRobotModel : public RobotModel {
	private:
		// weighting values
		float mOmegaPan;
		float mOmegaTilt;
		float mOmegaRot;
		float tolerance;
		bool useGlobalPlanner;

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
		MILDRobotModel();

		/*!
		 * \brief destructor of the MILDRobotModel
		 */
		virtual ~MILDRobotModel();

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

		RobotStatePtr calculateRobotState(const RobotStatePtr &sourceRobotState, const SimpleVector3 &position, const SimpleQuaternion &orientation);

		/*!
		 * \brief Calculates the movement costs from sourceRobotState to targetRobotState. Returns -1 if pose is not reachable
		 */
		float getMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState);
	};

	typedef boost::shared_ptr<MILDRobotModel> MILDRobotModelPtr;
}


#endif /* MILDROBOTMODEL_HPP_ */
