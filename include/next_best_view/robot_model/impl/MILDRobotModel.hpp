/**

Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Braun Kai, Heller Florian, Hutmacher Robin, Karrenbauer Oliver, Marek Felix, Mayr Matthias, Mehlhaus Jonas, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Stroh Daniel, Trautmann Jeremias, Walter Milena
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

4. The use is explicitly not permitted to any application which deliberately try to kill or do harm to any living creature.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

#include <boost/tuple/tuple.hpp>
#include "next_best_view/robot_model/RobotModel.hpp"
#include "typedef.hpp"
#include "geometry_msgs/Pose.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "next_best_view/helper/DebugHelper.hpp"
#include "next_best_view/helper/MapHelper.hpp"

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
        MapHelperPtr mMapHelperPtr;

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

        bool isPositionAllowed(const geometry_msgs::Point &position);

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
        geometry_msgs::Pose calculateCameraPose(const RobotStatePtr &sourceRobotState);

    private:
        /*!
         * \brief Init MapHelper if it is not initialized yet.
         */
        void initMapHelper();
	};

    typedef boost::shared_ptr<MILDRobotModel> MILDRobotModelPtr;
}
