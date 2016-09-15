/**

Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Heller Florian, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Stroh Daniel, Trautmann Jeremias, Walter Milena, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

4. The use is explicitly not permitted to any application which deliberately try to kill or do harm to any living creature.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef ROBOTMODEL_HPP_
#define ROBOTMODEL_HPP_

#include "next_best_view/common/CommonClass.hpp"
#include "next_best_view/robot_model/RobotState.hpp"
#include "typedef.hpp"
#include "nav_msgs/Path.h"

namespace next_best_view {

    // TODO costs -> inverse costs
	/*!
	 * \brief RobotModel generalizes a robot model.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
	class RobotModel : public CommonClass {
	private:
		RobotStatePtr mCurrentRobotState;
	public:
		/*!
		 * \brief the constructor of the RobotModel object
		 */
		RobotModel();

		/*!
		 * \brief the destructor of the RobotModel object
		 */
		virtual ~RobotModel();

		/*!
		 * \param orientation the orientation to reach
		 * \return if the orienation is reachable or not
		 */
		virtual bool isPoseReachable(const SimpleVector3 &position, const SimpleQuaternion &orientation) = 0;

        virtual bool isPositionReachable(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition) = 0;

        /*!
         * Client used for communication with the global_planner to calculate movement costs
         */
         virtual nav_msgs::Path getNavigationPath(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition, double sourceRotationBase, double targetRotationBase) = 0;
         virtual nav_msgs::Path getNavigationPath(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition) = 0;

		/*!
		 * \brief calculates the target robot state by assuming the saved current state of the roboter as source state
		 * \param position the position
		 * \param orientation the orientation
		 * \return the new robot state
		 */
		RobotStatePtr calculateRobotState(const SimpleVector3 &position, const SimpleQuaternion &orientation);

		/*!
		 * \brief calculates the target robot state
		 * \param currentRobotState the current robot state
		 * \param position the position
		 * \param orientation the orientation
		 * \return the new robot state
		 */
		virtual RobotStatePtr calculateRobotState(const RobotStatePtr &sourceRobotState, const SimpleVector3 &position, const SimpleQuaternion &orientation) = 0;

		/*!
		 * \param targetRobotState the target state
		 * \return the rating
		 */
        float getBase_TranslationalMovementCosts(const RobotStatePtr &targetRobotState);

		/*!
		 * \param sourceRobotState the source state
		 * \param targetRobotState the target state
		 * \return the rating to move to the pose.
		 */
        virtual float getBase_TranslationalMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState) = 0;

        float getPTU_TiltMovementCosts(const RobotStatePtr &targetRobotState);

        virtual float getPTU_TiltMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState) = 0;

        float getPTU_PanMovementCosts(const RobotStatePtr &targetRobotState);

        virtual float getPTU_PanMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState) = 0;

        float getBase_RotationalMovementCosts(const RobotStatePtr &targetRobotState);

        virtual float getBase_RotationalMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState) = 0;

        virtual float getDistance(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition) = 0;

        virtual geometry_msgs::Pose getRobotPose() = 0;

        virtual geometry_msgs::Pose getCameraPose() = 0;

        /*!
         * \brief Uses a given RobotState to calculate the camera frame
         */
        virtual geometry_msgs::Pose calculateCameraPose(const RobotStatePtr &sourceRobotState) = 0;

        /*!
         * \param currentRobotState sets the current robot state ptr
         */
		void setCurrentRobotState(const RobotStatePtr &currentRobotState);

		/*!
		 * \return the current robot state ptr
		 */
		RobotStatePtr getCurrentRobotState();
	};

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<RobotModel> RobotModelPtr;
}

#endif /* ROBOTMODEL_HPP_ */
