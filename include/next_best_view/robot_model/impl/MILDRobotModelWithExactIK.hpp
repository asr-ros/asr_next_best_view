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
#include "next_best_view/robot_model/impl/MILDRobotModel.hpp"
#include "next_best_view/robot_model/impl/MILDRobotState.hpp"
#include "typedef.hpp"
#include "geometry_msgs/Pose.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "next_best_view/rating/IKRatingModule.h"
#include "next_best_view/helper/DebugHelper.hpp"

namespace next_best_view {
	/*!
     * \brief MILDRobotModelWithExactIK implements a model of pan tilt unit robot where the inverse kinematic is calculated exactly.
     * \author Florian Aumann
     * \date 10.2015
	 * \version 1.0
	 * \copyright GNU Public License
	 */
    typedef std::tuple<double,double> PTUConfig;    //Pan and tilt angle of the ptu
    class MILDRobotModelWithExactIK : public MILDRobotModel {
	private:

         //Maximum number of iterations for the IK angle iteration method
         const unsigned int IKVisualizationMaximunIterationCount = 6;

         float mInverseKinematicIterationAccuracy;

         /*!
          * The rating module for the inverse kinematic sampling
          */
         IKRatingModulePtr ikRatingModule;

         /*!
          * Height of the tilt axis above ground
          */
         double h_tilt;
         double viewTriangleZPlane_angleAlpha;
         double viewTriangleZPlane_angleGamma;
         double viewTriangleZPlane_sideA;
         double viewTriangleZPlane_sideB;
         double viewTriangleXYPlane_sideC;
         unsigned int mPanAngleSamplingStepsPerIteration;
         unsigned int mIKVisualizationLastIterationCount = 0;
         double mTiltAngleOffset;
         double mPanAngleOffset;
         double mViewPointDistance;

         /*!
          * Detemines if the IK calculation should be visualized by markers. Turn this off to speed up calculation
          */
         bool mVisualizeIK;

         /*!
          * Transformation frame from the tilted-link to camera left
          */
         Eigen::Affine3d tiltToCameraEigen;

         /*!
          * Transformation frame from the pan-link to the tilt-link
          */
         Eigen::Affine3d panToTiltEigen;
         Eigen::Affine3d tiltToPanEigen;

         /*!
          * Transformation frame from the base-link to the pan-link
          */
         Eigen::Affine3d baseToPanEigen;
         Eigen::Affine3d panToBaseEigen;

         double x_product;

         /*!
          * Flag, shows if the tf parameters have already been initialized
          */
         bool tfParametersInitialized;

         //For Tests only
         int mNumberIKCalls;
         double mnTotalIKTime;

         /*!
          * Trys to calculate parameters needed for the inverse kinematic using tf transformations
          */
         bool setUpTFParameters();
         /*!
          * Helper: Calculates the acos of the value. If value is not within [-1, 1], clamps the value accordingly
          */
         double acosClamped(double value);
         /*!
          * Calculates the rotation angle of the base from its pose frame
          */
         double getBaseAngleFromBaseFrame(Eigen::Affine3d &baseFrame);
         /*!
          * Calculates the optimal pan angle for the given pose of the pan joint
          */
         double getPanAngleFromPanJointPose(Eigen::Affine3d &panJointFrame, MILDRobotStatePtr &robotState);
         /*!
          * Calculates the tilt angle and the projected pose of the tilt joint
          * returns true if successful
          */
         bool getTiltAngleAndTiltBasePointProjected(Eigen::Vector3d &planeNormal, Eigen::Vector3d &targetViewVector,  Eigen::Vector3d &target_view_center_point, double &tilt, Eigen::Vector3d &tilt_base_point_projected);
         /*!
          * Calculates the transformation frame of the tilt joint
          */
         Eigen::Affine3d getTiltJointFrame(Eigen::Vector3d &planeNormal, Eigen::Vector3d &targetViewVector,  Eigen::Vector3d &tilt_base_point);
         /*!
          * Visualizes the output of the IK calculation
          */
         void visualizeIKcalculation(Eigen::Vector3d &base_point, Eigen::Vector3d &base_orientation, Eigen::Vector3d &pan_joint_point, Eigen::Vector3d & pan_rotated_point, Eigen::Vector3d &tilt_base_point, Eigen::Vector3d &tilt_base_point_projected, Eigen::Vector3d &cam_point, Eigen::Vector3d &actual_view_center_point);
         /*!
          * Visualizes the output of the CameraPoseCorrection
          */
         void visualizeCameraPoseCorrection(Eigen::Vector3d &base_point, Eigen::Vector3d &base_orientation, Eigen::Vector3d &pan_joint_point, Eigen::Vector3d & pan_rotated_point, Eigen::Vector3d &tilt_base_point, Eigen::Vector3d &cam_point, Eigen::Vector3d &actual_view_center_point);
         /*!
          * Visualizes the IK camera target
          */
         void visualizeIKCameraTarget(Eigen::Vector3d &target_view_center_point, Eigen::Vector3d &target_camera_point);
         /*!
          * Deletes all visualization markers for the IK calculation
          */
         void resetIKVisualization();
         /*!
          * Visualizes a point for a single frame position
          */
         void visualizeIKPoint(Eigen::Vector3d &point, Eigen::Vector4d &colorRGBA, std::string ns, int id);
         /*!
          * Visualizes the translation between two frames through an arrow
          */
         void visualizeIKArrowLarge(Eigen::Vector3d &pointStart, Eigen::Vector3d &pointEnd, Eigen::Vector4d &colorRGBA, std::string ns, int id);
         /*!
          * Visualizes the translation between two frames through an arrow
          */
         void visualizeIKArrowSmall(Eigen::Vector3d &pointStart, Eigen::Vector3d &pointEnd, Eigen::Vector4d &colorRGBA, std::string ns, int id);
         /*!
          * Visualizes the translation between two frames through an arrow
          */
         void visualizeIKArrow(Eigen::Vector3d &pointStart, Eigen::Vector3d &pointEnd, Eigen::Vector4d &colorRGBA, std::string ns, Eigen::Vector3d &scaleParameters, int id);

    public:

		/*!
         * \brief constructor of the MILDRobotModelWithExactIK
		 */
        MILDRobotModelWithExactIK();

		/*!
         * \brief destructor of the MILDRobotModelWithExactIK
		 */
        virtual ~MILDRobotModelWithExactIK();


        /*!
         * \brief Calculates the inverse kinematics for the given camera pose
         */
		RobotStatePtr calculateRobotState(const RobotStatePtr &sourceRobotState, const SimpleVector3 &position, const SimpleQuaternion &orientation);

        /*!
         * \brief Calculates the best approximation for the given camera pose from the current base position
         */
        PTUConfig calculateCameraPoseCorrection(const RobotStatePtr &sourceRobotState, const SimpleVector3 &position, const SimpleQuaternion &orientation);

        /*!
         * \brief Sets the distance between the desired view center point and the camera
         */
        void setViewPointDistance(float viewPointDistance);

        ros::Publisher vis_pub;
	};

    typedef boost::shared_ptr<MILDRobotModelWithExactIK> MILDRobotModelWithExactIKPtr;
}
