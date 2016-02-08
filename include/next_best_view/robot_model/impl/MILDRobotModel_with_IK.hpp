/*
 * PTURoboterState.hpp
 *
 *  Created on: Sep 23, 2014
 *      Author: ralfschleicher
 */

#ifndef MILDROBOTMODELWITHIK_HPP_
#define MILDROBOTMODELWITHIK_HPP_

#include <boost/tuple/tuple.hpp>
#include "next_best_view/robot_model/RobotModel.hpp"
#include "next_best_view/robot_model/impl/MILDRobotState.hpp"
#include "typedef.hpp"
#include "geometry_msgs/Pose.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "next_best_view/rating/impl/DefaultIKRatingModule.h"

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
        float mViewPointDistance;
		float tolerance;
        float mInverseKinematicIterationAccuracy;

		bool useGlobalPlanner;
        tf::TransformListener *listener;

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
          * The rating module for the inverse kinematic sampling
          */
         IKRatingModulePtr ikRatingModule;

         /*!
          * Height of the tilt axis above ground
          */
         double h_tilt;
         double viewTriangle_angleAlpha;
         double viewTriangle_angleGamma;
         double viewTriangle_sideA;
         unsigned int mPanAngleSamplingStepsPerIteration;
         unsigned int mIKVisualizationLastMarkerCount = 0;
         double mTiltAngleOffset;

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

         /*!
          * Trys to calculate parameters needed for the inverse kinematic using tf transformations
          */
         bool setUpTFParameters();
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
		 * \brief constructor of the PTURobotModel
		 */
        MILDRobotModelWithIK();

		/*!
		 * \brief destructor of the MILDRobotModel
		 */
        virtual ~MILDRobotModelWithIK();

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
        float getBase_TranslationalMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState);

        float getPTU_TiltMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState);

        float getPTU_PanMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState);

        float getBase_RotationalMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState);
		
		float getDistance(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition);

        //Returns the current pose of the robots base from tf
        geometry_msgs::Pose getRobotPose();
        //Returns the current pose of the robots camera from tf
        geometry_msgs::Pose getCameraPose();

        ros::Publisher vis_pub;
	};

    typedef boost::shared_ptr<MILDRobotModelWithIK> MILDRobotModelWithIKPtr;
}


#endif /* MILDROBOTMODELWITHIK_HPP_ */
