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

#include <glpk.h>
#include <limits>
#include <ros/ros.h>
#include <ros/node_handle.h>

#include "next_best_view/robot_model/impl/MILDRobotModelWithExactIK.hpp"
#include "next_best_view/robot_model/impl/MILDRobotModel.hpp"
#include "next_best_view/helper/MathHelper.hpp"
#include "next_best_view/rating/impl/AngleApproximationIKRatingModule.h"
#include "next_best_view/rating/impl/NavigationPathIKRatingModule.h"
#include "next_best_view/rating/impl/SimpleIKRatingModule.h"

#include "nav_msgs/GetPlan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"

#include "urdf/model.h"
#include "urdf_model/joint.h"
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
//#include <robot_state_publisher/robot_state_publisher.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <tf_conversions/tf_eigen.h>

#include <locale>

#include <visualization_msgs/Marker.h>

namespace next_best_view {
    MILDRobotModelWithExactIK::MILDRobotModelWithExactIK() : MILDRobotModel() {
        mDebugHelperPtr = DebugHelper::getInstance();
        mDebugHelperPtr->write(std::stringstream() << "STARTING MILD ROBOT MODEL WITH EXACT IK", DebugHelper::ROBOT_MODEL);

        ros::NodeHandle n("nbv_robot_model");
        navigationCostClient = n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
        double inverseKinematicIterationAccuracy_, ncp_, fcp_;
        int panAngleSamplingStepsPerIteration_;
        bool visualizeIK_;
        std::string IKAngleRating_;
        n.getParam("panAngleSamplingStepsPerIteration", panAngleSamplingStepsPerIteration_);
        n.getParam("inverseKinematicIterationAccuracy", inverseKinematicIterationAccuracy_);
        n.getParam("visualizeIK", visualizeIK_);
        n.getParam("ncp", ncp_);
        n.getParam("fcp", fcp_);
        if (visualizeIK_)
        {
            mDebugHelperPtr->write("IK Visualization ENABLED", DebugHelper::PARAMETERS);
        }
        else
        {
            mDebugHelperPtr->write("IK Visualization DISABLED", DebugHelper::PARAMETERS);
        }
        mPanAngleSamplingStepsPerIteration = panAngleSamplingStepsPerIteration_;
        mViewPointDistance = (ncp_ + fcp_)/2.0;
        mInverseKinematicIterationAccuracy = inverseKinematicIterationAccuracy_;
        mVisualizeIK = visualizeIK_;
        mDebugHelperPtr->write(std::stringstream() << "mPanAngleSamplingStepsPerIteration: " << mPanAngleSamplingStepsPerIteration, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "mInverseKinematicIterationAccuracy: " << mInverseKinematicIterationAccuracy, DebugHelper::PARAMETERS);
        mDebugHelperPtr->write(std::stringstream() << "mViewPointDistance: " << mViewPointDistance, DebugHelper::PARAMETERS);
        //Temporary Visualization Publisher
        std::string IKVisualization;
        n.getParam("IKVisualization", IKVisualization);
        vis_pub = n.advertise<visualization_msgs::Marker>(IKVisualization, 1000);
        tfParametersInitialized = setUpTFParameters();
        n.getParam("IKAngleRating", IKAngleRating_);
        //IKAngleRating_ = std::toupper(IKAngleRating_);
        if (IKAngleRating_ == "ANGLE_APPROXIMATION")
        {
            ikRatingModule = AngleApproximationIKRatingModulePtr(new AngleApproximationIKRatingModule());
            mDebugHelperPtr->write(std::stringstream() << "Using AngleApproximation as IK angle rating algorithm", DebugHelper::PARAMETERS);
        }
        else if (IKAngleRating_ == "NAVIGATION_PATH")
        {
            ikRatingModule = NavigationPathIKRatingModulePtr(new NavigationPathIKRatingModule(RobotModelPtr(this)));
            mDebugHelperPtr->write(std::stringstream() << "Using NavigationPath as IK angle rating algorithm", DebugHelper::PARAMETERS);
        }
        else if (IKAngleRating_ == "SIMPLE")
        {
            ikRatingModule = SimpleIKRatingModulePtr(new SimpleIKRatingModule());
            mDebugHelperPtr->write(std::stringstream() << "Using SimpleIKRating as IK angle rating algorithm", DebugHelper::PARAMETERS);
        }
        else
        {
            ikRatingModule = SimpleIKRatingModulePtr(new SimpleIKRatingModule());
            ROS_WARN_STREAM("'" << IKAngleRating_ << "' is not a valid IK rating algorithm! Using the SimpleIKRating algorithm instead.");
        }
        mNumberIKCalls = 0;
        mnTotalIKTime = 0.0;
        mIKVisualizationLastIterationCount = IKVisualizationMaximunIterationCount;
	}

    MILDRobotModelWithExactIK::~MILDRobotModelWithExactIK() {}

    void MILDRobotModelWithExactIK::setViewPointDistance(float viewPointDistance) {
        this->mViewPointDistance = viewPointDistance;
        mDebugHelperPtr->write(std::stringstream() << "viewPointDistance set to: " << viewPointDistance, DebugHelper::PARAMETERS);

    }


    PTUConfig MILDRobotModelWithExactIK::calculateCameraPoseCorrection(const RobotStatePtr &sourceRobotState, const SimpleVector3 &position, const SimpleQuaternion &orientation)
    {
        MILDRobotStatePtr sourceMILDRobotState = boost::static_pointer_cast<MILDRobotState>(sourceRobotState);
        //Make sure the necessary geometry parameters are initialized
        mDebugHelperPtr->writeNoticeably("STARTING CALCULATE-CAMERA-POSE-CORRECTION METHOD", DebugHelper::ROBOT_MODEL);
        if (!tfParametersInitialized)
        {
            tfParametersInitialized = setUpTFParameters();
            if (!tfParametersInitialized)
            {
                ROS_ERROR_STREAM("Could not extract parameters from tf.");
                mDebugHelperPtr->writeNoticeably("ENDING CALCULATE-CAMERA-POSE-CORRECTION METHOD", DebugHelper::ROBOT_MODEL);
                return PTUConfig(0.0,0.0);
            }
        }
        Eigen::Vector3d basePosition(sourceMILDRobotState->x, sourceMILDRobotState->y, 0.0);
        //Calculate pose of the pan joint
        Eigen::Affine3d basePoseEigen = Eigen::Translation3d(basePosition) * Eigen::AngleAxisd((double)(sourceMILDRobotState->rotation), Eigen::Vector3d::UnitZ());
        Eigen::Affine3d panJointEigen = basePoseEigen * baseToPanEigen;
        //Calculate the target center point
        Eigen::Quaterniond targetOrientation(orientation.w(), orientation.x(), orientation.y(), orientation.z());
        Eigen::Vector3d targetViewVector = targetOrientation.toRotationMatrix() * Eigen::Vector3d::UnitX();
        Eigen::Vector3d target_view_center_point = Eigen::Vector3d(position[0], position[1], position[2]) + targetViewVector*mViewPointDistance;
        //Visualize target view
        if (vis_pub.getNumSubscribers() > 0)
        {
            Eigen::Vector3d target_cam_point(position[0], position[1], position[2]);
            visualizeIKCameraTarget(target_view_center_point, target_cam_point);
        }
        //Calculate PAN
        Eigen::Vector3d panJointToCenterPointProjected(target_view_center_point[0] - panJointEigen(0,3), target_view_center_point[1] - panJointEigen(1,3), 0.0);
        double viewTriangleXYPlane_sideA = panJointToCenterPointProjected.norm();
        double viewTriangleXYPlane_sideB = sqrt(pow(viewTriangleXYPlane_sideA, 2.0) - pow(viewTriangleXYPlane_sideC, 2.0));
        double viewTriangleXYPlane_AngleBeta = pow(viewTriangleXYPlane_sideA, 2.0) + pow(viewTriangleXYPlane_sideC, 2.0) - pow(viewTriangleXYPlane_sideB, 2.0);
        viewTriangleXYPlane_AngleBeta = viewTriangleXYPlane_AngleBeta/(2.0*viewTriangleXYPlane_sideA*viewTriangleXYPlane_sideC);
        viewTriangleXYPlane_AngleBeta = acos(viewTriangleXYPlane_AngleBeta);
        Eigen::Vector3d panJointXAxis(panJointEigen(0,0), panJointEigen(1,0), panJointEigen(2,0));
        Eigen::Vector3d panJointYAxis(panJointEigen(0,1), panJointEigen(1,1), panJointEigen(2,1));
        panJointXAxis.normalize();
        panJointYAxis.normalize();
        panJointToCenterPointProjected.normalize();
        double panAngle = viewTriangleXYPlane_AngleBeta - (M_PI/2.0 - asin(panJointToCenterPointProjected.dot(panJointYAxis)));
        //panAngle += viewTriangleXYPlane_AngleBeta; //mPanAngleOffset;
        mDebugHelperPtr->write(std::stringstream() << "viewTriangleXYPlane_sideA: " << viewTriangleXYPlane_sideA, DebugHelper::ROBOT_MODEL);
        mDebugHelperPtr->write(std::stringstream() << "viewTriangleXYPlane_sideB: " << viewTriangleXYPlane_sideB, DebugHelper::ROBOT_MODEL);
        mDebugHelperPtr->write(std::stringstream() << "viewTriangleXYPlane_sideC: " << viewTriangleXYPlane_sideC, DebugHelper::ROBOT_MODEL);
        mDebugHelperPtr->write(std::stringstream() << "viewTriangleXYPlane_AngleBeta: " << viewTriangleXYPlane_AngleBeta, DebugHelper::ROBOT_MODEL);
        mDebugHelperPtr->write(std::stringstream() << "asin(panJointToCenterPointProjected.dot(panJointYAxis)): " << asin(panJointToCenterPointProjected.dot(panJointYAxis)),
                    DebugHelper::ROBOT_MODEL);
        mDebugHelperPtr->write(std::stringstream() << "mPanAngleOffset: " << mPanAngleOffset, DebugHelper::ROBOT_MODEL);
        mDebugHelperPtr->write(std::stringstream() << "panJointToCenterPointProjected.dot(panJointXAxis): " << panJointToCenterPointProjected.dot(panJointXAxis),
                    DebugHelper::ROBOT_MODEL);
        //Calculate TILT
        Eigen::Affine3d panJointRotatedEigen = panJointEigen * Eigen::AngleAxisd(panAngle, Eigen::Vector3d::UnitZ());
        Eigen::Affine3d tiltJointEigen = panJointRotatedEigen * panToTiltEigen;
        Eigen::Vector3d tiltJointToViewCenter = target_view_center_point - Eigen::Vector3d(tiltJointEigen(0,3), tiltJointEigen(1,3), tiltJointEigen(2,3));
        Eigen::Vector3d tiltJointYAxis(tiltJointEigen(0,1), tiltJointEigen(1,1), tiltJointEigen(2,1));
        tiltJointYAxis.normalize();
        Eigen::Vector3d tiltJointToViewCenterProjected = tiltJointToViewCenter - tiltJointYAxis.dot(tiltJointToViewCenter) * tiltJointYAxis;
        double viewTriangleZPlane_sideA = tiltJointToViewCenterProjected.norm();
        double bTimesCos = viewTriangleZPlane_sideB*cos(viewTriangleZPlane_angleAlpha);
        double viewTriangleZPlane_sideC = - bTimesCos/2.0 + sqrt(pow(bTimesCos,2.0)/4.0-pow(viewTriangleZPlane_sideB,2.0)+pow(viewTriangleZPlane_sideA,2.0));

        mDebugHelperPtr->write(std::stringstream() << "viewTriangleZPlane_sideA: " << viewTriangleZPlane_sideA, DebugHelper::ROBOT_MODEL);
        mDebugHelperPtr->write(std::stringstream() << "viewTriangleZPlane_sideB: " << viewTriangleZPlane_sideB, DebugHelper::ROBOT_MODEL);
        mDebugHelperPtr->write(std::stringstream() << "viewTriangleZPlane_sideC: " << viewTriangleZPlane_sideC, DebugHelper::ROBOT_MODEL);

        double viewTriangleZPlane_angleGamma = pow(viewTriangleZPlane_sideB, 2.0) + pow(viewTriangleZPlane_sideA, 2.0) - pow(viewTriangleZPlane_sideC, 2.0);
        viewTriangleZPlane_angleGamma = acos(viewTriangleZPlane_angleGamma / (2.0*viewTriangleZPlane_sideB*viewTriangleZPlane_sideA));
        double tiltAngle = viewTriangleZPlane_angleGamma - acos(tiltJointToViewCenter[2]/tiltJointToViewCenter.norm());

        //Check angle angle constrains and truncate values if neccessaire
        double tiltMin = mTiltLimits.get<0>();
        double tiltMax = mTiltLimits.get<1>();
        if (tiltAngle < tiltMin)
        {
            ROS_WARN_STREAM("Calculated Tilt-Angle was too small: " << tiltAngle*(180.0/M_PI));
            tiltAngle = tiltMin;
        }
        if (tiltAngle > tiltMax)
        {
            ROS_WARN_STREAM("Calculated Tilt-Angle was too high: " << tiltAngle*(180.0/M_PI));
            tiltAngle = tiltMax;
        }

        double panMin = mPanLimits.get<0>();
        double panMax = mPanLimits.get<1>();
        if (panAngle < panMin)
        {
            ROS_WARN_STREAM("Calculated Pan-Angle was too small: " << panAngle*(180.0/M_PI));
            panAngle = panMin;
        }
        if (panAngle > panMax)
        {
            ROS_WARN_STREAM("Calculated Pan-Angle was too high: " << panAngle*(180.0/M_PI));
            panAngle = panMax;
        }

        //visualize current robot configuration and corrected view target
        if (vis_pub.getNumSubscribers() > 0)
        {
            Eigen::Vector3d baseOrientation(basePoseEigen(0,0), basePoseEigen(1,0), basePoseEigen(2,0));
            Eigen::Vector3d pan_base_point(panJointEigen(0,3), panJointEigen(1,3), panJointEigen(2,3));
            Eigen::Vector3d pan_rotated_point(panJointRotatedEigen(0,3), panJointRotatedEigen(1,3), panJointRotatedEigen(2,3));
            Eigen::Vector3d tilt_base_point(tiltJointEigen(0,3), tiltJointEigen(1,3), tiltJointEigen(2,3));
            Eigen::Affine3d camFrame = tiltJointEigen * Eigen::AngleAxisd(-tiltAngle, Eigen::Vector3d::UnitY()) * tiltToCameraEigen;
            Eigen::Vector3d cam_point(camFrame(0,3), camFrame(1,3), camFrame(2,3));
            Eigen::Affine3d actualViewCenterEigen = camFrame * Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, viewTriangleZPlane_sideA));
            Eigen::Vector3d actual_view_center_point(actualViewCenterEigen(0,3), actualViewCenterEigen(1,3), actualViewCenterEigen(2,3));
            visualizeCameraPoseCorrection(basePosition, baseOrientation, pan_base_point, pan_rotated_point, tilt_base_point,cam_point, actual_view_center_point);
        }

        mDebugHelperPtr->writeNoticeably("ENDING CALCULATE-CAMERA-POSE-CORRECTION METHOD", DebugHelper::ROBOT_MODEL);
        return PTUConfig(panAngle,tiltAngle);
    }


    //Solves the inverse kinematical problem for an given robot state and a pose for the camera
    RobotStatePtr MILDRobotModelWithExactIK::calculateRobotState(const RobotStatePtr &sourceRobotState, const SimpleVector3 &position, const SimpleQuaternion &orientation)
    {
        mDebugHelperPtr->writeNoticeably("STARTING CALCULATE-ROBOT-STATE METHOD", DebugHelper::ROBOT_MODEL);
        std::clock_t begin = std::clock();

        MILDRobotStatePtr sourceMILDRobotState = boost::static_pointer_cast<MILDRobotState>(sourceRobotState);
        MILDRobotStatePtr targetMILDRobotState(new MILDRobotState());
        targetMILDRobotState->pan = 0;
        targetMILDRobotState->tilt = 0;
        targetMILDRobotState->rotation = 0;
        targetMILDRobotState->x = 0;
        targetMILDRobotState->y = 0;

        double tiltMin = mTiltLimits.get<0>();
        double tiltMax = mTiltLimits.get<1>();

        if (mVisualizeIK && vis_pub.getNumSubscribers() > 0) {this->resetIKVisualization();}

        //Make sure the necessary geometry parameters are initialized
        if (!tfParametersInitialized)
        {
            tfParametersInitialized = setUpTFParameters();
            if (!tfParametersInitialized)
            {
                ROS_ERROR_STREAM("Could not extract parameters from tf.");
                mDebugHelperPtr->writeNoticeably("ENDING CALCULATE-ROBOT-STATE METHOD", DebugHelper::ROBOT_MODEL);
                return targetMILDRobotState;
            }
        }
        Eigen::Quaterniond targetOrientation(orientation.w(), orientation.x(), orientation.y(), orientation.z());
        Eigen::Vector3d targetViewVector = targetOrientation.toRotationMatrix() * Eigen::Vector3d::UnitX();

        //The plane normal is now defined as the negative cross product of the camera view vector and the z axis.
        //Any rotation around the view axis will be ignored to ensure that the resulting target camera pose is valid
        Eigen::Vector3d planeNormal = -targetViewVector.cross(Eigen::Vector3d::UnitZ());
        planeNormal.normalize(); targetViewVector.normalize();

        mDebugHelperPtr->write(std::stringstream() << "Source state: (Pan: " << sourceMILDRobotState->pan
                                << ", Tilt: " << sourceMILDRobotState->tilt
                                << ", Rotation " << sourceMILDRobotState->rotation
                                << ", X:" << sourceMILDRobotState->x
                                << ", Y:" << sourceMILDRobotState->y << ")",
                    DebugHelper::ROBOT_MODEL);
        mDebugHelperPtr->write(std::stringstream() << "Target Position: " << position[0] << ", " << position[1] << ", " << position[2],
                    DebugHelper::ROBOT_MODEL);
        mDebugHelperPtr->write(std::stringstream() << "Target Orientation: " << targetOrientation.w() << ", " << targetOrientation.x() << ", " << targetOrientation.y()<< ", " << targetOrientation.z(),
                    DebugHelper::ROBOT_MODEL);

        //Visualize target camera pose & target viewcenter
        Eigen::Vector3d target_view_center_point = Eigen::Vector3d(position[0], position[1], position[2]) + targetViewVector*mViewPointDistance;
        if (mVisualizeIK && vis_pub.getNumSubscribers() > 0)
        {
            Eigen::Vector3d target_cam_point(position[0], position[1], position[2]);
            visualizeIKCameraTarget(target_view_center_point, target_cam_point);
        }

        //Calculate TILT and position of Tilt joint
        double tilt; Eigen::Vector3d tilt_base_point_projected;
        if (!getTiltAngleAndTiltBasePointProjected(planeNormal, targetViewVector, target_view_center_point, tilt, tilt_base_point_projected))
        {
             ROS_ERROR_STREAM("No solution found for center point (" << position[0] << ", " << position[1] << ", " << position[2] << ")");
             mDebugHelperPtr->writeNoticeably("ENDING CALCULATE-ROBOT-STATE METHOD", DebugHelper::ROBOT_MODEL);
             return targetMILDRobotState;
        }
        mDebugHelperPtr->write(std::stringstream() << "tilt_base_point_projected: " << tilt_base_point_projected[0] << ", " << tilt_base_point_projected[1] << ", " << tilt_base_point_projected[2],
                    DebugHelper::ROBOT_MODEL);
        if (tilt < tiltMin)
        {
            ROS_WARN_STREAM("Calculated Tilt-Angle was too small: " << tilt*(180.0/M_PI));
            tilt = tiltMin;
        }
        if (tilt > tiltMax)
        {
            ROS_WARN_STREAM("Calculated Tilt-Angle was too high: " << tilt*(180.0/M_PI));
            tilt = tiltMax;
        }
        mDebugHelperPtr->write(std::stringstream() << "Tilt: " << tilt*(180.0/M_PI) << " deg",
                    DebugHelper::ROBOT_MODEL);

        Eigen::Vector3d tilt_base_point = tilt_base_point_projected + x_product*planeNormal;
        mDebugHelperPtr->write(std::stringstream() << "tilt_base_point: " << tilt_base_point[0] << ", " << tilt_base_point[1] << ", " << tilt_base_point[2],
                    DebugHelper::ROBOT_MODEL);

        // Get pose of PAN joint
        Eigen::Affine3d tiltFrame = getTiltJointFrame(planeNormal, targetViewVector,  tilt_base_point);
        Eigen::Affine3d tiltedFrame =  tiltFrame * Eigen::AngleAxisd(-tilt, Eigen::Vector3d::UnitY());
        Eigen::Affine3d camFrame = tiltedFrame * tiltToCameraEigen;
        Eigen::Affine3d actualViewCenterEigen(Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, mViewPointDistance)));
        actualViewCenterEigen = camFrame*actualViewCenterEigen;

        Eigen::Affine3d pan_rotated_Frame = tiltFrame * tiltToPanEigen;

        //Calculate PAN and base rotation
        double pan = this->getPanAngleFromPanJointPose(pan_rotated_Frame, sourceMILDRobotState);
        mDebugHelperPtr->write(std::stringstream() << "Pan: " << pan*(180.0/M_PI) << " deg", DebugHelper::ROBOT_MODEL);

        Eigen::Affine3d pan_Frame = pan_rotated_Frame * Eigen::AngleAxisd(pan, Eigen::Vector3d::UnitZ());
        Eigen::Affine3d base_Frame = pan_Frame * panToBaseEigen;

        //Visualization
        if (mVisualizeIK && vis_pub.getNumSubscribers() > 0)
        {
            Eigen::Vector3d base_point(base_Frame(0,3), base_Frame(1,3), base_Frame(2,3));
            Eigen::Vector3d pan_base_point(pan_Frame(0,3), pan_Frame(1,3), pan_Frame(2,3));
            Eigen::Vector3d pan_rotated_point(pan_rotated_Frame(0,3), pan_rotated_Frame(1,3), pan_rotated_Frame(2,3));
            Eigen::Vector3d cam_point(camFrame(0,3), camFrame(1,3), camFrame(2,3));
            Eigen::Vector3d actual_view_center_point(actualViewCenterEigen(0,3), actualViewCenterEigen(1,3), actualViewCenterEigen(2,3));
            Eigen::Vector3d base_orientation(base_Frame(0,0), base_Frame(1,0), base_Frame(2,0));
            visualizeIKcalculation(base_point, base_orientation, pan_base_point, pan_rotated_point, tilt_base_point,tilt_base_point_projected, cam_point, actual_view_center_point);
        }

        // Set output values
		// set pan
        targetMILDRobotState->pan = pan;
        //if (targetMILDRobotState->pan < -M_PI) { targetMILDRobotState->pan += 2 * M_PI; };
        //if (targetMILDRobotState->pan > M_PI) { targetMILDRobotState->pan -= 2 * M_PI; };

		// set rotation
        targetMILDRobotState->rotation = this->getBaseAngleFromBaseFrame(base_Frame);
		while (targetMILDRobotState->rotation < 0) { targetMILDRobotState->rotation += 2 * M_PI; };
		while (targetMILDRobotState->rotation > 2 * M_PI) { targetMILDRobotState->rotation -= 2 * M_PI; };

		// set tilt
        targetMILDRobotState->tilt = tilt;

		// set x, y
        targetMILDRobotState->x = base_Frame(0,3);
        targetMILDRobotState->y = base_Frame(1,3);
        mDebugHelperPtr->write(std::stringstream() << "Target state: (Pan: " << targetMILDRobotState->pan << " rad"
                                << ", Tilt: " << targetMILDRobotState->tilt << " rad"
                                << ", Rotation " << targetMILDRobotState->rotation << " rad"
                                << ", X: " << targetMILDRobotState->x << " m"
                                << ", Y: " << targetMILDRobotState->y << " m)",
                    DebugHelper::ROBOT_MODEL);


        std::clock_t end = std::clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        mNumberIKCalls++;
        mnTotalIKTime += elapsed_secs;
        mDebugHelperPtr->write(std::stringstream() << "IK Calculation took " << elapsed_secs << " seconds. Total calculation time: "
                                << mnTotalIKTime << " over " << mNumberIKCalls << " calculations.",
                    DebugHelper::ROBOT_MODEL);

        mDebugHelperPtr->writeNoticeably("ENDING CALCULATE-ROBOT-STATE METHOD", DebugHelper::ROBOT_MODEL);
        return targetMILDRobotState;
	}

    Eigen::Affine3d MILDRobotModelWithExactIK::getTiltJointFrame(Eigen::Vector3d &planeNormal, Eigen::Vector3d &targetViewVector,  Eigen::Vector3d &tilt_base_point)
    {
        Eigen::Vector3d viewDirection;
        viewDirection = targetViewVector;
        viewDirection[2] = 0.0;
        viewDirection.normalize();
        Eigen::Matrix4d tiltFrame_Rotation;
        tiltFrame_Rotation.col(0) << viewDirection[0], viewDirection[1],  0.0 , 0.0;
        tiltFrame_Rotation.col(1) << -planeNormal[0] , -planeNormal[1] ,  -planeNormal[2] ,0.0;
        tiltFrame_Rotation.col(2) << 0.0, 0.0, 1.0, 0.0;
        tiltFrame_Rotation.col(3) << tilt_base_point[0] , tilt_base_point[1] ,  tilt_base_point[2] , 1.0;

        Eigen::Affine3d tiltFrame(tiltFrame_Rotation);
        return tiltFrame;
    }

    bool MILDRobotModelWithExactIK::getTiltAngleAndTiltBasePointProjected(Eigen::Vector3d &planeNormal, Eigen::Vector3d &targetViewVector,  Eigen::Vector3d &target_view_center_point, double &tilt, Eigen::Vector3d &tilt_base_point_projected)
    {
        //Calculate tilt base point (t1, t2, t3)
        double t1, t2, t3;
        //Calculate t3 using h
        t3 = h_tilt - target_view_center_point[2];
        //Calculate t2 using abc-formula
        double a, b, c;
        double planeNormalX = planeNormal[0];

        // in case planeNormalX is zero, calculation can be done in an easier way
        if (abs(planeNormalX) < 10e-6)
        {
            //Note: I will assume here, that x and y cannot both be zero, since the plane normal has to lie in the XY-plane
            t2 = (t3*planeNormal[2])/planeNormal[1];
            t1 = sqrt(pow(viewTriangleZPlane_sideA, 2.0) - pow(t2, 2.0) - pow(t3, 2.0));
            //Note: The equation solved above has a positive and negative solution. The correct one is whichever points in the same direction as the targetViewvector
            if (targetViewVector[0]*t1+targetViewVector[1]*t2 > 0)
            {
                //-> Flip sign if needed
                t1 *= -1.0;
            }
        }
        else // in any other case, a quadratic equation needs to be solved for t2 and t1 will be derived from the result
        {
            mDebugHelperPtr->write("planNormalX NOT equal 0", DebugHelper::ROBOT_MODEL);
            a = 1 + pow(planeNormal(1)/planeNormalX, 2.0);
            b = (2*t3*planeNormal(1)*planeNormal(2))/pow(planeNormalX, 2.0);
            c = -pow(viewTriangleZPlane_sideA, 2.0) + pow(t3, 2.0)*(1+pow(planeNormal(2)/planeNormalX, 2.0));
            if (pow(b, 2.0)<4*a*c)
            {
                return false;
            }

            double t2_1, t2_2, t1_1, t1_2;
            t2_1 = (-b + sqrt(pow(b, 2.0)-4*a*c))/(2*a);
            t2_2 = (-b - sqrt(pow(b, 2.0)-4*a*c))/(2*a);
            //Calculate feasible t1
            t1_1 = -(t2_1*planeNormal(1)+t3*planeNormal(2))/planeNormalX;
            t1_2 = -(t2_2*planeNormal(1)+t3*planeNormal(2))/planeNormalX;
            //Choose t1, t2
            if (targetViewVector[0]*t1_1+targetViewVector[1]*t2_1 < 0)
            {
                t1 = t1_1;
                t2 = t2_1;
            }
            else
            {
                t1 = t1_2;
                t2 = t2_2;
            }
        }

        //get projected tilt base point
        tilt_base_point_projected = Eigen::Vector3d(t1+target_view_center_point[0], t2+target_view_center_point[1], t3+target_view_center_point[2]);
        //get tilt angle
        Eigen::Vector3d targetToBase(tilt_base_point_projected[0]-target_view_center_point[0], tilt_base_point_projected[1]-target_view_center_point[1], tilt_base_point_projected[2]-target_view_center_point[2]);
        targetToBase.normalize();
        double targetToBase_Angle = acos(targetToBase[2]);
        mDebugHelperPtr->write(std::stringstream() << "targetToBase_Angle: " << targetToBase_Angle, DebugHelper::ROBOT_MODEL);
        tilt = targetToBase_Angle+mTiltAngleOffset;
        return true;
    }



    double MILDRobotModelWithExactIK::getPanAngleFromPanJointPose(Eigen::Affine3d &panJointFrame, MILDRobotStatePtr &robotState)
    {
        unsigned int iterationCount = 1;
        double phiMin = mPanLimits.get<0>();
        double phiMax = mPanLimits.get<1>();
        double currentBestAngle = (phiMax-phiMin)/2.0+phiMin;
        double currentBestRating, newBestRating = 0.0;
        double currentAngleRange = phiMax-phiMin;
        geometry_msgs::Point actualRobotPosition, targetRobotPosition;
        actualRobotPosition.x = robotState->x;
        actualRobotPosition.y = robotState->y;
        actualRobotPosition.z = targetRobotPosition.z = 0.0;
        Eigen::Affine3d baseFrame;
        mDebugHelperPtr->write(std::stringstream() << "phiMin: " << phiMin << " phiMax: " << phiMax, DebugHelper::ROBOT_MODEL);
        mDebugHelperPtr->write(std::stringstream() << "currentAngleRange: " << currentAngleRange << " currentBestAngle: " << currentBestAngle,
                    DebugHelper::ROBOT_MODEL);
        mDebugHelperPtr->write(std::stringstream() << "mPanAngleSamplingStepsPerIteration: " << mPanAngleSamplingStepsPerIteration,
                    DebugHelper::ROBOT_MODEL);
        Eigen::Vector3d basePoint, basePoint2;
        Eigen::Vector3d baseOrientation;
        //do sampling
        do
        {
            std::ostringstream converter;
            converter << iterationCount;
            std::string nsIterationVector = std::string("iterationVector") + converter.str();
            double angleStepSize = currentAngleRange / mPanAngleSamplingStepsPerIteration;
            double currentIterationAngle, currentRating, angleCenter;
            currentBestRating = newBestRating;
            angleCenter = currentBestAngle;
            newBestRating = -1.0;
            for (unsigned int i = 0; i < mPanAngleSamplingStepsPerIteration; i++)
            {
                currentIterationAngle = angleCenter - currentAngleRange/2.0 + i * angleStepSize;
                if (phiMax>currentIterationAngle && phiMin<currentIterationAngle)
                {
                    //Calculate the base frame with respect to the current angle
                    baseFrame = panJointFrame * Eigen::AngleAxisd(-currentIterationAngle, Eigen::Vector3d::UnitZ()) * panToBaseEigen;
                    basePoint = Eigen::Vector3d(baseFrame(0,3),baseFrame(1,3),baseFrame(2,3));
                    baseOrientation = Eigen::Vector3d(baseFrame(0,0),baseFrame(1,0),baseFrame(2,0));
                    baseOrientation.normalize();
                    basePoint2 = basePoint+baseOrientation*0.15;
                    targetRobotPosition.x = baseFrame(0,3);
                    targetRobotPosition.y = baseFrame(1,3);
                    float baseAngle = getBaseAngleFromBaseFrame(baseFrame);
                    currentRating = ikRatingModule->getPanAngleRating(actualRobotPosition, targetRobotPosition, robotState->rotation, baseAngle);
                    mDebugHelperPtr->write(std::stringstream() << "PTU-Angle: " << currentIterationAngle << " with base angle: " << baseAngle << " and rating: " << currentRating,
                                DebugHelper::ROBOT_MODEL);
                    if (currentRating > newBestRating)
                    {
                        newBestRating = currentRating;
                        currentBestAngle = currentIterationAngle;
                    }
                    if (mVisualizeIK && vis_pub.getNumSubscribers() > 0)
                    {
                        Eigen::Vector4d color_succeeded(1.0-currentRating,currentRating, 0.0, 1.0);
                        visualizeIKPoint(basePoint, color_succeeded, nsIterationVector, 2*i);
                        visualizeIKArrowSmall(basePoint, basePoint2, color_succeeded, nsIterationVector, 2*i+1);
                    }
                }
            }
            mDebugHelperPtr->write(std::stringstream() << "Best angle: " << currentBestAngle << " with rating: " << newBestRating
                                    << " and angle range " << currentAngleRange,
                        DebugHelper::ROBOT_MODEL);
            currentAngleRange = currentAngleRange / 2.0;
            iterationCount++;
        //Loop while there is still significant change in the angle rating and while the maximum number of iterations has not yet been reached
        } while(fabs(currentBestRating-newBestRating) > mInverseKinematicIterationAccuracy && iterationCount <= IKVisualizationMaximunIterationCount);

        mIKVisualizationLastIterationCount = iterationCount;
        if (currentBestRating < 0.0) {ROS_ERROR_STREAM("No valid solution found for this pan frame.");}
        return -currentBestAngle;
    }


    bool MILDRobotModelWithExactIK::setUpTFParameters()
    {
        //Get Parameters from TF-Publishers
        tf::StampedTransform panToTiltTF, baseToPanTF, tiltToCamLeftTF, tiltToCamRightTF;
        Eigen::Affine3d tiltAxisPointEigen, tiltToCamLeftEigen, tiltToCamRightEigen, cameraLeftPointEigen, cameraRightPointEigen, cameraMidPointEigen;
        ROS_INFO_STREAM("Looking up tf transforms");
        ros::spinOnce();
        try
        {      
            //Wait for first transform to be published
            if (listener.waitForTransform("/map", "/ptu_mount_link", ros::Time(), ros::Duration(4.0)))
            {
                //Assume that tf is alive and lookups will be successful
                listener.lookupTransform("/ptu_pan_link", "/ptu_tilt_link", ros::Time(0), panToTiltTF);
                listener.lookupTransform("/base_link", "/ptu_pan_link", ros::Time(0), baseToPanTF);
                listener.lookupTransform("/ptu_tilted_link", "/camera_left_frame", ros::Time(0), tiltToCamLeftTF);
                listener.lookupTransform("/ptu_tilted_link", "/camera_right_frame", ros::Time(0), tiltToCamRightTF);
            }
            else
            {
                ROS_ERROR("TF lookup timed out. Is the transformation publisher running?");
                return false;
            }
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("An error occured during tf-lookup: %s",ex.what());
            return false;
        }
        tf::poseTFToEigen(panToTiltTF, panToTiltEigen);
        tf::poseTFToEigen(baseToPanTF, baseToPanEigen);
        tf::poseTFToEigen(tiltToCamLeftTF, tiltToCamLeftEigen);
        tf::poseTFToEigen(tiltToCamRightTF, tiltToCamRightEigen);

        //Quick fix: Currently, the pan angle is included pan to tilt transformation and therefore the transformation must be adjusted accordingly before using it to extract parameters
        //In the future, this problem should be approached by addign another joint to the kinematic chain to represent static and dynamic transformation frames seperately
        //ROS_INFO_STREAM("panToTiltEigen:");
        //ROS_INFO_STREAM(panToTiltEigen(0,0) << ", " << panToTiltEigen(0,1) << ", " <<  panToTiltEigen(0,2) << ", " << panToTiltEigen(0,3) << ")");
        //ROS_INFO_STREAM(panToTiltEigen(1,0) << ", " << panToTiltEigen(1,1) << ", " <<  panToTiltEigen(1,2) << ", " << panToTiltEigen(1,3) << ")");
        //ROS_INFO_STREAM(panToTiltEigen(2,0) << ", " << panToTiltEigen(2,1) << ", " <<  panToTiltEigen(2,2) << ", " << panToTiltEigen(2,3) << ")");
        //ROS_INFO_STREAM(panToTiltEigen(3,0) << ", " << panToTiltEigen(3,1) << ", " <<  panToTiltEigen(3,2) << ", " << panToTiltEigen(3,3) << ")");
        panToTiltEigen = Eigen::Affine3d::Identity();

        tiltAxisPointEigen = baseToPanEigen * panToTiltEigen;
        cameraLeftPointEigen = tiltAxisPointEigen * tiltToCamLeftEigen;
        cameraRightPointEigen = tiltAxisPointEigen * tiltToCamRightEigen;

        //Hacky-ish approach to get the center point between left and right camera
        cameraMidPointEigen = cameraLeftPointEigen;
        cameraMidPointEigen(0,3) = (cameraLeftPointEigen(0,3) + cameraRightPointEigen(0,3)) / 2.0;
        cameraMidPointEigen(1,3) = (cameraLeftPointEigen(1,3) + cameraRightPointEigen(1,3)) / 2.0;
        cameraMidPointEigen(2,3) = (cameraLeftPointEigen(2,3) + cameraRightPointEigen(2,3)) / 2.0;
        tiltToCameraEigen = tiltAxisPointEigen.inverse() * cameraMidPointEigen;

        tiltToPanEigen = panToTiltEigen.inverse();
        panToBaseEigen = baseToPanEigen.inverse();
        h_tilt = tiltAxisPointEigen.matrix()(2,3);
        ROS_INFO_STREAM("Height above ground: " << h_tilt);

        Eigen::Vector3d cam_axis_x(cameraMidPointEigen(0,0), cameraMidPointEigen(1,0), cameraMidPointEigen(2,0));
        Eigen::Vector3d cam_axis_y(cameraMidPointEigen(0,1), cameraMidPointEigen(1,1), cameraMidPointEigen(2,1));
        Eigen::Vector3d cam_axis_z(cameraMidPointEigen(0,2), cameraMidPointEigen(1,2), cameraMidPointEigen(2,2));
        cam_axis_x.normalize();
        cam_axis_y.normalize();
        cam_axis_z.normalize();
        Eigen::Vector3d tilt_to_cam(tiltAxisPointEigen(0,3)-cameraMidPointEigen(0,3), tiltAxisPointEigen(1,3)-cameraMidPointEigen(1,3), tiltAxisPointEigen(2,3)-cameraMidPointEigen(2,3));
        x_product = cam_axis_x.dot(tilt_to_cam);
        tilt_to_cam -= x_product*cam_axis_x;
        viewTriangleZPlane_sideB = tilt_to_cam.norm();
        tilt_to_cam.normalize();
        viewTriangleZPlane_angleAlpha = acos(cam_axis_z.dot(tilt_to_cam));
        viewTriangleZPlane_sideA = sqrt(pow(mViewPointDistance,2.0)+pow(viewTriangleZPlane_sideB,2.0)-2*mViewPointDistance*viewTriangleZPlane_sideB*cos(viewTriangleZPlane_angleAlpha));
        viewTriangleZPlane_angleGamma = pow(mViewPointDistance, 2.0) - pow(viewTriangleZPlane_sideA,2.0)-pow(viewTriangleZPlane_sideB,2.0);
        viewTriangleZPlane_angleGamma = viewTriangleZPlane_angleGamma / (-2.0*viewTriangleZPlane_sideA*viewTriangleZPlane_sideB);
        viewTriangleZPlane_angleGamma = acos(viewTriangleZPlane_angleGamma);
        mTiltAngleOffset = viewTriangleZPlane_angleGamma-viewTriangleZPlane_angleAlpha-M_PI/2.0;
        //Calculate additional values for the camera pose correction
        Eigen::Affine3d panToCameraEigen = panToTiltEigen * tiltToCameraEigen;
        Eigen::Vector3d cam_axis_x_pan_coordinates(panToCameraEigen(0,2), panToCameraEigen(1,2), panToCameraEigen(2,2));
        Eigen::Vector3d cam_axis_z_pan_coordinates(panToCameraEigen(0,1), panToCameraEigen(1,1), panToCameraEigen(2,1));
        cam_axis_x_pan_coordinates.normalize();
        cam_axis_z_pan_coordinates.normalize();
        Eigen::Vector3d panToCameraNormal(panToCameraEigen(0,3), panToCameraEigen(1,3), panToCameraEigen(2,3));
        panToCameraNormal = panToCameraNormal - cam_axis_x_pan_coordinates * panToCameraNormal.dot(cam_axis_x_pan_coordinates) - cam_axis_z_pan_coordinates * panToCameraNormal.dot(cam_axis_z_pan_coordinates);
        viewTriangleXYPlane_sideC = panToCameraNormal.norm();
        panToCameraNormal.normalize();
        mPanAngleOffset = acos(panToCameraNormal.dot(Eigen::Vector3d::UnitX()));

        ROS_INFO_STREAM("viewTriangleZPlane_angleAlpha: " << viewTriangleZPlane_angleAlpha);
        ROS_INFO_STREAM("viewTriangleZPlane_angleGamma: " << viewTriangleZPlane_angleGamma);
        ROS_INFO_STREAM("viewTriangleZPlane_sideA: " << viewTriangleZPlane_sideA);
        ROS_INFO_STREAM("viewTriangleZPlane_sideB: " << viewTriangleZPlane_sideB);
        ROS_INFO_STREAM("viewTriangleXYPlane_sideC: " << viewTriangleXYPlane_sideC);
        ROS_INFO_STREAM("mTiltAngleOffset: " << mTiltAngleOffset);
        ROS_INFO_STREAM("mPanAngleOffset: " << mPanAngleOffset);
        ROS_INFO_STREAM("TF lookup successful.");

        return true;
    }

    void MILDRobotModelWithExactIK::resetIKVisualization()
    {
        visualization_msgs::Marker resetMarker = visualization_msgs::Marker();
        resetMarker.id = 0;
        resetMarker.header.frame_id = "/map";
        resetMarker.action = visualization_msgs::Marker::DELETE;
        resetMarker.lifetime = ros::Duration();
        resetMarker.scale.x = 0.02;
        resetMarker.scale.y = 0.02;
        resetMarker.scale.z = 0.02;

        //Reset camToActualViewCenterVector
        resetMarker.header.stamp = ros::Time::now();
        resetMarker.type = visualization_msgs::Marker::ARROW;
        resetMarker.ns = "camToActualViewCenterVector";
        vis_pub.publish(resetMarker);
        //Reset tiltToCamVector
        resetMarker.header.stamp = ros::Time::now();
        resetMarker.type = visualization_msgs::Marker::ARROW;
        resetMarker.ns = "tiltToCamVector";
        vis_pub.publish(resetMarker);
        //Reset tiltBaseVectorProjected
        resetMarker.header.stamp = ros::Time::now();
        resetMarker.type = visualization_msgs::Marker::SPHERE;
        resetMarker.ns = "tiltBaseVectorProjected";
        vis_pub.publish(resetMarker);
        //Reset tiltBaseVector
        resetMarker.header.stamp = ros::Time::now();
        resetMarker.type = visualization_msgs::Marker::SPHERE;
        resetMarker.ns = "tiltBaseVector";
        vis_pub.publish(resetMarker);
        //Reset panToTiltVector
        resetMarker.header.stamp = ros::Time::now();
        resetMarker.type = visualization_msgs::Marker::ARROW;
        resetMarker.ns = "panToTiltVector";
        vis_pub.publish(resetMarker);
        //Reset panBaseVector
        resetMarker.header.stamp = ros::Time::now();
        resetMarker.type = visualization_msgs::Marker::SPHERE;
        resetMarker.ns = "panBaseVector";
        vis_pub.publish(resetMarker);
        //Reset baseToPanVector
        resetMarker.header.stamp = ros::Time::now();
        resetMarker.type = visualization_msgs::Marker::ARROW;
        resetMarker.ns = "baseToPanVector";
        vis_pub.publish(resetMarker);
        //Reset targetCameraVector
        resetMarker.header.stamp = ros::Time::now();
        resetMarker.type = visualization_msgs::Marker::ARROW;
        resetMarker.ns = "targetCameraVector";
        vis_pub.publish(resetMarker);
        //Reset basePose
        resetMarker.header.stamp = ros::Time::now();
        resetMarker.type = visualization_msgs::Marker::SPHERE;
        resetMarker.ns = "basePose";
        vis_pub.publish(resetMarker);
        resetMarker.type = visualization_msgs::Marker::ARROW;
        resetMarker.header.stamp = ros::Time::now();
        resetMarker.id = 1;
        vis_pub.publish(resetMarker);
        ROS_INFO_STREAM("Deleting markers for " << mIKVisualizationLastIterationCount << " iterations");
        for (unsigned int i = 1; i < mIKVisualizationLastIterationCount + 1; i++)
        {
            std::ostringstream converter;
            converter << i;
            std::string nsIterationVector = std::string("iterationVector") + converter.str();
            for (unsigned int j = 0; j < 3*(mPanAngleSamplingStepsPerIteration+1); j++)
            {
                resetMarker.ns = nsIterationVector;
                resetMarker.id = j;
                resetMarker.header.stamp = ros::Time::now();
                vis_pub.publish(resetMarker);
            }
        }
        ros::spinOnce();
        mIKVisualizationLastIterationCount = 0;
    }

    void MILDRobotModelWithExactIK::visualizeIKCameraTarget(Eigen::Vector3d &target_view_center_point, Eigen::Vector3d &target_camera_point)
    {
        Eigen::Vector4d color_green(0.0,1.0,0.0,1.0);
        visualizeIKArrowLarge(target_camera_point, target_view_center_point, color_green, "targetCameraVector", 0);
    }

    void MILDRobotModelWithExactIK::visualizeIKcalculation(Eigen::Vector3d &base_point, Eigen::Vector3d &base_orientation, Eigen::Vector3d &pan_joint_point, Eigen::Vector3d & pan_rotated_point, Eigen::Vector3d &tilt_base_point, Eigen::Vector3d &tilt_base_point_projected, Eigen::Vector3d &cam_point, Eigen::Vector3d &actual_view_center_point)
    {
        Eigen::Vector4d color_red(1.0,0.0,0.0,1.0);
        Eigen::Vector4d color_blue(0.0,0.0,1.0,1.0);
        base_orientation.normalize();
        Eigen::Vector3d base_point2 = base_point + base_orientation * 0.3;
        visualizeIKArrowLarge(cam_point, actual_view_center_point, color_blue, "camToActualViewCenterVector", 0);
        visualizeIKArrowSmall(tilt_base_point,cam_point, color_blue, "tiltToCamVector", 0);
        visualizeIKPoint(tilt_base_point_projected, color_red, "tiltBaseVectorProjected", 0);
        visualizeIKPoint(tilt_base_point, color_blue, "tiltBaseVector", 0);
        visualizeIKArrowSmall(pan_rotated_point,tilt_base_point, color_blue, "panToTiltVector", 0);
        visualizeIKPoint(pan_rotated_point, color_blue, "panBaseVector", 0);
        visualizeIKArrowSmall(base_point,pan_joint_point, color_blue, "baseToPanVector", 0);
        visualizeIKPoint(base_point, color_blue, "basePose", 0);
        visualizeIKArrowLarge(base_point, base_point2, color_blue, "basePose", 1);
    }

    void MILDRobotModelWithExactIK::visualizeCameraPoseCorrection(Eigen::Vector3d &base_point, Eigen::Vector3d &base_orientation, Eigen::Vector3d &pan_joint_point, Eigen::Vector3d & pan_rotated_point, Eigen::Vector3d &tilt_base_point, Eigen::Vector3d &cam_point, Eigen::Vector3d &actual_view_center_point)
    {
        Eigen::Vector4d color_red(1.0,0.0,0.0,1.0);
        base_orientation.normalize();
        Eigen::Vector3d base_point2 = base_point + base_orientation * 0.3;
        visualizeIKArrowLarge(cam_point, actual_view_center_point, color_red, "camToActualViewCenterVector_poseCorrection", 0);
        visualizeIKArrowSmall(tilt_base_point,cam_point, color_red, "tiltToCamVector_poseCorrection", 0);
        visualizeIKPoint(tilt_base_point, color_red, "tiltBaseVector_poseCorrection", 0);
        visualizeIKArrowSmall(pan_rotated_point,tilt_base_point, color_red, "panToTiltVector_poseCorrection", 0);
        visualizeIKPoint(pan_rotated_point, color_red, "panBaseVector_poseCorrection", 0);
        visualizeIKArrowSmall(base_point,pan_joint_point, color_red, "baseToPanVector_poseCorrection", 0);
        visualizeIKPoint(base_point, color_red, "basePose_poseCorrection", 0);
        visualizeIKArrowLarge(base_point, base_point2, color_red, "basePose_poseCorrection", 1);
    }

    void MILDRobotModelWithExactIK::visualizeIKPoint(Eigen::Vector3d &point, Eigen::Vector4d &colorRGBA, std::string ns, int id)
    {
        visualization_msgs::Marker pointMarker = visualization_msgs::Marker();
        pointMarker.header.stamp = ros::Time();
        pointMarker.header.frame_id = "/map";
        pointMarker.type = pointMarker.SPHERE;
        pointMarker.action = pointMarker.ADD;
        pointMarker.id = id;
        pointMarker.lifetime = ros::Duration();
        pointMarker.ns = ns;
        pointMarker.scale.x = 0.02;
        pointMarker.scale.y = 0.02;
        pointMarker.scale.z = 0.02;
        pointMarker.color.r = colorRGBA[0];
        pointMarker.color.g = colorRGBA[1];
        pointMarker.color.b = colorRGBA[2];
        pointMarker.color.a = colorRGBA[3];
        pointMarker.pose.position.x = point[0];
        pointMarker.pose.position.y = point[1];
        pointMarker.pose.position.z = point[2];
        vis_pub.publish(pointMarker);
    }

    void MILDRobotModelWithExactIK::visualizeIKArrowSmall(Eigen::Vector3d &pointStart, Eigen::Vector3d &pointEnd, Eigen::Vector4d &colorRGBA, std::string ns, int id)
    {
        Eigen::Vector3d scaleParameters(0.005, 0.01, 0.01);
        visualizeIKArrow(pointStart, pointEnd, colorRGBA, ns, scaleParameters, id);
    }

    void MILDRobotModelWithExactIK::visualizeIKArrowLarge(Eigen::Vector3d &pointStart, Eigen::Vector3d &pointEnd, Eigen::Vector4d &colorRGBA, std::string ns, int id)
    {
        Eigen::Vector3d scaleParameters(0.02, 0.05, 0.1);
        visualizeIKArrow(pointStart, pointEnd, colorRGBA, ns, scaleParameters, id);
    }

    void MILDRobotModelWithExactIK::visualizeIKArrow(Eigen::Vector3d &pointStart, Eigen::Vector3d &pointEnd, Eigen::Vector4d &colorRGBA, std::string ns, Eigen::Vector3d &scaleParameters, int id)
    {
        geometry_msgs::Point point1, point2;
        visualization_msgs::Marker arrowMarker = visualization_msgs::Marker();
        arrowMarker.header.stamp = ros::Time();
        arrowMarker.header.frame_id = "/map";
        arrowMarker.type = arrowMarker.ARROW;
        arrowMarker.action = arrowMarker.ADD;
        arrowMarker.id = id;
        arrowMarker.lifetime = ros::Duration();
        arrowMarker.ns = ns;
        arrowMarker.scale.x = scaleParameters[0];     //d Shaft
        arrowMarker.scale.y = scaleParameters[1];    //d Head
        arrowMarker.scale.z = scaleParameters[2];     //l Head
        arrowMarker.color.r = colorRGBA[0];
        arrowMarker.color.g = colorRGBA[1];
        arrowMarker.color.b = colorRGBA[2];
        arrowMarker.color.a = colorRGBA[3];
        point1.x = pointStart[0];
        point1.y = pointStart[1];
        point1.z = pointStart[2];
        point2.x = pointEnd[0];
        point2.y = pointEnd[1];
        point2.z = pointEnd[2];
        arrowMarker.points.push_back(point1);
        arrowMarker.points.push_back(point2);
        vis_pub.publish(arrowMarker);
    }

    double MILDRobotModelWithExactIK::getBaseAngleFromBaseFrame(Eigen::Affine3d &baseFrame)
    {
        Eigen::Vector3d xAxis(baseFrame(0,0), baseFrame(1,0), 0.0);
        Eigen::Vector3d yAxis(baseFrame(0,1), baseFrame(1,1), 0.0);
        xAxis.normalize();
        yAxis.normalize();

        if (yAxis[0] >= 0)
        {
            return acos(xAxis[0]);
        }
        else
        {
            return 2.0*M_PI - acos(xAxis[0]);
        }
    }
}

