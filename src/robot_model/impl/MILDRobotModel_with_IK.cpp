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

#include "next_best_view/robot_model/impl/MILDRobotModel_with_IK.hpp"
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
#include <tf_conversions/tf_eigen.h>

#include <visualization_msgs/Marker.h>

namespace next_best_view {
    MILDRobotModelWithIK::MILDRobotModelWithIK() : RobotModel() {
        ros::NodeHandle n("nbv_srv");
        navigationCostClient = n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
        double mOmegaPan_, mOmegaTilt_, mOmegaUseBase_, tolerance_, speedFactorPTU_,speedFactorBaseMove_,speedFactorBaseRot_,inverseKinematicIterationAccuracy_, ncp_, fcp_;
        int panAngleSamplingStepsPerIteration_;
        bool useGlobalPlanner_;
        n.getParam("mOmegaPan", mOmegaPan_);
        n.getParam("mOmegaTilt", mOmegaTilt_);
        n.getParam("mOmegaUseBase", mOmegaUseBase_);
        n.getParam("speedFactorPTU", speedFactorPTU_);
        n.getParam("speedFactorBaseMove", speedFactorBaseMove_);
        n.getParam("speedFactorBaseRot", speedFactorBaseRot_);
        n.getParam("tolerance", tolerance_);
        n.getParam("useGlobalPlanner", useGlobalPlanner_);
        n.getParam("panAngleSamplingStepsPerIteration", panAngleSamplingStepsPerIteration_);
        n.getParam("inverseKinematicIterationAccuracy", inverseKinematicIterationAccuracy_);
        n.getParam("ncp", ncp_);
        n.getParam("fcp", fcp_);
        useGlobalPlanner = useGlobalPlanner_;
        if (useGlobalPlanner_)
        {
            ROS_DEBUG("Use of global planner ENABLED");
        }
        else
        {
            ROS_DEBUG("Use of global planner DISABLED. Using simplified calculation instead...");
        }

        mOmegaPan = mOmegaPan_;
        mOmegaTilt = mOmegaTilt_;
        mOmegaUseBase = mOmegaUseBase_;
        speedFactorPTU = speedFactorPTU_;
        speedFactorBaseMove = speedFactorBaseMove_;
        speedFactorBaseRot = speedFactorBaseRot_;
        tolerance = tolerance_;
        mPanAngleSamplingStepsPerIteration = panAngleSamplingStepsPerIteration_;
        mViewPointDistance = (ncp_ + fcp_)/2.0;
        mInverseKinematicIterationAccuracy = inverseKinematicIterationAccuracy_;
        ROS_DEBUG_STREAM("mOmegaUseBase: " << mOmegaPan);
        ROS_DEBUG_STREAM("speedFactorPTU: " << speedFactorPTU);
        ROS_DEBUG_STREAM("speedFactorBaseMove: " << speedFactorBaseMove);
        ROS_DEBUG_STREAM("speedFactorBaseRot: " << speedFactorBaseRot);
        ROS_DEBUG_STREAM("tolerance: " << tolerance);
        ROS_DEBUG_STREAM("mOmegaPan: " << mOmegaPan);
        ROS_DEBUG_STREAM("mOmegaTilt: " << mOmegaTilt);
        ROS_DEBUG_STREAM("mPanAngleSamplingStepsPerIteration: " << mPanAngleSamplingStepsPerIteration);
        ROS_DEBUG_STREAM("mInverseKinematicIterationAccuracy: " << mInverseKinematicIterationAccuracy);
        ROS_DEBUG_STREAM("mViewPointDistance: " << mViewPointDistance);
		this->setPanAngleLimits(0, 0);
		this->setTiltAngleLimits(0, 0);
        this->setRotationAngleLimits(0, 0);
        listener = new tf::TransformListener();
        //Temporary Visualization Publisher
        vis_pub = n.advertise<visualization_msgs::Marker>( "/nbv/IK_Visualization", 1000);
        tfParametersInitialized = setUpTFParameters();
        ikRatingModule = DefaultIKRatingModulePtr(new DefaultIKRatingModule());
	}

    MILDRobotModelWithIK::~MILDRobotModelWithIK() {}

    geometry_msgs::Pose MILDRobotModelWithIK::getRobotPose()
    {
        geometry_msgs::Pose robotPose;
        tf::StampedTransform transform;
        listener->lookupTransform("/map", "/base_link", ros::Time(0), transform);
        robotPose.position.x = transform.getOrigin()[0];
        robotPose.position.y = transform.getOrigin()[1];
        robotPose.position.z = transform.getOrigin()[2];
        tf::quaternionTFToMsg(transform.getRotation(), robotPose.orientation);
        return robotPose;
    }

    geometry_msgs::Pose MILDRobotModelWithIK::getCameraPose()
    {
        geometry_msgs::Pose cameraPose;
        tf::StampedTransform transform;
        listener->lookupTransform("/map", "/ptu_mount_link", ros::Time(0), transform);
        cameraPose.position.x = transform.getOrigin()[0];
        cameraPose.position.y = transform.getOrigin()[1];
        cameraPose.position.z = transform.getOrigin()[2];
        tf::quaternionTFToMsg(transform.getRotation(), cameraPose.orientation);
        return cameraPose;
    }

    void MILDRobotModelWithIK::setPanAngleLimits(float minAngleDegrees, float maxAngleDegrees) {
		mPanLimits.get<0>() = MathHelper::degToRad(minAngleDegrees);
		mPanLimits.get<1>() = MathHelper::degToRad(maxAngleDegrees);
	}

    void MILDRobotModelWithIK::setTiltAngleLimits(float minAngleDegrees, float maxAngleDegrees) {
		mTiltLimits.get<0>() = MathHelper::degToRad(minAngleDegrees);
		mTiltLimits.get<1>() = MathHelper::degToRad(maxAngleDegrees);
	}

    void MILDRobotModelWithIK::setRotationAngleLimits(float minAngleDegrees, float maxAngleDegrees) {
        mRotationLimits.get<0>() = MathHelper::degToRad(minAngleDegrees);
		mRotationLimits.get<1>() = MathHelper::degToRad(maxAngleDegrees);
	}

    void MILDRobotModelWithIK::setViewPointDistance(float viewPointDistance) {
        this->mViewPointDistance = viewPointDistance;
    }

    bool MILDRobotModelWithIK::isPoseReachable(const SimpleVector3 &position, const SimpleQuaternion &orientation)
    {
        SimpleVector3 visualAxis = MathHelper::getVisualAxis(orientation);
		SimpleSphereCoordinates sphereCoords = MathHelper::convertC2S(visualAxis);

		return (mTiltLimits.get<0>() <= sphereCoords[1] && sphereCoords[1] <= mTiltLimits.get<1>());
	}

    bool MILDRobotModelWithIK::isPositionReachable(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition)
    {
        nav_msgs::Path path = getNavigationPath(sourcePosition, targetPosition);

        if (path.poses.empty())
        {
            return false;
        }
        else
        {
            int lastPose = path.poses.size()-1;
            float distanceToLastPoint = sqrt(pow(targetPosition.x - path.poses[lastPose].pose.position.x, 2) + pow(targetPosition.y  - path.poses[lastPose].pose.position.y, 2));
            ROS_DEBUG_STREAM("Target: " << targetPosition.x << ", " << targetPosition.y);
            ROS_DEBUG_STREAM("Actual position: " << path.poses[lastPose].pose.position.x << ", " << path.poses[lastPose].pose.position.y);
            return distanceToLastPoint < 0.01f;
        }
    }

    //Comment?
    //Solves the inverse kinematical problem for an given robot state and a pose for the camera
    RobotStatePtr MILDRobotModelWithIK::calculateRobotState(const RobotStatePtr &sourceRobotState, const SimpleVector3 &position, const SimpleQuaternion &orientation)
    {
        //Make sure the necessary geometry parameters are initialized
        if (!tfParametersInitialized)
        {
            tfParametersInitialized = setUpTFParameters();
        }
		MILDRobotStatePtr sourceMILDRobotState = boost::static_pointer_cast<MILDRobotState>(sourceRobotState);
		MILDRobotStatePtr targetMILDRobotState(new MILDRobotState());

        ROS_INFO_STREAM("Calculate state for: (Pan: " << sourceMILDRobotState->pan << ", Tilt: " << sourceMILDRobotState->tilt << ", Rotation " << sourceMILDRobotState->rotation << ", X:" << sourceMILDRobotState->x << ", Y:" << sourceMILDRobotState->y << ")");
        ROS_INFO_STREAM("Position: " << position[0] << ", " << position[1] << ", " << position[2]);
        ROS_INFO_STREAM("Orientation: " << orientation.w() << ", " << orientation.x() << ", " << orientation.y()<< ", " << orientation.z());

        //Calculate ViewCenterPoint
        //TODO:Abfangen von Drehungen um die Y-Achse
        Eigen::Affine3d targetCameraPoseEigen = Eigen::Affine3d(Eigen::Translation3d(Eigen::Vector3d(position[0], position[1], position[2])))*Eigen::Quaterniond(orientation.w(), orientation.x(), orientation.y(), orientation.z());
        Eigen::Affine3d viewCenterEigen = targetCameraPoseEigen * Eigen::Affine3d (Eigen::Translation3d(Eigen::Vector3d(0.0, mViewPointDistance, 0.0)));

        //Calculate TILT and position of Tilt joint
        Eigen::Vector3d planeNormal(targetCameraPoseEigen(0,0), targetCameraPoseEigen(1,0), targetCameraPoseEigen(2,0));
        Eigen::Vector3d targetViewVector(targetCameraPoseEigen(0,1), targetCameraPoseEigen(1,1), targetCameraPoseEigen(2,1));
        planeNormal.normalize();
        targetViewVector.normalize();
        ROS_INFO_STREAM("planeNormal: " << planeNormal[0] << ", " << planeNormal[1] << ", " << planeNormal[2]);
        ROS_INFO_STREAM("targetViewVector: " << targetViewVector[0] << ", " << targetViewVector[1] << ", " << targetViewVector[2]);
        //Calculate tilt base point (t1, t2, t3)
        double t1, t2, t3;
        //Calculate t3 using h
        t3 = h_tilt - viewCenterEigen(2,3);
        //Calculate t2 using abc-formula
        double a, b, c;
        a = 1 + pow(planeNormal(1)/planeNormal(0), 2.0);
        b = (2*t3*planeNormal(1)*planeNormal(2))/pow(planeNormal(0), 2.0);
        c = -pow(viewTriangle_sideA, 2.0) + pow(t3, 2.0)*(1+pow(planeNormal(2)/planeNormal(0), 2.0));
        if (pow(b, 2.0)<4*a*c)
        {
            ROS_ERROR_STREAM("No solution found.");
            return targetMILDRobotState;
        }

        double t2_1, t2_2, t1_1, t1_2;
        t2_1 = (-b + sqrt(pow(b, 2.0)-4*a*c))/(2*a);
        t2_2 = (-b - sqrt(pow(b, 2.0)-4*a*c))/(2*a);
        //Calculate feasible t1
        t1_1 = -(t2_1*planeNormal(1)+t3*planeNormal(2))/planeNormal(0);
        t1_2 = -(t2_2*planeNormal(1)+t3*planeNormal(2))/planeNormal(0);
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

        //get tilt base point
        Eigen::Vector3d tilt_base_point_projected(t1+viewCenterEigen(0,3), t2+viewCenterEigen(1,3), t3+viewCenterEigen(2,3));
        ROS_INFO_STREAM("tilt_base_point: " << tilt_base_point_projected[0] << ", " << tilt_base_point_projected[1] << ", " << tilt_base_point_projected[2]);
        double tilt;
        Eigen::Vector3d targetToBase(tilt_base_point_projected[0]-viewCenterEigen(0,3), tilt_base_point_projected[1]-viewCenterEigen(1,3), tilt_base_point_projected[2]-viewCenterEigen(2,3));
        targetToBase.normalize();
        double targetToBase_Angle = acos(targetToBase[2]);
        ROS_INFO_STREAM("targetToBase_Angle: " << targetToBase_Angle);
        tilt = targetToBase_Angle+viewTriangle_angleGamma-viewTriangle_angleAlpha-M_PI/2.0;

        ROS_INFO_STREAM("Tilt: " << tilt*(180.0/M_PI));

        Eigen::Vector3d tilt_base_point = tilt_base_point_projected + x_product*planeNormal;

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
        Eigen::Affine3d tiltedFrame =  tiltFrame * Eigen::AngleAxisd(-tilt, Eigen::Vector3d::UnitY());
        Eigen::Affine3d camFrame = tiltedFrame * tiltToCameraEigen;
        Eigen::Affine3d actualViewCenterEigen(Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, mViewPointDistance)));
        actualViewCenterEigen = camFrame*actualViewCenterEigen;
        
        Eigen::Affine3d pan_rotated_Frame = tiltFrame * panToTiltEigen.inverse();

        //Calculate PAN and base rotation
        double pan = getPanAngleFromPanJointPose(pan_rotated_Frame, sourceMILDRobotState);
        ROS_INFO_STREAM("Pan: " << pan*(180.0/M_PI));

        Eigen::Affine3d pan_Frame = pan_rotated_Frame * Eigen::AngleAxisd(-pan, Eigen::Vector3d::UnitZ());
        Eigen::Affine3d base_Frame = pan_Frame * baseToPanEigen.inverse();

        //Visualization
        Eigen::Vector3d base_point(base_Frame(0,3), base_Frame(1,3), base_Frame(2,3));
        Eigen::Vector3d pan_base_point(pan_Frame(0,3), pan_Frame(1,3), pan_Frame(2,3));
        Eigen::Vector3d pan_rotated_point(pan_rotated_Frame(0,3), pan_rotated_Frame(1,3), pan_rotated_Frame(2,3));
        Eigen::Vector3d cam_point(camFrame(0,3), camFrame(1,3), camFrame(2,3));
        Eigen::Vector3d actual_view_center_point(actualViewCenterEigen(0,3), actualViewCenterEigen(1,3), actualViewCenterEigen(2,3));
        Eigen::Vector3d target_view_center_point(viewCenterEigen(0,3), viewCenterEigen(1,3), viewCenterEigen(2,3));
        Eigen::Vector3d target_cam_point(position[0], position[1], position[2]);
        visualizeIKcalculation(base_point, pan_base_point, pan_rotated_point, tilt_base_point,tilt_base_point_projected, cam_point, actual_view_center_point,target_view_center_point, target_cam_point);

		// set pan
        targetMILDRobotState->pan = pan;

		// set rotation
        //targetMILDRobotState->rotation = sourceMILDRobotState->rotation + x_rot_plus - x_rot_minus;
		while (targetMILDRobotState->rotation < 0) { targetMILDRobotState->rotation += 2 * M_PI; };
		while (targetMILDRobotState->rotation > 2 * M_PI) { targetMILDRobotState->rotation -= 2 * M_PI; };

		// set tilt
        targetMILDRobotState->tilt = tilt;

		// set x, y
        targetMILDRobotState->x = base_Frame(0,3);
        targetMILDRobotState->y = base_Frame(1,3);
        ROS_DEBUG_STREAM("Targetstate: (Pan: " << targetMILDRobotState->pan << ", Tilt: " << targetMILDRobotState->tilt << ", Rotation " << targetMILDRobotState->rotation << ", X:" << targetMILDRobotState->x << ", Y:" << targetMILDRobotState->y << ")");
		return targetMILDRobotState;
	}

    double MILDRobotModelWithIK::getPanAngleFromPanJointPose(Eigen::Affine3d &panJointFrame, MILDRobotStatePtr &robotState)
    {
        unsigned int iterationCount = 1;
        double actualPhi = robotState->pan;
        double acualRho = robotState->rotation;
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
        ROS_INFO_STREAM("phiMin: " << phiMin << " phiMax: " << phiMax);
        Eigen::Vector4d color_failed(1.0,0.0,0.0,1.0);
        Eigen::Vector3d basePoint;
        Eigen::Vector3d panJointPoint;
        //do sampling
        do
        {
            std::ostringstream converter;
            converter << iterationCount;
            string nsIterationVector = string("iterationVector") + converter.str();
            double angleStepSize = currentAngleRange / mPanAngleSamplingStepsPerIteration;
            double currentAngle, currentRating, angleCenter;
            currentBestRating = newBestRating;
            angleCenter = currentBestAngle;
            newBestRating = -1.0;
            for (unsigned int i = 0; i < mPanAngleSamplingStepsPerIteration; i++)
            {
                currentAngle = angleCenter - currentAngleRange/2.0 + i * angleStepSize;
                if (phiMax>currentAngle && phiMin<currentAngle)
                {
                    //Calculate the base frame with respect to the current angle
                    baseFrame = panJointFrame * Eigen::AngleAxisd(-currentAngle, Eigen::Vector3d::UnitZ()) * baseToPanEigen.inverse();
                    basePoint = Eigen::Vector3d(baseFrame(0,3),baseFrame(1,3),baseFrame(2,3));
                    panJointPoint = Eigen::Vector3d(panJointFrame(0,3),panJointFrame(1,3),panJointFrame(2,3));
                    targetRobotPosition.x = baseFrame(0,3);
                    targetRobotPosition.y = baseFrame(1,3);
                    if (isPositionReachable(actualRobotPosition, targetRobotPosition))
                    {
                        nav_msgs::Path navigationPath = getNavigationPath(actualRobotPosition, targetRobotPosition);
                        currentRating = ikRatingModule->getPanAngleRating(panJointFrame, currentAngle, navigationPath);
                        ROS_DEBUG_STREAM("Angle: " << currentAngle << " with rating: " << currentRating);
                        if (currentRating > newBestRating)
                        {
                            newBestRating = currentRating;
                            currentBestAngle = currentAngle;
                        }
                        Eigen::Vector4d color_succeeded(1.0-currentRating,currentRating, 0.0, 1.0);
                        visualizeIKArrowSmall(basePoint, panJointPoint, color_succeeded, nsIterationVector, 2*i);
                        visualizeIKPoint(basePoint, color_succeeded, nsIterationVector, 2*i+1);
                    }
                    else
                    {
                        visualizeIKArrowSmall(basePoint, panJointPoint, color_failed, nsIterationVector, 2*i);
                        visualizeIKPoint(basePoint, color_failed, nsIterationVector, 2*i+1);
                    }
                }
            }
            ROS_INFO_STREAM("Best angle: " << currentBestAngle << " with rating: " << newBestRating);
            currentAngleRange = currentAngleRange / 2.0;
            iterationCount++;
        } while(fabs(currentBestRating-newBestRating) > mInverseKinematicIterationAccuracy);
        if (currentBestRating < 0.0) {ROS_ERROR_STREAM("No valid solution found for this pan frame.");}
        return currentBestAngle;
    }


    bool MILDRobotModelWithIK::setUpTFParameters()
    {
        //Get Parameters from TF-Publishers
        tf::StampedTransform tiltToCameraTF, panToTiltTF, baseToPanTF, tiltAxisPointTF,cameraLeftPointTF;
        Eigen::Affine3d tiltAxisPointEigen, cameraLeftPointEigen;
        ROS_INFO_STREAM("Looking up tf transforms...");
        try
        {
            //listener->waitForTransform("/map", "/ptu_tilted_link", ros::Time(0));
            //listener->waitForTransform("/ptu_tilted_link", "/camera_left_frame", ros::Time(0));
            listener->lookupTransform("/map", "/ptu_tilt_link", ros::Time(0), tiltAxisPointTF);
            listener->lookupTransform("/map", "/camera_left_frame", ros::Time(0), cameraLeftPointTF);
            listener->lookupTransform("/ptu_tilted_link", "/camera_left_frame", ros::Time(0), tiltToCameraTF);
            listener->lookupTransform("/ptu_pan_link", "/ptu_tilt_link", ros::Time(0), panToTiltTF);
            listener->lookupTransform("/base_link", "/ptu_pan_link", ros::Time(0), baseToPanTF);
        }
        catch (tf::TransformException ex)
        {
          ROS_ERROR("An error occured during tf-lookup: %s",ex.what());
          return false;
        }
        tf::poseTFToEigen(tiltToCameraTF, tiltToCameraEigen);
        tf::poseTFToEigen(panToTiltTF, panToTiltEigen);
        tf::poseTFToEigen(baseToPanTF, baseToPanEigen);
        tf::poseTFToEigen(tiltAxisPointTF, tiltAxisPointEigen);
        tf::poseTFToEigen(cameraLeftPointTF, cameraLeftPointEigen);
        h_tilt = tiltAxisPointEigen.matrix()(2,3);
        ROS_INFO_STREAM("Height above ground: " << h_tilt);

        Eigen::Vector3d cam_axis_x(cameraLeftPointEigen(0,0), cameraLeftPointEigen(1,0), cameraLeftPointEigen(2,0));
        Eigen::Vector3d cam_axis_y(cameraLeftPointEigen(0,1), cameraLeftPointEigen(1,1), cameraLeftPointEigen(2,1));
        Eigen::Vector3d cam_axis_z(cameraLeftPointEigen(0,2), cameraLeftPointEigen(1,2), cameraLeftPointEigen(2,2));
        cam_axis_x.normalize();
        cam_axis_y.normalize();
        cam_axis_z.normalize();
        Eigen::Vector3d tilt_to_cam(tiltAxisPointEigen(0,3)-cameraLeftPointEigen(0,3), tiltAxisPointEigen(1,3)-cameraLeftPointEigen(1,3), tiltAxisPointEigen(2,3)-cameraLeftPointEigen(2,3));
        x_product = cam_axis_x.dot(tilt_to_cam);
        tilt_to_cam -= x_product*tilt_to_cam;
        double viewTriangle_sideB = tilt_to_cam.norm();
        tilt_to_cam.normalize();
        viewTriangle_angleAlpha = acos(cam_axis_z.dot(tilt_to_cam));
        viewTriangle_sideA = sqrt(pow(mViewPointDistance,2.0)+pow(viewTriangle_sideB,2.0)-2*mViewPointDistance*viewTriangle_sideB*cos(viewTriangle_angleAlpha));
        viewTriangle_angleGamma = pow(mViewPointDistance, 2.0) - pow(viewTriangle_sideA,2.0)-pow(viewTriangle_sideB,2.0);
        viewTriangle_angleGamma = viewTriangle_angleGamma / (-2.0*viewTriangle_sideA*viewTriangle_sideB);
        viewTriangle_angleGamma = acos(viewTriangle_angleGamma);
        ROS_INFO_STREAM("viewTriangle_angleAlpha: " << viewTriangle_angleAlpha);
        ROS_INFO_STREAM("viewTriangle_angleGamma: " << viewTriangle_angleGamma);
        ROS_INFO_STREAM("viewTriangle_sideB: " << viewTriangle_sideB);
        ROS_INFO_STREAM("viewTriangle_sideA: " << viewTriangle_sideA);
        ROS_INFO_STREAM("TF lookup successful.");

        return true;
    }

    void MILDRobotModelWithIK::visualizeIKcalculation(Eigen::Vector3d &base_point, Eigen::Vector3d &pan_joint_point, Eigen::Vector3d & pan_rotated_point, Eigen::Vector3d &tilt_base_point, Eigen::Vector3d &tilt_base_point_projected, Eigen::Vector3d &cam_point, Eigen::Vector3d &actual_view_center_point, Eigen::Vector3d &target_view_center_point, Eigen::Vector3d &target_camera_point)
    {
        Eigen::Vector4d color_red(1.0,0.0,0.0,1.0);
        Eigen::Vector4d color_green(0.0,1.0,0.0,1.0);
        Eigen::Vector4d color_blue(0.0,0.0,1.0,1.0);
        visualizeIKArrowLarge(target_camera_point, target_view_center_point, color_green, "targetCameraVector", 0);
        visualizeIKArrowLarge(cam_point, actual_view_center_point, color_blue, "camToActualViewCenterVector", 0);
        visualizeIKArrowSmall(tilt_base_point,cam_point, color_blue, "tiltToCamVector", 0);
        visualizeIKPoint(tilt_base_point_projected, color_red, "tiltBaseVectorProjected", 0);
        visualizeIKPoint(tilt_base_point, color_blue, "tiltBaseVector", 0);
        visualizeIKArrowSmall(pan_rotated_point,tilt_base_point, color_blue, "panToTiltVector", 0);
        visualizeIKPoint(pan_rotated_point, color_blue, "panBaseVector", 0);
        visualizeIKArrowSmall(base_point,pan_joint_point, color_blue, "baseToPanVector", 0);
        visualizeIKPoint(base_point, color_blue, "baseVector", 0);
    }


    void MILDRobotModelWithIK::visualizeIKPoint(Eigen::Vector3d &point, Eigen::Vector4d &colorRGBA, string ns, int id)
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

    void MILDRobotModelWithIK::visualizeIKArrowSmall(Eigen::Vector3d &pointStart, Eigen::Vector3d &pointEnd, Eigen::Vector4d &colorRGBA, std::string ns, int id)
    {
        Eigen::Vector3d scaleParameters(0.005, 0.01, 0.01);
        visualizeIKArrow(pointStart, pointEnd, colorRGBA, ns, scaleParameters, id);
    }

    void MILDRobotModelWithIK::visualizeIKArrowLarge(Eigen::Vector3d &pointStart, Eigen::Vector3d &pointEnd, Eigen::Vector4d &colorRGBA, std::string ns, int id)
    {
        Eigen::Vector3d scaleParameters(0.02, 0.05, 0.1);
        visualizeIKArrow(pointStart, pointEnd, colorRGBA, ns, scaleParameters, id);
    }

    void MILDRobotModelWithIK::visualizeIKArrow(Eigen::Vector3d &pointStart, Eigen::Vector3d &pointEnd, Eigen::Vector4d &colorRGBA, string ns, Eigen::Vector3d &scaleParameters, int id)
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


    float MILDRobotModelWithIK::getBase_TranslationalMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState)
    {
        MILDRobotStatePtr sourceMILDRobotState = boost::static_pointer_cast<MILDRobotState>(sourceRobotState);
        MILDRobotStatePtr targetMILDRobotState = boost::static_pointer_cast<MILDRobotState>(targetRobotState);

        float distance ;
        geometry_msgs::Point sourcePoint, targetPoint;
        sourcePoint.x = sourceMILDRobotState->x;
        sourcePoint.y = sourceMILDRobotState->y;
        sourcePoint.z = 0;
        targetPoint.x = targetMILDRobotState->x;
        targetPoint.y = targetMILDRobotState->y;
        targetPoint.z = 0;

        distance = getDistance(sourcePoint, targetPoint);

        float mu = 0.0;
        float sigma = 0.5;
        float movementCosts = std::exp(-pow((distance-mu), 2.0)/(2.0*pow(sigma, 2.0))); // [0, 1]
        return movementCosts;
    }

    float MILDRobotModelWithIK::getPTU_PanMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState)
    {
        MILDRobotStatePtr sourceMILDRobotState = boost::static_pointer_cast<MILDRobotState>(sourceRobotState);
        MILDRobotStatePtr targetMILDRobotState = boost::static_pointer_cast<MILDRobotState>(targetRobotState);

        float panDiff = targetMILDRobotState->pan - sourceMILDRobotState->pan;

        float panSpan = mPanLimits.get<1>() - mPanLimits.get<0>();

        float ptuPanCosts = fabs(panDiff)/ panSpan;

        return 1.0 - ptuPanCosts;
    }

    float MILDRobotModelWithIK::getPTU_TiltMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState)
    {
        MILDRobotStatePtr sourceMILDRobotState = boost::static_pointer_cast<MILDRobotState>(sourceRobotState);
        MILDRobotStatePtr targetMILDRobotState = boost::static_pointer_cast<MILDRobotState>(targetRobotState);
        float tiltDiff = targetMILDRobotState->tilt - sourceMILDRobotState->tilt;

        float tiltSpan = mTiltLimits.get<1>() - mTiltLimits.get<0>();

        float ptuTiltCosts = fabs(tiltDiff)/ tiltSpan;

        return 1.0 - ptuTiltCosts;
    }

    float MILDRobotModelWithIK::getBase_RotationalMovementCosts(const RobotStatePtr &sourceRobotState, const RobotStatePtr &targetRobotState)
    {
        MILDRobotStatePtr sourceMILDRobotState = boost::static_pointer_cast<MILDRobotState>(sourceRobotState);
        MILDRobotStatePtr targetMILDRobotState = boost::static_pointer_cast<MILDRobotState>(targetRobotState);

        float rotDiff = targetMILDRobotState->rotation - sourceMILDRobotState->rotation;

        geometry_msgs::Point sourcePoint, targetPoint;
        sourcePoint.x = sourceMILDRobotState->x;
        sourcePoint.y = sourceMILDRobotState->y;
        sourcePoint.z = 0;
        targetPoint.x = targetMILDRobotState->x;
        targetPoint.y = targetMILDRobotState->y;
        targetPoint.z = 0;


        float rotationCosts = std::min(fabs(rotDiff), (float)(2.0f*M_PI-fabs(rotDiff)))/M_PI;
        return 1.0 - rotationCosts;
    }

    float MILDRobotModelWithIK::getDistance(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition)
    {
        float distance = 0;
        float costs = 0;
        if (useGlobalPlanner) //Use global planner to calculate distance
        {
            nav_msgs::Path path;
            ROS_DEBUG_STREAM("Calculate path from (" << sourcePosition.x << ", " << sourcePosition.y << ") to (" << targetPosition.x << ", "<< targetPosition.y << ")");

            path = getNavigationPath(sourcePosition, targetPosition);

            if (!path.poses.empty())
            {
                unsigned int size = path.poses.size();
                distance = 0;
                for (unsigned int i = 0; i < size ; i++)
                {
                    ROS_DEBUG_STREAM("Path (" << path.poses[i].pose.position.x << ", " << path.poses[i].pose.position.y << ")");
                }
                //Calculate distance from source point to first point in path
                distance += sqrt(pow(sourcePosition.x - path.poses[0].pose.position.x, 2) + pow(sourcePosition.y - path.poses[0].pose.position.y, 2));
                //Calculate path length
                for (unsigned int i = 0; i < size - 1 ; i++)
                {
                    distance += sqrt(pow(path.poses[i].pose.position.x - path.poses[i+1].pose.position.x, 2) + pow(path.poses[i].pose.position.y - path.poses[i+1].pose.position.y, 2));
                }
            }
            else
            {
                costs = -1;
                ROS_ERROR("Could not get navigation path..");
            }
        }
        else //Use euclidean distance
        {
            distance = sqrt(pow(sourcePosition.x -targetPosition.x, 2) + pow(sourcePosition.y - targetPosition.y, 2));
        }
        ROS_DEBUG_STREAM("Global planner distance: " << distance);
        ROS_DEBUG_STREAM("Euclidian distance: " << sqrt(pow(sourcePosition.x -targetPosition.x, 2) + pow(sourcePosition.y - targetPosition.y, 2)));
        return distance;
    }
    nav_msgs::Path MILDRobotModelWithIK::getNavigationPath(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition)
    {
        return getNavigationPath(sourcePosition, targetPosition, 0, 0);
    }

    nav_msgs::Path MILDRobotModelWithIK::getNavigationPath(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition, double sourceRotationBase, double targetRotationBase)
    {
        nav_msgs::GetPlan srv;
        srv.request.start.header.frame_id = "map";
        srv.request.goal.header.frame_id = "map";
        srv.request.start.pose.position = sourcePosition;
        srv.request.goal.pose.position = targetPosition;
        Eigen::Quaterniond sourceRotationEigen(Eigen::AngleAxisd(sourceRotationBase,Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond targetRotationEigen(Eigen::AngleAxisd(targetRotationBase,Eigen::Vector3d::UnitZ()));
        geometry_msgs::Quaternion sourceRotation;
        sourceRotation.w = sourceRotationEigen.w();
        sourceRotation.x = sourceRotationEigen.x();
        sourceRotation.y = sourceRotationEigen.y();
        sourceRotation.z = sourceRotationEigen.z();
        geometry_msgs::Quaternion targetRotation;
        targetRotation.w = targetRotationEigen.w();
        targetRotation.x = targetRotationEigen.x();
        targetRotation.y = targetRotationEigen.y();
        targetRotation.z = targetRotationEigen.z();
        srv.request.start.pose.orientation = sourceRotation;
        srv.request.goal.pose.orientation = targetRotation;
        srv.request.tolerance = tolerance;

        nav_msgs::Path path;
        if (navigationCostClient.call(srv))
        {
            path = srv.response.plan;
            ROS_DEBUG_STREAM("Path size:" << path.poses.size());
        }
        else
        {
            ROS_ERROR("Failed to call the global planner.");
        }
        return path;
    }

    // Not fully implemented
    // Guideline: http://www.orocos.org/kdl/examples
    geometry_msgs::Pose MILDRobotModelWithIK::calculateCameraPose(const RobotStatePtr &sourceRobotState)
    {
        /*MILDRobotStatePtr sourceMILDRobotState = boost::static_pointer_cast<MILDRobotState>(sourceRobotState);

        urdf::Model * myModel;
        geometry_msgs::Pose cameraPose;

        tf::Transform pan(tf::Quaternion(tf::Vector3(0,0,1), sourceMILDRobotState->pan));
        tf::Transform tilt(tf::Quaternion(tf::Vector3(0,-1,0), sourceMILDRobotState->tilt));

        urdf::Model myModel;
        ros::Rate loop_rate(30);
        //geometry_msgs::Pose cameraPose;

        if(!myModel.initParam("/robot_description"))
        {
            ROS_ERROR("Could not get robot description.");
        }
        else
        {
            KDL::Tree my_tree;
            if (!kdl_parser::treeFromUrdfModel(myModel, my_tree))
            {
                   ROS_ERROR("Failed to construct kdl tree");
            }
            else
            {
                 KDL::Chain chain;
                 my_tree.getChain(myModel.getRoot()->name, "mount_to_camera_left", chain);
                 ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);


                 //robot_state_publisher::RobotStatePublisher* publisher = new robot_state_publisher::RobotStatePublisher(my_tree);
                 //my_tree.getSegment()

                 typedef std::map<std::string, boost::shared_ptr<urdf::Joint> > joint_map;

                 std::map<std::string, double> positions;
                 joint_map joints = myModel.joints_;

                 for(joint_map::const_iterator it =  joints.begin(); it !=  joints.end(); ++it)
                 {
                    std::string name = it->first;
                    urdf::Joint* joint = it->second.get();
                    //joint->dynamics.
                    ROS_DEBUG_STREAM(name);
                    if(joint->type==urdf::Joint::REVOLUTE || joint->type==urdf::Joint::CONTINUOUS || joint->type==urdf::Joint::PRISMATIC) positions[name] = 0;
                 }
                 while (ros::ok())
                 {
                     //publisher->publishTransforms(positions, ros::Time::now());

                     // Process a round of subscription messages
                     ros::spinOnce();

                     // This will adjust as needed per iteration
                     loop_rate.sleep();
                 }
             }
        } */
        geometry_msgs::Pose myPose;
        return myPose;
    }
}

