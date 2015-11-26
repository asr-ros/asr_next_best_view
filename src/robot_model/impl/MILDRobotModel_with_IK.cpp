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
#include <tf_conversions/tf_eigen.h>

#include <visualization_msgs/Marker.h>

namespace next_best_view {
    MILDRobotModelWithIK::MILDRobotModelWithIK() : RobotModel() {
        ros::NodeHandle n("nbv_srv");
        navigationCostClient = n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
        double mOmegaPan_, mOmegaTilt_, mOmegaUseBase_, tolerance_, speedFactorPTU_,speedFactorBaseMove_,speedFactorBaseRot_;
        bool useGlobalPlanner_;
        n.getParam("mOmegaPan", mOmegaPan_);
        n.getParam("mOmegaTilt", mOmegaTilt_);
        n.getParam("mOmegaUseBase", mOmegaUseBase_);
        n.getParam("speedFactorPTU", speedFactorPTU_);
        n.getParam("speedFactorBaseMove", speedFactorBaseMove_);
        n.getParam("speedFactorBaseRot", speedFactorBaseRot_);
        n.getParam("tolerance", tolerance_);
        n.getParam("useGlobalPlanner", useGlobalPlanner_);
        useGlobalPlanner = useGlobalPlanner_;
        if (useGlobalPlanner_)
        {
            ROS_DEBUG("Use of global planner ENABLED");
        }
        else
        {
            ROS_DEBUG("Use of global planner DISABLED. Using simplified calculation instead...");
        }

        ROS_DEBUG_STREAM("mOmegaUseBase: " << mOmegaUseBase_);
        ROS_DEBUG_STREAM("speedFactorPTU: " << speedFactorPTU_);
        ROS_DEBUG_STREAM("speedFactorBaseMove: " << speedFactorBaseMove_);
        ROS_DEBUG_STREAM("speedFactorBaseRot: " << speedFactorBaseRot_);
        ROS_DEBUG_STREAM("tolerance: " << tolerance_);
        ROS_DEBUG_STREAM("mOmegaPan: " << mOmegaPan_);
        ROS_DEBUG_STREAM("mOmegaTilt: " << mOmegaTilt_);
        mOmegaPan = mOmegaPan_;
        mOmegaTilt = mOmegaTilt_;
        mOmegaUseBase = mOmegaUseBase_;
        speedFactorPTU = speedFactorPTU_;
        speedFactorBaseMove = speedFactorBaseMove_;
        speedFactorBaseRot = speedFactorBaseRot_;
        tolerance = tolerance_;
        viewPointDistance = 1.0; //!!!!TODO: Read in distance!!!!
		this->setPanAngleLimits(0, 0);
		this->setTiltAngleLimits(0, 0);
        this->setRotationAngleLimits(0, 0);
        listener = new tf::TransformListener();
        //Temporary Visualization Publisher
        vis_pub = n.advertise<visualization_msgs::Marker>( "/nbv/IK_Visualization", 1000);
        setUpTFParameters();
        setUpTFParameters();
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
        this->viewPointDistance = viewPointDistance;
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
        setUpTFParameters();
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

        ROS_INFO_STREAM("Calculate state for: (Pan: " << sourceMILDRobotState->pan << ", Tilt: " << sourceMILDRobotState->tilt << ", Rotation " << sourceMILDRobotState->rotation << ", X:" << sourceMILDRobotState->x << ", Y:" << sourceMILDRobotState->y << ")");
        ROS_INFO_STREAM("Position: " << position[0] << ", " << position[1] << ", " << position[2]);
        ROS_INFO_STREAM("Orientation: " << orientation.w() << ", " << orientation.x() << ", " << orientation.y()<< ", " << orientation.z());

        //double viewTriangle_angleAlpha = sphereCoords[2] - currentPhi - currentRho;
        //viewTriangle_angleAlpha = viewTriangle_angleAlpha > M_PI ? viewTriangle_angleAlpha - 2 * M_PI : viewTriangle_angleAlpha;
        //viewTriangle_angleAlpha = viewTriangle_angleAlpha < -M_PI ? viewTriangle_angleAlpha + 2 * M_PI : viewTriangle_angleAlpha;

        //Calculate ViewCenterPoint
        //TODO:Abfangen von Drehungen um die Y-Achse
        Eigen::Affine3d targetCameraPoseEigen(Eigen::Translation3d(Eigen::Vector3d(position[0], position[1], position[2])));
        targetCameraPoseEigen = targetCameraPoseEigen*Eigen::Quaterniond(orientation.w(), orientation.x(), orientation.y(), orientation.z());
        Eigen::Affine3d viewCenterEigen(Eigen::Translation3d(Eigen::Vector3d(0.0, viewPointDistance, 0.0)));
        viewCenterEigen = targetCameraPoseEigen*viewCenterEigen;
        ROS_INFO_STREAM("View Center:");
        ROS_INFO_STREAM(viewCenterEigen(0,0) << ", " << viewCenterEigen(0,1) << ", " <<viewCenterEigen(0,2) << ", " <<viewCenterEigen(0,3));
        ROS_INFO_STREAM(viewCenterEigen(1,0) << ", " << viewCenterEigen(1,1) << ", " <<viewCenterEigen(1,2) << ", " <<viewCenterEigen(1,3));
        ROS_INFO_STREAM(viewCenterEigen(2,0) << ", " << viewCenterEigen(2,1) << ", " <<viewCenterEigen(2,2) << ", " <<viewCenterEigen(2,3));

        visualization_msgs::Marker targetCameraVector = visualization_msgs::Marker();
        targetCameraVector.header.stamp = ros::Time();
        targetCameraVector.header.frame_id = "/map";
        targetCameraVector.type = targetCameraVector.ARROW;
        targetCameraVector.action = targetCameraVector.ADD;
        targetCameraVector.id = 0;
        targetCameraVector.lifetime = ros::Duration();
        targetCameraVector.ns = "targetCamera";
        targetCameraVector.scale.x = 0.02;
        targetCameraVector.scale.y = 0.05;
        targetCameraVector.scale.z = 0.1;
        targetCameraVector.color.a = 1;
        targetCameraVector.color.r = 0;
        targetCameraVector.color.g = 1;
        targetCameraVector.color.b = 0;
        geometry_msgs::Point point1;
        geometry_msgs::Point point2;
        point1.x = position[0];
        point1.y = position[1];
        point1.z = position[2];
        point2.x = viewCenterEigen(0,3);
        point2.y = viewCenterEigen(1,3);
        point2.z = viewCenterEigen(2,3);
        targetCameraVector.points.push_back(point1);
        targetCameraVector.points.push_back(point2);
        vis_pub.publish(targetCameraVector);

        //BEGIN:Calculate TILT and position of tilt joint
        //----------------------------------------------
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
        c = -pow(viewTriangle_sideC, 2.0) + pow(t3, 2.0)*(1+pow(planeNormal(2)/planeNormal(0), 2.0));
        if (pow(b, 2.0)<4*a*c)
        {
            ROS_ERROR_STREAM("No solution found.");
            return targetMILDRobotState;
        }


        double t2_1, t2_2, t1_1, t1_2;
        t2_1 = (-b + sqrt(pow(b, 2.0)-4*a*c))/(2*a);
        t2_2 = (-b - sqrt(pow(b, 2.0)-4*a*c))/(2*a);
        ROS_INFO_STREAM("a: " << a << " b: " << b << " c: " << c);
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
        ROS_INFO_STREAM("t1_1 " << t1_1 << " t1_2: " << t1_2);
        ROS_INFO_STREAM("t2_1 " << t2_1 << " t2_2: " << t2_2);
        ROS_INFO_STREAM("Transform: " << t1 << ", " << t2 << ", " << t3);

        //get tilt base point
        Eigen::Vector3d tiltBasePoint(t1+viewCenterEigen(0,3), t2+viewCenterEigen(1,3), t3+viewCenterEigen(2,3));
        ROS_INFO_STREAM("tiltBasePoint: " << tiltBasePoint[0] << ", " << tiltBasePoint[1] << ", " << tiltBasePoint[2]);
        double tilt;
        //Calculate the point where sideA meets the XY plane
        //if (fabs(targetViewVector[2]) < 0.001) {
        //    tilt = 0.0;
        //}
        //else
        {
            Eigen::Vector3d targetToBase(tiltBasePoint[0]-viewCenterEigen(0,3), tiltBasePoint[1]-viewCenterEigen(1,3), tiltBasePoint[2]-viewCenterEigen(2,3));
            targetToBase.normalize();
            double targetToBase_Angle = acos(targetToBase[2]);

            ROS_INFO_STREAM("targetToBase_Angle: " << targetToBase_Angle);

            tilt = viewTriangle_angleAlpha - targetToBase_Angle;
        }
        ROS_INFO_STREAM("viewTriangle_sideC: " << viewTriangle_sideC << " tilt: " << tilt);
        //----------------------------------------------
        //END:Calculate TILT and position of tilt joint

        visualization_msgs::Marker tiltBaseVectorProjected = visualization_msgs::Marker();
        tiltBaseVectorProjected.header.stamp = ros::Time();
        tiltBaseVectorProjected.header.frame_id = "/map";
        tiltBaseVectorProjected.type = targetCameraVector.SPHERE;
        tiltBaseVectorProjected.action = tiltBaseVectorProjected.ADD;
        tiltBaseVectorProjected.id = 0;
        tiltBaseVectorProjected.lifetime = ros::Duration();
        tiltBaseVectorProjected.ns = "tiltBaseVectorProjected";
        tiltBaseVectorProjected.scale.x = 0.02;
        tiltBaseVectorProjected.scale.y = 0.02;
        tiltBaseVectorProjected.scale.z = 0.02;
        tiltBaseVectorProjected.color.a = 1;
        tiltBaseVectorProjected.color.r = 1;
        tiltBaseVectorProjected.color.g = 0;
        tiltBaseVectorProjected.color.b = 0;
        tiltBaseVectorProjected.pose.position.x = tiltBasePoint(0);
        tiltBaseVectorProjected.pose.position.y = tiltBasePoint(1);
        tiltBaseVectorProjected.pose.position.z = tiltBasePoint(2);
        vis_pub.publish(tiltBaseVectorProjected);

        tiltBasePoint += x_product*planeNormal;

        visualization_msgs::Marker tiltBaseVector = visualization_msgs::Marker();
        tiltBaseVector.header.stamp = ros::Time();
        tiltBaseVector.header.frame_id = "/map";
        tiltBaseVector.type = targetCameraVector.SPHERE;
        tiltBaseVector.action = tiltBaseVector.ADD;
        tiltBaseVector.id = 0;
        tiltBaseVector.lifetime = ros::Duration();
        tiltBaseVector.ns = "tiltBaseVector";
        tiltBaseVector.scale.x = 0.02;
        tiltBaseVector.scale.y = 0.02;
        tiltBaseVector.scale.z = 0.02;
        tiltBaseVector.color.a = 1;
        tiltBaseVector.color.r = 0;
        tiltBaseVector.color.g = 0;
        tiltBaseVector.color.b = 1;
        tiltBaseVector.pose.position.x = tiltBasePoint(0);
        tiltBaseVector.pose.position.y = tiltBasePoint(1);
        tiltBaseVector.pose.position.z = tiltBasePoint(2);
        vis_pub.publish(tiltBaseVector);
        
        //----------------------------------------------
        //BEGIN: Calculate new Camera Frame for visualization
        Eigen::Vector3d viewDirection;
        viewDirection = targetViewVector;
        viewDirection[2] = 0.0;
        viewDirection.normalize();
        Eigen::Matrix4d tiltFrame_Rotation;
        tiltFrame_Rotation.col(0) << viewDirection[0], viewDirection[1],  0.0 , 0.0;
        tiltFrame_Rotation.col(1) << -planeNormal[0] , -planeNormal[1] ,  -planeNormal[2] ,0.0;
        tiltFrame_Rotation.col(2) << 0.0, 0.0, 1.0, 0.0;
        tiltFrame_Rotation.col(3) << tiltBasePoint[0] , tiltBasePoint[1] ,  tiltBasePoint[2] , 1.0;

        Eigen::Affine3d tiltFrame(tiltFrame_Rotation);
        Eigen::Affine3d tiltedFrame =  tiltFrame * Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitY());
        Eigen::Affine3d camFrame = tiltedFrame * tiltToCameraEigen;
        Eigen::Affine3d actualViewCenterEigen(Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, viewPointDistance)));
        actualViewCenterEigen = camFrame*actualViewCenterEigen;
        
        visualization_msgs::Marker tiltToCamVector = visualization_msgs::Marker();
        tiltToCamVector.header.stamp = ros::Time();
        tiltToCamVector.header.frame_id = "/map";
        tiltToCamVector.type = targetCameraVector.ARROW;
        tiltToCamVector.action = tiltToCamVector.ADD;
        tiltToCamVector.id = 0;
        tiltToCamVector.lifetime = ros::Duration();
        tiltToCamVector.ns = "tiltToCamVector";
        tiltToCamVector.scale.x = 0.005;     //d Shaft
        tiltToCamVector.scale.y = 0.01;    //d Head
        tiltToCamVector.scale.z = 0.01;     //l Head
        tiltToCamVector.color.a = 1;
        tiltToCamVector.color.r = 0;
        tiltToCamVector.color.g = 0;
        tiltToCamVector.color.b = 1;
        point1.x = tiltBasePoint[0];
        point1.y = tiltBasePoint[1];
        point1.z = tiltBasePoint[2];
        point2.x = camFrame(0,3);
        point2.y = camFrame(1,3);
        point2.z = camFrame(2,3);
        tiltToCamVector.points.push_back(point1);
        tiltToCamVector.points.push_back(point2);
        vis_pub.publish(tiltToCamVector);

        visualization_msgs::Marker camToActualViewCenterVector = visualization_msgs::Marker();
        camToActualViewCenterVector.header.stamp = ros::Time();
        camToActualViewCenterVector.header.frame_id = "/map";
        camToActualViewCenterVector.type = targetCameraVector.ARROW;
        camToActualViewCenterVector.action = camToActualViewCenterVector.ADD;
        camToActualViewCenterVector.id = 0;
        camToActualViewCenterVector.lifetime = ros::Duration();
        camToActualViewCenterVector.ns = "camToActualViewCenterVector";
        camToActualViewCenterVector.scale.x = 0.02;
        camToActualViewCenterVector.scale.y = 0.05;
        camToActualViewCenterVector.scale.z = 0.1;
        camToActualViewCenterVector.color.a = 1;
        camToActualViewCenterVector.color.r = 0;
        camToActualViewCenterVector.color.g = 0;
        camToActualViewCenterVector.color.b = 1;
        point1.x = camFrame(0,3);
        point1.y = camFrame(1,3);
        point1.z = camFrame(2,3);
        point2.x = actualViewCenterEigen(0,3);
        point2.y = actualViewCenterEigen(1,3);
        point2.z = actualViewCenterEigen(2,3);
        camToActualViewCenterVector.points.push_back(point1);
        camToActualViewCenterVector.points.push_back(point2);
        vis_pub.publish(camToActualViewCenterVector);

        //----------------------------------------------
        //END: Calculate new Camera Frame for visualization

		// set pan
        //targetMILDRobotState->pan = sourceMILDRobotState->pan + x_pan_plus - x_pan_minus;

		// set rotation
        //targetMILDRobotState->rotation = sourceMILDRobotState->rotation + x_rot_plus - x_rot_minus;
		while (targetMILDRobotState->rotation < 0) { targetMILDRobotState->rotation += 2 * M_PI; };
		while (targetMILDRobotState->rotation > 2 * M_PI) { targetMILDRobotState->rotation -= 2 * M_PI; };

		// set tilt
		targetMILDRobotState->tilt = sphereCoords[1];

		// set x, y
		targetMILDRobotState->x = position[0];
		targetMILDRobotState->y = position[1];
        ROS_DEBUG_STREAM("Targetstate: (Pan: " << targetMILDRobotState->pan << ", Tilt: " << targetMILDRobotState->tilt << ", Rotation " << targetMILDRobotState->rotation << ", X:" << targetMILDRobotState->x << ", Y:" << targetMILDRobotState->y << ")");
		return targetMILDRobotState;
	}

    void MILDRobotModelWithIK::setUpTFParameters()
    {
        //Get Parameters from TF-Publishers
        tf::StampedTransform cameraPoseTF, tiltToCameraTF, tiltAxisPointTF;
        Eigen::Affine3d cameraPoseEigen, tiltAxisPointEigen;
        ROS_INFO_STREAM("Lookup transform");
        try
        {
            //listener->waitForTransform("/map", "/ptu_tilted_link", ros::Time(0));
            //listener->waitForTransform("/ptu_tilted_link", "/camera_left_frame", ros::Time(0));
            listener->lookupTransform("/map", "/ptu_tilted_link", ros::Time(0), tiltAxisPointTF);
            listener->lookupTransform("/ptu_tilted_link", "/camera_left_frame", ros::Time(0), tiltToCameraTF);
        }
        catch (tf::TransformException ex)
        {
          ROS_ERROR("An error occured during tf-lookup: %s",ex.what());
          return;
        }
        tf::poseTFToEigen(cameraPoseTF, cameraPoseEigen);
        tf::poseTFToEigen(tiltToCameraTF, tiltToCameraEigen);
        tf::poseTFToEigen(tiltAxisPointTF, tiltAxisPointEigen);
        cameraPoseEigen = tiltAxisPointEigen*tiltToCameraEigen;
        h_tilt = cameraPoseEigen.matrix()(2,3);
        ROS_INFO_STREAM("Height above ground: " << h_tilt);

        Eigen::Vector3d cam_axis_x(cameraPoseEigen(0,0), cameraPoseEigen(1,0), cameraPoseEigen(2,0));
        Eigen::Vector3d cam_axis_z(cameraPoseEigen(0,2), cameraPoseEigen(1,2), cameraPoseEigen(2,2));
        cam_axis_x.normalize();
        cam_axis_z.normalize();
        ROS_INFO_STREAM("cam_axis_x: " << cam_axis_x[0] << ", " << cam_axis_x[1] << ", " << cam_axis_x[2]);
        ROS_INFO_STREAM("cam_axis_z: " << cam_axis_z[0] << ", " << cam_axis_z[1] << ", " << cam_axis_z[2]);
        Eigen::Vector3d tilt_to_cam(tiltAxisPointEigen(0,3)-cameraPoseEigen(0,3), tiltAxisPointEigen(1,3)-cameraPoseEigen(1,3), tiltAxisPointEigen(2,3)-cameraPoseEigen(2,3));
        x_product = cam_axis_x.dot(tilt_to_cam);
        ROS_INFO_STREAM("x_product: " << x_product);
        tilt_to_cam -= x_product*tilt_to_cam;
        double viewTriangle_sideB = tilt_to_cam.norm();
        tilt_to_cam.normalize();
        double viewTriangle_angleGamma = acos(cam_axis_z.dot(tilt_to_cam));
        viewTriangle_sideC = sqrt(pow(viewPointDistance,2.0)+pow(viewTriangle_sideB,2.0)-2*viewPointDistance*viewTriangle_sideB*cos(viewTriangle_angleGamma));
        viewTriangle_angleAlpha = pow(viewPointDistance,2.0)-pow(viewTriangle_sideB,2.0)-pow(viewTriangle_sideC,2.0);
        viewTriangle_angleAlpha /= (-2*viewTriangle_sideB*viewTriangle_sideC);
        viewTriangle_angleAlpha = acos(viewTriangle_angleAlpha);
        viewTriangle_angleAlpha = M_PI/2.0;
        ROS_INFO_STREAM("viewTriangle_angleAlpha: " << viewTriangle_angleAlpha);
        ROS_INFO_STREAM("viewTriangle_angleGamma: " << viewTriangle_angleGamma);
        ROS_INFO_STREAM("viewTriangle_sideB: " << viewTriangle_sideB);
        ROS_INFO_STREAM("viewTriangle_sideC: " << viewTriangle_sideC);
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
        nav_msgs::GetPlan srv;
        srv.request.start.header.frame_id = "map";
        srv.request.goal.header.frame_id = "map";
        srv.request.start.pose.position = sourcePosition;
        srv.request.goal.pose.position = targetPosition;
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

