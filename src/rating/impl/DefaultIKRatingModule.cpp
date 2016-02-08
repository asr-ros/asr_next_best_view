/*
 * DefaultIKRatingModule.cpp
 *
 *  Created on: Dez 23, 2015
 *      Author: florianaumann
 */
#include "next_best_view/rating/impl/DefaultIKRatingModule.h"
#include <ros/ros.h>

namespace next_best_view {
    DefaultIKRatingModule::DefaultIKRatingModule(RobotModelPtr robotModel) : IKRatingModule()
    {
        mRobotModel = robotModel;
    }
    DefaultIKRatingModule::~DefaultIKRatingModule() { }

    double DefaultIKRatingModule::getPanAngleRating(Eigen::Affine3d &panJointFrame, double panAngle, nav_msgs::Path &navigationPath)
    {
        if (navigationPath.poses.size() > 2)
        {
            //Get length and angle change of the path
            double length = 0;
            double currentDotProduct;
            double absolutAngleChange = 0;
            std::vector<geometry_msgs::PoseStamped>::iterator poseIterator = navigationPath.poses.end();
            geometry_msgs::Pose previousPose_2 = poseIterator->pose;
            poseIterator--;
            geometry_msgs::Pose previousPose_1 = poseIterator->pose;
            poseIterator--;
            do
            {
                length += sqrt(pow(previousPose_1.position.x-(*poseIterator).pose.position.x, 2.0)+pow(previousPose_1.position.y-(*poseIterator).pose.position.y, 2.0));
                Eigen::Vector2d v1(previousPose_1.position.x - previousPose_2.position.x, previousPose_1.position.y - previousPose_2.position.y);
                Eigen::Vector2d v2(-previousPose_2.position.x + (*poseIterator).pose.position.x, -previousPose_2.position.y + (*poseIterator).pose.position.y);
                v1.normalize();
                v2.normalize();
                currentDotProduct = v1.dot(v2);
                if (currentDotProduct < -1.0) currentDotProduct = -1.0;
                if (currentDotProduct > 1.0) currentDotProduct = 1.0;
                absolutAngleChange += fabs(acos(currentDotProduct));
                previousPose_2 = previousPose_1;
                previousPose_1 = poseIterator->pose;
                poseIterator--;

            } while(poseIterator > navigationPath.poses.begin() && length < 1.0);  //consider only the last 1 meter of the path
            double rating = pow(0.6, absolutAngleChange);
            ROS_DEBUG_STREAM("Rating for angle " << panAngle << ": " << rating << " (absolutAngleChange: " << absolutAngleChange << ", length: " << length << ")");
            return rating;
        }
        else
        {
            return 0.0;
        }
    }
    double DefaultIKRatingModule::getPanAngleRating(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition, double sourceRotationBase, double targetRotationBase)
    {
        nav_msgs::Path navigationPath = this->mRobotModel->getNavigationPath(sourcePosition, targetPosition, sourceRotationBase, targetRotationBase);

        if (navigationPath.poses.size() > 2)
        {
            //Get length and angle change of the path
            double length = 0;
            double currentDotProduct;
            double absolutAngleChange = 0;
            std::vector<geometry_msgs::PoseStamped>::iterator poseIterator = navigationPath.poses.end();
            geometry_msgs::Pose previousPose_2 = poseIterator->pose;
            poseIterator--;
            geometry_msgs::Pose previousPose_1 = poseIterator->pose;
            poseIterator--;
            do
            {
                length += sqrt(pow(previousPose_1.position.x-(*poseIterator).pose.position.x, 2.0)+pow(previousPose_1.position.y-(*poseIterator).pose.position.y, 2.0));
                Eigen::Vector2d v1(previousPose_1.position.x - previousPose_2.position.x, previousPose_1.position.y - previousPose_2.position.y);
                Eigen::Vector2d v2(-previousPose_2.position.x + (*poseIterator).pose.position.x, -previousPose_2.position.y + (*poseIterator).pose.position.y);
                v1.normalize();
                v2.normalize();
                currentDotProduct = v1.dot(v2);
                if (currentDotProduct < -1.0) currentDotProduct = -1.0;
                if (currentDotProduct > 1.0) currentDotProduct = 1.0;
                absolutAngleChange += fabs(acos(currentDotProduct));
                previousPose_2 = previousPose_1;
                previousPose_1 = poseIterator->pose;
                poseIterator--;

            } while(poseIterator > navigationPath.poses.begin() && length < 1.0);  //consider only the last 1 meter of the path
            double rating = pow(0.6, absolutAngleChange);
            ROS_DEBUG_STREAM("Rating: " << rating << " (absolutAngleChange: " << absolutAngleChange << ", length: " << length << ")");
            return rating;
        }
        else
        {
            return 0.0;
        }
    }
}

