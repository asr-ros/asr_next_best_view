/*
 * AngleApproximationIKRatingModule.cpp
 *
 *  Created on: Jul 06, 2015
 *      Author: florianaumann
 */
#include "next_best_view/rating/impl/AngleApproximationIKRatingModule.h"
#include <ros/ros.h>

namespace next_best_view {
    AngleApproximationIKRatingModule::AngleApproximationIKRatingModule() : IKRatingModule() {}
    AngleApproximationIKRatingModule::~AngleApproximationIKRatingModule() { }

    double AngleApproximationIKRatingModule::getPanAngleRating(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition, double sourceRotationBase, double targetRotationBase)
    {
        float angleBetweenPoints = 0.0;
        if (fabs(targetPosition.y - sourcePosition.y) > 0.000001f && fabs(targetPosition.x - sourcePosition.x) > 0.000001f)  //Prevent undefined behavior when distance if close to zero
        {
            ROS_INFO_STREAM("Rating for (" << sourcePosition.x << ", " << sourcePosition.y << ") to (" << targetPosition.x << ", " << targetPosition.y << ")");
            angleBetweenPoints = std::atan2(targetPosition.y - sourcePosition.y, targetPosition.x - sourcePosition.x);
            angleBetweenPoints = fmod(angleBetweenPoints- M_PI/2.0, 2.0*M_PI);
            if (angleBetweenPoints <= 0) {angleBetweenPoints += M_PI;}
        }
        ROS_INFO_STREAM("angleBetweenPoints: " << angleBetweenPoints);

        if (sourceRotationBase <= 0) {sourceRotationBase += M_PI;}
        if (targetRotationBase <= 0) {targetRotationBase += M_PI;}
        float sourceRotDiff = fmod(sourceRotationBase - angleBetweenPoints, 2.0*M_PI);
        float targetRotDiff = fmod(targetRotationBase - angleBetweenPoints, 2.0*M_PI);
        if (sourceRotDiff <= 0) {sourceRotDiff += M_PI;}
        if (targetRotDiff <= 0) {targetRotDiff += M_PI;}
        ROS_INFO_STREAM("sourceRotDiff: " << sourceRotDiff);
        ROS_INFO_STREAM("targetRotDiff: " << targetRotDiff);

        double rotationCosts = std::min(fabs(sourceRotDiff), (float)(2.0f*M_PI-fabs(sourceRotDiff)))
                                + std::min(fabs(targetRotDiff), (float)(2.0f*M_PI-fabs(targetRotDiff)));
        ROS_INFO_STREAM("rotationCosts: " << rotationCosts);
        float rotDiff = fmod(targetRotationBase - sourceRotationBase, 2.0*M_PI);
        if (rotDiff <= 0) {rotDiff += 2.0*M_PI;}
        rotDiff = std::min(fabs(rotDiff), (float)(2.0f*M_PI-fabs(rotDiff)));

        double rating = pow(0.6, rotDiff);
        return rating;
    }
}


