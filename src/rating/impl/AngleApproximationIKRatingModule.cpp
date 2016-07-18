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
            //ROS_INFO_STREAM("Rating for (" << sourcePosition.x << ", " << sourcePosition.y << ") to (" << targetPosition.x << ", " << targetPosition.y << ")");
            angleBetweenPoints = std::atan2(targetPosition.y - sourcePosition.y, targetPosition.x - sourcePosition.x);
        }
        //Make sure, input is between 0 and 2 PI
        if (sourceRotationBase <= 0) {sourceRotationBase += 2.0*M_PI;}
        if (targetRotationBase <= 0) {targetRotationBase += 2.0*M_PI;}
        float sourceRotDiff = fmod(sourceRotationBase - angleBetweenPoints, 2.0*M_PI);
        float targetRotDiff = fmod(targetRotationBase - angleBetweenPoints, 2.0*M_PI);
        //Make sure, rotation difference is between 0 and 2 PI
        if (sourceRotDiff <= 0) {sourceRotDiff += 2.0*M_PI;}
        if (targetRotDiff <= 0) {targetRotDiff += 2.0*M_PI;}

        //Calculate costs, take costs for turning left oder right, depending on which ones better
        double rotationCosts = std::min(fabs(targetRotDiff), (float)(2.0f*M_PI-fabs(targetRotDiff))) + std::min(fabs(sourceRotDiff), (float)(2.0f*M_PI-fabs(sourceRotDiff)));

        double rating = pow(0.6, rotationCosts);
        return rating;
    }
}


