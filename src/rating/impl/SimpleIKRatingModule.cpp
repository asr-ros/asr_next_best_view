/*
 * DefaultIKRatingModule.cpp
 *
 *  Created on: Dez 23, 2015
 *      Author: florianaumann
 */
#include "next_best_view/rating/impl/SimpleIKRatingModule.h"
#include <ros/ros.h>

namespace next_best_view {
    SimpleIKRatingModule::SimpleIKRatingModule() : IKRatingModule() {}
    SimpleIKRatingModule::~SimpleIKRatingModule() { }

    double SimpleIKRatingModule::getPanAngleRating(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition, double sourceRotationBase, double targetRotationBase)
    {
        float angleBetweenPoints = 0.0;
        if (fabs(targetPosition.y - sourcePosition.y) > 0.000001f && fabs(targetPosition.x - sourcePosition.x) > 0.000001f)  //Prevent undefined behavior when distance if close to zero
        {
            angleBetweenPoints = std::atan2(targetPosition.y - sourcePosition.y, targetPosition.x - sourcePosition.x);
            angleBetweenPoints = fmod(angleBetweenPoints, 2.0*M_PI);
        }

        float sourceRotDiff = sourceRotationBase - angleBetweenPoints;
        float targetRotDiff = targetRotationBase - angleBetweenPoints;

        double rotationCosts = std::min(fabs(sourceRotDiff), (float)(2.0f*M_PI-fabs(sourceRotDiff)))
                                + std::min(fabs(targetRotDiff), (float)(2.0f*M_PI-fabs(targetRotDiff)));

        double rating = pow(0.6, rotationCosts);
        return rating;
    }
}

