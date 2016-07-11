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
        float rotDiff = fmod(targetRotationBase - sourceRotationBase, 2.0*M_PI);
        if (rotDiff <= 0) {rotDiff += 2.0*M_PI;}
        rotDiff = std::min(fabs(rotDiff), (float)(2.0f*M_PI-fabs(rotDiff)));

        double rating = pow(0.6, rotDiff);
        return rating;
    }
}

