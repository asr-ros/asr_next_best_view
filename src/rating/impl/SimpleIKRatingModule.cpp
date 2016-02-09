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
        double absolutAngleChange = fabs(sourceRotationBase - targetRotationBase);
        if (absolutAngleChange > M_PI) {absolutAngleChange = 2.0*M_PI - absolutAngleChange;}
        double rating = pow(0.6, absolutAngleChange);
        return rating;
    }
}

