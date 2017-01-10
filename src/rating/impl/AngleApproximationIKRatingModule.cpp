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
        double rotationCosts = std::min((float)fabs(targetRotDiff), (float)(2.0f*M_PI-fabs(targetRotDiff))) + std::min((float)fabs(sourceRotDiff), (float)(2.0f*M_PI-fabs(sourceRotDiff)));

        double rating = pow(0.6, rotationCosts);
        return rating;
    }
}


