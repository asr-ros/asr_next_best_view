/**

Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Braun Kai, Heller Florian, Hutmacher Robin, Karrenbauer Oliver, Marek Felix, Mayr Matthias, Mehlhaus Jonas, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Stroh Daniel, Trautmann Jeremias, Walter Milena
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

4. The use is explicitly not permitted to any application which deliberately try to kill or do harm to any living creature.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "next_best_view/rating/impl/NavigationPathIKRatingModule.h"
#include <ros/ros.h>

namespace next_best_view {
    NavigationPathIKRatingModule::NavigationPathIKRatingModule(RobotModelPtr robotModel) : IKRatingModule()
    {
        mRobotModel = robotModel;
        mDebugHelperPtr = DebugHelper::getInstance();
    }
    NavigationPathIKRatingModule::~NavigationPathIKRatingModule() { }

    double NavigationPathIKRatingModule::getPanAngleRating(const geometry_msgs::Point &sourcePosition, const geometry_msgs::Point &targetPosition, double sourceRotationBase, double targetRotationBase)
    {
        targetRotationBase = fmod(targetRotationBase + M_PI, 2.0 * M_PI);
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
            mDebugHelperPtr->write(std::stringstream() << "Rating: " << rating
                                    << " (absolutAngleChange: " << absolutAngleChange
                                    << ", length: " << length << ")",
                        DebugHelper::IK_RATING);
            return rating;
        }
        else
        {
            return 0.0;
        }
    }
}

