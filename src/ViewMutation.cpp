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

#include "next_best_view/ViewMutation.hpp"
#include "boost/range/irange.hpp"

namespace next_best_view {

    ViewMutation::ViewMutation(const MapHelperPtr &mapHelperPtr, int improvementIterations, float improvementAngle, float radius)
        : mMapHelperPtr(mapHelperPtr) {
        setRotationMatrices(improvementIterations, improvementAngle);
        setPositionOffsets(improvementIterations, radius);
    }

    ViewportPointCloudPtr ViewMutation::selectAndMutate(const ViewportPointCloudPtr &samples, int iterationStep) {
        return mutate(selection(samples, iterationStep), iterationStep);
    }

    ViewportPointCloudPtr ViewMutation::selection(const ViewportPointCloudPtr &in, int iterationStep) {
        ViewportPointCloudPtr result(new ViewportPointCloud());
        // most simple selection: we take the best 20 ones
        // TODO: take fewer samples/let them be closer to each other depending on iterationStep (0.2, 20)
        BOOST_REVERSE_FOREACH (ViewportPoint &p, *in) {
            for (ViewportPoint &goodHeuristicViewport : *result) {
                if ((goodHeuristicViewport.getPosition() - p.getPosition()).lpNorm<2>() < 0.2) {
                    goto nextViewport;
                }
            }
            result->push_back(p);
            if (result->size() >= 20) {
                break;
            }
            nextViewport:;
        }
        return result;
    }

    ViewportPointCloudPtr ViewMutation::mutate(const ViewportPointCloudPtr &in, int iterationStep) {
        ViewportPointCloudPtr result(new ViewportPointCloud());
        // TODO: apply transformations
        std::vector<SimpleVector3> positionOffsets = mPositionOffsetsPerRadius.at(iterationStep % mPositionOffsetsPerRadius.size()).second;
        for (ViewportPoint &goodRatedViewportPoint : *in) {
            // viewport itself, no/identity mutation
            result->push_back(goodRatedViewportPoint);

            // get direction vector and rotationMatrices
            SimpleVector3 dirVector = MathHelper::quatToDir(goodRatedViewportPoint.getSimpleQuaternion());
            std::vector<std::pair<double, std::vector<Eigen::Matrix3f>>> rotationMatricesPerRadius;
            if (dirVector[0] > dirVector[1]) {
                // if x value is greater than y value
                // rotate around y axis
                rotationMatricesPerRadius = mRotationMatricesYAxisPerRadius;
            } else {
                rotationMatricesPerRadius = mRotationMatricesXAxisPerRadius;
            }
            std::vector<Eigen::Matrix3f> rotationMatrices = rotationMatricesPerRadius.at(iterationStep % rotationMatricesPerRadius.size()).second;

            // do the mutation on goodRatedViewport
            for (SimpleVector3 &positionOffset : positionOffsets) {
                SimpleVector3 pos = goodRatedViewportPoint.getPosition() + positionOffset;
                if (!mMapHelperPtr->isOccupancyValueAcceptable(mMapHelperPtr->getRaytracingMapOccupancyValue(pos))) {
                    // if the new position can't be reached we should not keep the mutation
                    continue;
                }

                for (Eigen::Matrix3f &rotationMatrix : rotationMatrices) {
                    SimpleVector3 orientation = rotationMatrix * dirVector;
                    SimpleQuaternion quat = MathHelper::dirToQuat(orientation);

                    // copy viewportPoint, but change quat
                    ViewportPoint viewport(pos, quat);
                    // TODO: determine real hypothesis/use kdtreefilter
//                    IndicesPtr hypothesisNearby;
//                    if (!getFeasibleHypothesis(pos, hypothesisNearby)) {
//                        continue;
//                    }
                    viewport.child_indices = goodRatedViewportPoint.child_indices;
                    viewport.point_cloud = goodRatedViewportPoint.point_cloud;
                    viewport.object_type_set = goodRatedViewportPoint.object_type_set;
                    viewport.oldIdx = 0;
                    result->push_back(viewport);
                }
            }
        }
        return result;
    }

    void ViewMutation::setRotationMatrices(int iterationSteps, double maxAngle) {
        SimpleVector3 xAxis(1.0, 0.0, 0.0);
        SimpleVector3 yAxis(0.0, 1.0, 0.0);
        SimpleVector3 up   (0.0, 0.0, 1.0);

        // to transform degree to radians, because degree angles are more readable
        float degToRad = M_PI / 180.0;

        // cos (30°), used to generate width of hexagon
        float cos30 = cos(30 * degToRad);

        // generate rotationMatrices per radius
        for (int i : boost::irange(0, iterationSteps)) {
            // we reduce radius exponentially
            double r = pow(2.0, -static_cast<double>(i)) * maxAngle;

            // width of hexagon with radius r
            float horizontalSpacing = cos30 * r;
            // up is horizontal rotation/movement (x value)
            // ortho is vertical rotation/movement (y value)
            // this creates rotation matrices that rotate 2d around 2 axis
            // the angles per axis are based on the (x,y) position values of a hexagon
            std::vector<Eigen::Matrix3f> rotationMatrices;
            rotationMatrices.push_back(Eigen::AngleAxisf( r * degToRad, xAxis).toRotationMatrix());
            rotationMatrices.push_back(Eigen::AngleAxisf(-r * degToRad, xAxis).toRotationMatrix());
            rotationMatrices.push_back((Eigen::AngleAxisf( r / 2.0 * degToRad, xAxis) * Eigen::AngleAxisf( horizontalSpacing * degToRad, up)).toRotationMatrix());
            rotationMatrices.push_back((Eigen::AngleAxisf(-r / 2.0 * degToRad, xAxis) * Eigen::AngleAxisf( horizontalSpacing * degToRad, up)).toRotationMatrix());
            rotationMatrices.push_back((Eigen::AngleAxisf( r / 2.0 * degToRad, xAxis) * Eigen::AngleAxisf(-horizontalSpacing * degToRad, up)).toRotationMatrix());
            rotationMatrices.push_back((Eigen::AngleAxisf(-r / 2.0 * degToRad, xAxis) * Eigen::AngleAxisf(-horizontalSpacing * degToRad, up)).toRotationMatrix());
            mRotationMatricesXAxisPerRadius.push_back(std::make_pair(r, rotationMatrices));

            rotationMatrices.clear();
            rotationMatrices.push_back(Eigen::AngleAxisf( r * degToRad, yAxis).toRotationMatrix());
            rotationMatrices.push_back(Eigen::AngleAxisf(-r * degToRad, yAxis).toRotationMatrix());
            rotationMatrices.push_back((Eigen::AngleAxisf( r / 2.0 * degToRad, yAxis) * Eigen::AngleAxisf( horizontalSpacing * degToRad, up)).toRotationMatrix());
            rotationMatrices.push_back((Eigen::AngleAxisf(-r / 2.0 * degToRad, yAxis) * Eigen::AngleAxisf( horizontalSpacing * degToRad, up)).toRotationMatrix());
            rotationMatrices.push_back((Eigen::AngleAxisf( r / 2.0 * degToRad, yAxis) * Eigen::AngleAxisf(-horizontalSpacing * degToRad, up)).toRotationMatrix());
            rotationMatrices.push_back((Eigen::AngleAxisf(-r / 2.0 * degToRad, yAxis) * Eigen::AngleAxisf(-horizontalSpacing * degToRad, up)).toRotationMatrix());
            mRotationMatricesYAxisPerRadius.push_back(std::make_pair(r, rotationMatrices));
        }
    }

    void ViewMutation::setPositionOffsets(int iterationSteps, double radius) {
        // to transform degree to radians, because degree angles are more readable
        float degToRad = M_PI / 180.0;

        // cos (30°), used to generate width of hexagon
        float cos30 = cos(30 * degToRad);

        for (int i : boost::irange(0, iterationSteps)) {
            // we reduce radius exponentially
            double r = pow(2.0, -static_cast<double>(i)) * radius;

            // width of hexagon with radius r
            float horizontalSpacing = cos30 * r;

            // up is horizontal rotation/movement (x value)
            // ortho is vertical rotation/movement (y value)
            // this creates rotation matrices that rotate 2d around 2 axis
            // the angles per axis are based on the (x,y) position values of a hexagon
            std::vector<SimpleVector3> positionOffsets;
            positionOffsets.push_back(SimpleVector3(0,  r, 0));
            positionOffsets.push_back(SimpleVector3(0, -r, 0));
            positionOffsets.push_back(SimpleVector3( horizontalSpacing,  r / 2.0, 0));
            positionOffsets.push_back(SimpleVector3( horizontalSpacing, -r / 2.0, 0));
            positionOffsets.push_back(SimpleVector3(-horizontalSpacing,  r / 2.0, 0));
            positionOffsets.push_back(SimpleVector3(-horizontalSpacing, -r / 2.0, 0));

            mPositionOffsetsPerRadius.push_back(std::make_pair(r, positionOffsets));
        }
    }
}
