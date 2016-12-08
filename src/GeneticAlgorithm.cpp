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

#include "next_best_view/GeneticAlgorithm.hpp"
#include "boost/range/irange.hpp"
#include <chrono>

namespace next_best_view {

    GeneticAlgorithm::GeneticAlgorithm(NextBestViewCalculatorPtr nbvCalcPtr, const MapHelperPtr &mapHelperPtr, ClusterExtractionPtr clusterExtractionPtr, int improvementIterations, float improvementAngle, float radius, int minIterationGA)
        : mNBVCalcPtr(nbvCalcPtr),
          mMapHelperPtr(mapHelperPtr),
          mClusterExtractionPtr(clusterExtractionPtr),
          mMinIterationGA(minIterationGA) {
        setRotationMatrices(improvementIterations, improvementAngle);
        setPositionOffsets(improvementIterations, radius);
    }

    ViewportPointCloudPtr GeneticAlgorithm::selectAndMutate(const ViewportPointCloudPtr &samples, int iterationStep) {
        auto begin = std::chrono::high_resolution_clock::now();
        auto newViewports = mutate(selection(samples, iterationStep), iterationStep);
        auto finish = std::chrono::high_resolution_clock::now();
        // cast timediff to float in seconds
        ROS_INFO_STREAM("select and mutate took " << std::chrono::duration<float>(finish-begin).count() << " seconds.");
        return newViewports;
    }

    ViewportPointCloudPtr GeneticAlgorithm::selection(const ViewportPointCloudPtr &in, int iterationStep) {
        ViewportPointCloudPtr result(new ViewportPointCloud());
        // most simple selection: we take the best 20 ones
        // TODO: maybe use cluster information to generate per cluster? might be useful for prediction/keep other options
        BoundingBoxPtrsPtr bbs = mClusterExtractionPtr->getClusters();
        std::vector<unsigned int> samplesPerBB(bbs->size());
        for (unsigned int i = 0; i < bbs->size(); i++) {
            samplesPerBB[i] = 0;
        }
        float radius = 0.07 * pow(2, -(iterationStep - mMinIterationGA));
        radius = max(0.07f, radius);
        unsigned int nViewports = 10;// * pow(2, -(iterationStep - mMinIterationGA));
        nViewports = max(static_cast<unsigned int>(1), nViewports);
        BOOST_REVERSE_FOREACH (ViewportPoint &p, *in) {
            int bbIdx = 0;
            bool pHasHypothesisInMissingBB = false;
            bool haveEnoughSamples = true;
            // make sure new viewport is not too close to other selected viewports
            for (ViewportPoint &goodHeuristicViewport : *result) {
                if ((goodHeuristicViewport.getPosition() - p.getPosition()).lpNorm<2>() < radius) {
                    goto nextViewport;
                }
            }
            // check if viewport contains a hypothesis in a bb where we need more samples
            for (BoundingBoxPtr &bbPtr : *bbs) {
                if (samplesPerBB[bbIdx] >= nViewports) {
                    bbIdx ++;
                    continue; // we have enough samples for the bb
                }
                for (int hypothesisIndex : *p.child_indices) {
                    ObjectPoint hypothesis = p.child_point_cloud->at(hypothesisIndex);
                    if (bbPtr->contains(hypothesis.getPosition())) {
                        samplesPerBB[bbIdx] ++;
                        pHasHypothesisInMissingBB = true;
                        break;
                    }
                }
                bbIdx ++;
            }
            if (pHasHypothesisInMissingBB) {
                result->push_back(p);
            }
            // check if we have enough samples and can stop selection
            for (unsigned int i = 0; i < bbs->size(); i++) {
                if (samplesPerBB[i] < nViewports) {
                    haveEnoughSamples = false;
                }
            }
            if (haveEnoughSamples) {
                break;
            }
            nextViewport:;
        }
        return result;
    }

    ViewportPointCloudPtr GeneticAlgorithm::mutate(const ViewportPointCloudPtr &in, int iterationStep) {
        ViewportPointCloudPtr result(new ViewportPointCloud());
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
                    // TODO: determine real hypothesis/use kdtreefilter/is it worth?
//                    IndicesPtr hypothesisNearby;
//                    if (!getFeasibleHypothesis(pos, hypothesisNearby)) {
//                        continue;
//                    }
                    //viewport.child_indices = goodRatedViewportPoint.child_indices;
                    viewport.point_cloud = goodRatedViewportPoint.point_cloud;
                    viewport.object_type_set = goodRatedViewportPoint.object_type_set;
                    viewport.oldIdx = 0;
                    result->push_back(viewport);
                }
            }
        }
        IndicesPtr feasibleMutatedViewportIndices;
        ROS_INFO_STREAM("mNBVCalcPtr: " << mNBVCalcPtr);
        mNBVCalcPtr->getFeasibleViewports(result, feasibleMutatedViewportIndices);
        if (feasibleMutatedViewportIndices->size() != result->size()) {
            ROS_INFO_STREAM("we generated some non feasible mutated viewports");
            result = ViewportPointCloudPtr(new ViewportPointCloud(*result, *feasibleMutatedViewportIndices));
        }
        return result;
    }

    void GeneticAlgorithm::setRotationMatrices(int iterationSteps, double maxAngle) {
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

    void GeneticAlgorithm::setPositionOffsets(int iterationSteps, double radius) {
        // to transform degree to radians, because degree angles are more readable
        float degToRad = M_PI / 180.0;

        // cos (30°), used to generate width of hexagon
        float cos30 = cos(30 * degToRad);

        for (int i : boost::irange(0, iterationSteps)) {
            // we reduce radius exponentially
            double r = pow(2.0, -static_cast<double>(i)) * radius * 2.5;

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
