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

#pragma once

#include "typedef.hpp"
#include "next_best_view/NextBestViewCalculator.hpp"
#include "next_best_view/NextBestViewCache.hpp"
#include "next_best_view/helper/MapHelper.hpp"
#include "next_best_view/cluster/ClusterExtraction.hpp"

#include "next_best_view/space_sampler/impl/HexagonSpaceSamplePattern.hpp"

namespace next_best_view {

    class NextBestViewCalculator;
    typedef boost::shared_ptr<NextBestViewCalculator> NextBestViewCalculatorPtr;

    class GeneticAlgorithm {
    private:
        // pair<double, std::vector<Eigen::matrix3f>> is the radius in angle (double) and a number of rotations around radius
        std::vector<std::pair<double, std::vector<SimpleVector3>>> mPositionOffsetsPerRadius;
        NextBestViewCachePtr mNBVCachePtr;
        MapHelperPtr mMapHelperPtr;
        ClusterExtractionPtr mClusterExtractionPtr;
        int mImprovementIterationSteps;
        float mMaxAngle;
        float mRadius;
        int mMinIterationGA;
        int mNViewportsPerBB;


    public:
        GeneticAlgorithm(NextBestViewCachePtr nbvCachePtr, const MapHelperPtr &mapHelperPtr,  ClusterExtractionPtr clusterExtractionPtr, int improvementIterations, float improvementAngle, float radius, int minIterationGA, int nViewportsPerBB);

        ViewportPointCloudPtr selectAndMutate(const ViewportPointCloudPtr &samples, int iterationStep);

        float getMaxAngle() const;

        void setMaxAngle(float maxAngle);

        float getRadius() const;

        void setRadius(float radius);

        int getImprovementIterationSteps() const;

        void setImprovementIterationSteps(int iterationSteps);

        int getNViewportsPerBB() const;

        void setNViewportsPerBB(int value);

    private:
        ViewportPointCloudPtr select(const ViewportPointCloudPtr &in, int iterationStep);

        ViewportPointCloudPtr mutate(const ViewportPointCloudPtr &in, int iterationStep);

        /**
         * @brief selectFromSortedViewports this method handles in as a sorted vector, without knowing the sorting function.
         * @param in the sorted samples to select from
         * @param iterationStep used to converge faster
         * @return
         */
        ViewportPointCloudPtr selectFromSortedViewports(const ViewportPointCloudPtr &in, int iterationStep);

        std::vector<SimpleQuaternion> calculateRotationTransformations(SimpleVector3 dirVector, int iterationStep);

        void setPositionOffsets();
    };
    typedef boost::shared_ptr<GeneticAlgorithm> GeneticAlgorithmPtr;
}

