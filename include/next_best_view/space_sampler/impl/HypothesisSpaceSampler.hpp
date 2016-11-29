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

#include "next_best_view/helper/MapHelper.hpp"
#include "next_best_view/space_sampler/SpaceSampler.hpp"
#include "next_best_view/space_sampler/SpaceSamplePattern.hpp"
#include "next_best_view/pcl/BoundingBox.hpp"

namespace next_best_view {

    /*!
     * \brief The HypothesisSpaceSampler class generates samples near hypothesis, so it still works if there are no hypothesis nearby.
     * Furthermore it generates more samples near a hypothesis cluster, if there are very few samples near that cluster.
     */
    // TODO: add more parameters to make this class more configureable
    class HypothesisSpaceSampler : public SpaceSampler {
    private:
        MapHelperPtr mMapHelperPtr;
        DebugHelperPtr mDebugHelperPtr;
        // a bounding box over all bounding boxes to determine the area where we create samples
        // it might be a bit more efficient if we sample inside the bounding boxes and filter overlaping samples
        // worst case is a sparse point cloud with far distributed samples
        BoundingBoxPtr mSamplingBB;
        // these might be overlaping
        std::vector<BoundingBoxPtr> mBBs;
        // used to sample in mSamplingBB
        SpaceSamplePatternPtr mSpaceSamplePattern;
        // fcp/used to expand bounding boxes
        double mOffset;

    public:
        HypothesisSpaceSampler(const MapHelperPtr &mapHelperPtr, const SpaceSamplePatternPtr &spaceSamplePattern, double offset = 0.3);

        virtual ~HypothesisSpaceSampler();

        SamplePointCloudPtr getSampledSpacePointCloud(SimpleVector3 currentSpacePosition = SimpleVector3(), float contractor = 1.0);

        SamplePointCloudPtr generateSamples(const BoundingBoxPtr &bb, SimpleVector3 currentSpacePosition, float factor = 1.0);

        void setObjectPointCloud(const ObjectPointCloudPtr &pointCloudPtr);
    };

    /*!
     * \brief Definition for the shared pointer type of the class.
     */
    typedef boost::shared_ptr<HypothesisSpaceSampler> HypothesisSpaceSamplerPtr;
}

