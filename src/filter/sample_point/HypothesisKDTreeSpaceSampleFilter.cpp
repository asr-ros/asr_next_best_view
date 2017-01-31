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

#include "next_best_view/filter/sample_point/HypothesisKDTreeSpaceSampleFilter.hpp"

namespace next_best_view {

    HypothesisKDTreeSpaceSampleFilter::HypothesisKDTreeSpaceSampleFilter(float fcp) : mFcp(fcp) { }

    HypothesisKDTreeSpaceSampleFilter::~HypothesisKDTreeSpaceSampleFilter() { }

    void HypothesisKDTreeSpaceSampleFilter::doFiltering(IndicesPtr &resultIndicesPtr) {
        // if indices is not set we will use all points
        SamplePointCloudPtr samplesPtr = getInputPointCloud();
        IndicesPtr samplesIndicesPtr = getInputPointIndices();

        // go through all samples
        for (int idx : *samplesIndicesPtr) {
            IndicesPtr childIndicesPtr(new Indices());
            if (getFeasibleHypothesis(samplesPtr->at(idx).getSimpleVector3(), childIndicesPtr)) {
                samplesPtr->at(idx).child_indices = childIndicesPtr;
                resultIndicesPtr->push_back(idx);
            }
        }
    }

    bool HypothesisKDTreeSpaceSampleFilter::getFeasibleHypothesis(SimpleVector3 samplePoint, IndicesPtr &resultIndicesPtr) {
        // get max radius by far clipping plane of camera. this marks the limit for the visible object distance.
        double radius = mFcp;

        ObjectPoint comparablePoint(samplePoint);

        // the resulting child indices will be written in here
        IndicesPtr childIndicesPtr(new Indices());
        // we don't need distances, but distances are written in here
        SquaredDistances dismissDistances;

        // this is a radius search - which reduces the complexity level for frustum culling.
        int k = mKdTreePtr->radiusSearch(comparablePoint, radius, *childIndicesPtr, dismissDistances);

        // if there is no result of neighboured points, no need to add this point.
        if (k == 0) {
            return false;
        }

        // set the indices
        resultIndicesPtr = childIndicesPtr;
        return true;
    }

    void HypothesisKDTreeSpaceSampleFilter::setObjectPointCloud(const ObjectPointCloudPtr &pointCloudPtr) {
        CommonClass::setObjectPointCloud(pointCloudPtr);

        mKdTreePtr = KdTreePtr(new KdTree());
        mKdTreePtr->setInputCloud(pointCloudPtr);
    }
}
