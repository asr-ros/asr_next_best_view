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

#include "next_best_view/space_sampler/impl/HexagonSpaceSamplePattern.hpp" 


namespace next_best_view {

    HexagonSpaceSamplePattern::HexagonSpaceSamplePattern() : mPatternSizeFactor(1.0) { }

    HexagonSpaceSamplePattern::~HexagonSpaceSamplePattern() { }

    SamplePointCloudPtr HexagonSpaceSamplePattern::getSampledSpacePointCloud(SimpleVector3 currentSpacePosition, const BoundingBoxPtr &bb) {
        SimpleVector3 startPos = currentSpacePosition;
        // find startposition near center of bb, if currentSpacePosition is not in bb
        if (!bb->contains(currentSpacePosition)) {
            SimpleVector3 centerPos = (bb->minPos + bb->maxPos) * 0.5;
            SimpleVector3 offset = (centerPos - currentSpacePosition);
            int xFactor = ceil(offset[0] / getPatternWidth());
            int yFactor = ceil(offset[1] / getPatternHeight());
            startPos = currentSpacePosition + SimpleVector3(xFactor * getPatternWidth(), yFactor * getPatternHeight(), 0);
        }

        // find maximum distance from startPos to bb border
        SimpleVector3 diff1 = bb->maxPos - startPos;
        SimpleVector3 diff2 = startPos - bb->minPos;
        float maxHalfWidth = max(diff1[0], diff2[0]);
        float maxHalfHeight = max(diff1[1], diff2[1]);

        // max number of patterns between startPos and border
        int spanX = ceil(maxHalfWidth / getPatternWidth()) + 1;
        int spanY = ceil(maxHalfHeight / getPatternHeight()) + 1;
        float horizontalSpacing = mRadius * cos(M_PI / 6.0); // patternWidth
        float interPointSpacing = mRadius; // patternHeight

        SamplePointCloudPtr samples(new SamplePointCloud());
        // spanX * patternwidth > bb.xWidth, so we sample a bit more that the actual bb
        for (int x = -spanX; x <= spanX; x++) {
            for (int y = -spanY; y <= spanY; y++) {
                double offsetting = abs(y) % 2 == 0 ? horizontalSpacing : 0.0;
                // upperPoint
                SimpleVector3 upperPoint(x * 2.0 * horizontalSpacing + offsetting, .5 * interPointSpacing + y * 1.5 * mRadius, 0);
                upperPoint[0] += startPos[0];
                upperPoint[1] += startPos[1];

                // lower Point
                SimpleVector3 lowerPoint(x * 2.0 * horizontalSpacing + offsetting, -.5 * interPointSpacing + y * 1.5 * mRadius, 0);
                lowerPoint[0] += startPos[0];
                lowerPoint[1] += startPos[1];

                // check if in bb and add to pointcloud
                if (bb->contains(upperPoint)) {
                    samples->push_back(SamplePoint(upperPoint));
                }

                if (bb->contains(lowerPoint)) {
                    samples->push_back(SamplePoint(lowerPoint));
                }
            }
        }
        return samples;
    }

    float HexagonSpaceSamplePattern::getPatternWidth() {
        return mHexagonWidth * mPatternSizeFactor;
    }

    float HexagonSpaceSamplePattern::getPatternHeight() {
        return mRadius * 2.0 * mPatternSizeFactor;
    }

    float HexagonSpaceSamplePattern::getRadius() const {
        return mRadius;
    }

    void HexagonSpaceSamplePattern::setRadius(float radius) {
        mRadius = radius;
        mHexagonWidth = radius * cos(M_PI / 6.0) * 2.0;
        setPatternSizeFactor(1.0);
    }

    float HexagonSpaceSamplePattern::getPatternSizeFactor() {
        return mPatternSizeFactor;
    }

    void HexagonSpaceSamplePattern::setPatternSizeFactor(float factor) {
        mPatternSizeFactor = factor;
    }
}
