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

#include "next_best_view/space_sampler/impl/MapBasedRandomSpaceSampler.hpp"

namespace next_best_view {

    SamplePointCloudPtr MapBasedRandomSpaceSampler::getSampledSpacePointCloud(SimpleVector3 currentSpacePosition, float contractor)
    {
        SamplePointCloudPtr pointCloud = SamplePointCloudPtr(new SamplePointCloud());

        SamplePoint currentSamplePoint;
        currentSamplePoint.x = currentSpacePosition[0];
        currentSamplePoint.y = currentSpacePosition[1];
        pointCloud->push_back(currentSamplePoint);

        double width = mMapHelperPtr->getMetricWidth() * contractor;
        double height = mMapHelperPtr->getMetricHeight() * contractor;

        while (pointCloud->size() < mSampleSize)
        {
            //random numbers between -1 and 1;
            double xRandom = ((double) std::rand() / (RAND_MAX)) * 2 - 1;
            double yRandom = ((double) std::rand() / (RAND_MAX)) * 2 - 1;

            SimpleVector3 randomPoint(currentSpacePosition[0] + xRandom * width, currentSpacePosition[1] + yRandom * height, currentSpacePosition[2]);

            //make sure randomPoint does not intersect with an obstacle and is inside the map
            int8_t occupancyValue = mMapHelperPtr->getRaytracingMapOccupancyValue(randomPoint);
            if (mMapHelperPtr->isOccupancyValueAcceptable(occupancyValue))
            {
                SamplePoint samplePoint;
                samplePoint.x = randomPoint[0];
                samplePoint.y = randomPoint[1];
                pointCloud->push_back(samplePoint);
            }
        }

        return pointCloud;


    }
}
