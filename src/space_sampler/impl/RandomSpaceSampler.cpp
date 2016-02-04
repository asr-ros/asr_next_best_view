#include "next_best_view/space_sampler/impl/RandomSpaceSampler.hpp"

namespace next_best_view {

    SamplePointCloudPtr RandomSpaceSampler::getSampledSpacePointCloud(SimpleVector3 currentSpacePosition, float contractor)
    {
        SamplePointCloudPtr pointCloud = SamplePointCloudPtr(new SamplePointCloud());
        pointCloud->push_back(currentSpacePosition);

        float width = mMapHelperPtr->getMetricWidth() * contractor;
        float height = mMapHelperPtr->getMetricHeight() * contractor;

        float xStart = std::max(0.f, currentSpacePosition[0] - (0.5f * width));
        float xEnd = std::min(mMapHelperPtr->getMetricWidth(), currentSpacePosition[0] + (0.5f * width));

        float yStart = std::max(0.f, currentSpacePosition[1] - (0.5f * height));
        float yEnd = std::min(mMapHelperPtr->getMetricHeight(), currentSpacePosition[1] + (0.5f * height));

        float xDist = xEnd - xStart;
        float yDist = yEnd - yStart;

        while (pointCloud->size() < mSampleSize)
        {
            //random numbers between 0 and 1;
            float xRandom = ((float) std::rand() / (RAND_MAX));
            float yRandom = ((float) std::rand() / (RAND_MAX));

            SimpleVector3 randomPoint(xStart + xRandom * xDist, yStart + yRandom * yDist, 0);

            //make sure randomPoint does not intersect with an obstacle
            int8_t occupancyValue = mMapHelperPtr->getRaytracingMapOccupancyValue(randomPoint);
            if (mMapHelperPtr->isOccupancyValueAcceptable(occupancyValue))
            {
                pointCloud->push_back(randomPoint);
            }
        }

        return pointCloud;


    }
}
