#include "next_best_view/space_sampler/impl/RandomSpaceSampler.hpp"

namespace next_best_view {

    SamplePointCloudPtr RandomSpaceSampler::getSampledSpacePointCloud(SimpleVector3 currentSpacePosition, float contractor)
    {
        SamplePointCloudPtr pointCloud = SamplePointCloudPtr(new SamplePointCloud());
        pointCloud->push_back(currentSpacePosition);

        float width = mMapHelperPtr->getMetricWidth() * contractor;
        float height = mMapHelperPtr->getMetricHeight() * contractor;

        float startX = std::max(0.f, currentSpacePosition[0] - (0.5f * width));
        float startY = std::max(0.f, currentSpacePosition[1] - (0.5f * height));

        unsigned int numberPoints = 100;

        while (pointCloud->size() < numberPoints)
        {
            //random numbers between 0 and 1;
            float rX = ((float) std::rand() / (RAND_MAX));
            float rY = ((float) std::rand() / (RAND_MAX));

            float clampedX = std::min(mMapHelperPtr->getMetricWidth(), startX + rX * width);
            float clampedY = std::min(mMapHelperPtr->getMetricHeight(), startY + rY * height);
            SimpleVector3 randomPoint(clampedX, clampedY, 0);

            //check if randomPoint does not intersect with an obstacle
            int8_t occupancyValue = mMapHelperPtr->getRaytracingMapOccupancyValue(randomPoint);
            if (mMapHelperPtr->isOccupancyValueAcceptable(occupancyValue))
            {
                pointCloud->push_back(randomPoint);
            }
        }


    }
}
