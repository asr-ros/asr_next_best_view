#include "next_best_view/space_sampler/impl/MapBasedRandomSpaceSampler.hpp"

namespace next_best_view {

    SamplePointCloudPtr MapBasedRandomSpaceSampler::getSampledSpacePointCloud(SimpleVector3 currentSpacePosition, float contractor)
    {
        SamplePointCloudPtr pointCloud = SamplePointCloudPtr(new SamplePointCloud());
        pointCloud->push_back(currentSpacePosition);

        double width = mMapHelperPtr->getMetricWidth() * contractor;
        double height = mMapHelperPtr->getMetricHeight() * contractor;

        while (pointCloud->size() < mSampleSize)
        {
            //random numbers between -1 and 1;
            double xRandom = ((double) std::rand() / (RAND_MAX)) * 2 - 1;
            double yRandom = ((double) std::rand() / (RAND_MAX)) * 2 - 1;

            SimpleVector3 randomPoint(currentSpacePosition[0] + xRandom * width, currentSpacePosition[1] + yRandom * height, 0.);

            //make sure randomPoint does not intersect with an obstacle and is inside the map
            int8_t occupancyValue = mMapHelperPtr->getRaytracingMapOccupancyValue(randomPoint);
            if (mMapHelperPtr->isOccupancyValueAcceptable(occupancyValue))
            {
                pointCloud->push_back(randomPoint);
            }
        }

        return pointCloud;


    }
}
