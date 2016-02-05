#include "next_best_view/space_sampler/impl/RandomSpaceSampler.hpp"

namespace next_best_view {

    SamplePointCloudPtr RandomSpaceSampler::getSampledSpacePointCloud(SimpleVector3 currentSpacePosition, float contractor)
    {
        SamplePointCloudPtr pointCloud = SamplePointCloudPtr(new SamplePointCloud());
        pointCloud->push_back(currentSpacePosition);

        double width = mMapHelperPtr->getMetricWidth() * contractor;
        double height = mMapHelperPtr->getMetricHeight() * contractor;

        double xStart = std::max(0., currentSpacePosition[0] - (0.5 * width));
        double xEnd = std::min((double) mMapHelperPtr->getMetricWidth(), currentSpacePosition[0] + (0.5 * width));

        double yStart = std::max(0., currentSpacePosition[1] - (0.5 * height));
        double yEnd = std::min((double) mMapHelperPtr->getMetricHeight(), currentSpacePosition[1] + (0.5 * height));

        double xDist = xEnd - xStart;
        double yDist = yEnd - yStart;

        while (pointCloud->size() < mSampleSize)
        {
            //random numbers between 0 and 1;
            double xRandom = ((double) std::rand() / (RAND_MAX));
            double yRandom = ((double) std::rand() / (RAND_MAX));

            SimpleVector3 randomPoint(xStart + xRandom * xDist, yStart + yRandom * yDist, 0.);

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
