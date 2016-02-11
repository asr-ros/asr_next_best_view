#pragma once

#include "next_best_view/helper/MapHelper.hpp"
#include "next_best_view/space_sampler/SpaceSampler.hpp"

namespace next_best_view {

    class MapBasedRandomSpaceSampler : public SpaceSampler {
    private:
        MapHelperPtr mMapHelperPtr;
        unsigned int mSampleSize;
    public:
        /*!
         * \brief constructor for RandomSpaceSampler object
         */
        MapBasedRandomSpaceSampler(const MapHelperPtr &mapHelperPtr, unsigned int sampleSize) : mMapHelperPtr(mapHelperPtr), mSampleSize(sampleSize) {}

        SamplePointCloudPtr getSampledSpacePointCloud(SimpleVector3 currentSpacePosition = SimpleVector3(), float contractor = 1.0);
    };

    /*!
     * \brief Definition for the shared pointer type of the class.
     */
    typedef boost::shared_ptr<MapBasedRandomSpaceSampler> MapBasedRandomSpaceSamplerPtr;
}


