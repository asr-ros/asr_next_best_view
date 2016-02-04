#pragma once

#include <boost/foreach.hpp>
#include "next_best_view/helper/MapHelper.hpp"
#include "next_best_view/space_sampler/SpaceSampler.hpp"

namespace next_best_view {
    /*!
     * \brief PlaneSubSpaceSampler implements the space sampling in a plane.
     * \author Ralf Schleicher
     * \date 2014
     * \version 1.0
     * \copyright GNU Public License
     */
    class RandomSpaceSampler : public SpaceSampler {
    private:
        MapHelperPtr mMapHelperPtr;
        unsigned int mSampleSize;
    public:
        /*!
         * \brief constructor for CostmapBasedSpaceSampler object
         */
        RandomSpaceSampler(const MapHelperPtr &mapHelperPtr, unsigned int sampleSize) : mMapHelperPtr(mapHelperPtr), mSampleSize(sampleSize) {}

        /*!
         * \brief destructor for CostmapBasedSpaceSampler object
         */
        virtual ~RandomSpaceSampler();

        SamplePointCloudPtr getSampledSpacePointCloud(SimpleVector3 currentSpacePosition = SimpleVector3(), float contractor = 1.0);

        void setHexagonRadius(double radius);

        double getHexagonRadius();
    };

    /*!
     * \brief Definition for the shared pointer type of the class.
     */
    typedef boost::shared_ptr<RandomSpaceSampler> RandomSpaceSamplerPtr;
}


