/*
 * CostmapBasedSpaceSampler.cpp
 *
 *  Created on: Oct 16, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/space_sampler/impl/MapBasedHexagonSpaceSampler.hpp"

namespace next_best_view {
	MapBasedHexagonSpaceSampler::MapBasedHexagonSpaceSampler(const MapHelperPtr &mapUtilityPtr) :  mMapHelperPtr(mapUtilityPtr), mHexagonRadius(1.0) {

	}

	MapBasedHexagonSpaceSampler::~MapBasedHexagonSpaceSampler() { }

	SamplePointCloudPtr MapBasedHexagonSpaceSampler::getSampledSpacePointCloud(SimpleVector3 currentSpacePosition, float contractor) {
		SamplePointCloudPtr pointCloud = SamplePointCloudPtr(new SamplePointCloud());
		pointCloud->push_back(currentSpacePosition);

		double width = mMapHelperPtr->getMetricWidth();
		double height = mMapHelperPtr->getMetricHeight();

		//Comment: Black magic
		float radius = pow(contractor,.382) * mHexagonRadius;
		float horizontalSpacing = radius * cos(M_PI / 6.0);
		float interPointSpacing = radius;

		int spanX = ceil(contractor * .5 * width / horizontalSpacing);
		int spanY = ceil(contractor * .25 * height / radius);
		int span = max(spanX, spanY);

        xtop = 0;
        ytop = 0;
        xbot = width;
        ybot = height;

		for (int x = -span; x < span; x++) {
			for (int y = -span; y <= span; y++) {
				double offsetting = abs(y) % 2 == 0 ? horizontalSpacing : 0.0;
                SimpleVector3 upperPoint(x * 2.0 * horizontalSpacing + offsetting, .5 * interPointSpacing + y * 1.5 * radius, 0);
				upperPoint += currentSpacePosition;
				SimpleVector3 lowerPoint(x * 2.0 * horizontalSpacing + offsetting, -.5 * interPointSpacing + y * 1.5 * radius, 0);
				lowerPoint += currentSpacePosition;

				int8_t upperOccupancyValue = mMapHelperPtr->getRaytracingMapOccupancyValue(upperPoint);
				int8_t lowerOccupancyValue = mMapHelperPtr->getRaytracingMapOccupancyValue(lowerPoint);

                if(lowerPoint[0] < xbot) xbot = lowerPoint[0];
                if(lowerPoint[1] < ybot) ybot = lowerPoint[1];
                if(upperPoint[0] > xtop) xtop = upperPoint[0];
                if(upperPoint[1] > ytop) ytop = upperPoint[1];
    /*

                if (x == -span && y == -span)
                {
                    xbot = lowerPoint[0];
                    ybot = lowerPoint[1];
                }

                if (x == span-1 && y == span)
                {
                    xtop = upperPoint[0];
                    ytop = upperPoint[1];
                }*/


                if (mMapHelperPtr->isOccupancyValueAcceptable(upperOccupancyValue)) {
					pointCloud->push_back(upperPoint);
				}

                if (mMapHelperPtr->isOccupancyValueAcceptable(lowerOccupancyValue)) {
					pointCloud->push_back(lowerPoint);
				}
			}
		}

		return pointCloud;
	}

	void MapBasedHexagonSpaceSampler::setHexagonRadius(double radius) {
		mHexagonRadius = radius;
	}

	double MapBasedHexagonSpaceSampler::getHexagonRadius() {
		return mHexagonRadius;
	}
}


