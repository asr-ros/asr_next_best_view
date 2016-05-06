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

        SamplePoint currentSamplePoint;
        currentSamplePoint.x = currentSpacePosition[0];
        currentSamplePoint.y = currentSpacePosition[1];
        pointCloud->push_back(currentSamplePoint);

	ROS_DEBUG_STREAM("MapBasedHexagonSpaceSampler::getSampledSpacePointCloud contractor" << contractor);
	
	double width = mMapHelperPtr->getMetricWidth();
	double height = mMapHelperPtr->getMetricHeight();

	//Comment: Black magic
	float radius = pow(contractor,.382) * mHexagonRadius;
	float horizontalSpacing = radius * cos(M_PI / 6.0);
	float interPointSpacing = radius;

	int spanX = ceil(contractor * .5 * width / horizontalSpacing);
	int spanY = ceil(contractor * .25 * height / radius);
	int span = std::max(spanX, spanY);

        xtop = 0;
        ytop = 0;
        xbot = width;
        ybot = height;

	for (int x = -span; x < span; x++) {
	  for (int y = -span; y <= span; y++) {
	    double offsetting = abs(y) % 2 == 0 ? horizontalSpacing : 0.0;
	    SimpleVector3 upperPoint(x * 2.0 * horizontalSpacing + offsetting, .5 * interPointSpacing + y * 1.5 * radius, 0);
	    upperPoint[0] += currentSpacePosition[0];
	    upperPoint[1] += currentSpacePosition[1];

	    SimpleVector3 lowerPoint(x * 2.0 * horizontalSpacing + offsetting, -.5 * interPointSpacing + y * 1.5 * radius, 0);
	    lowerPoint[0] += currentSpacePosition[0];
	    lowerPoint[1] += currentSpacePosition[1];

	    int8_t upperOccupancyValue = mMapHelperPtr->getRaytracingMapOccupancyValue(upperPoint);
	    int8_t lowerOccupancyValue = mMapHelperPtr->getRaytracingMapOccupancyValue(lowerPoint);

	    if(lowerPoint[0] < xbot) xbot = lowerPoint[0];
	    if(lowerPoint[1] < ybot) ybot = lowerPoint[1];
	    if(upperPoint[0] > xtop) xtop = upperPoint[0];
	    if(upperPoint[1] > ytop) ytop = upperPoint[1];

	    if (mMapHelperPtr->isOccupancyValueAcceptable(upperOccupancyValue)) {
	      SamplePoint samplePoint;
	      samplePoint.x = upperPoint[0];
	      samplePoint.y = upperPoint[1];
	      pointCloud->push_back(samplePoint);
	    }

	    if (mMapHelperPtr->isOccupancyValueAcceptable(lowerOccupancyValue)) {
	      SamplePoint samplePoint;
	      samplePoint.x = lowerPoint[0];
	      samplePoint.y = lowerPoint[1];
	      pointCloud->push_back(samplePoint);
	    }
	  }
	}
	ROS_DEBUG_STREAM("MapBasedHexagonSpaceSampler::size" << pointCloud->size());
	return pointCloud;
    }

  void MapBasedHexagonSpaceSampler::setHexagonRadius(double radius) {
    mHexagonRadius = radius;
  }

  double MapBasedHexagonSpaceSampler::getHexagonRadius() {
    return mHexagonRadius;
  }
}


