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

#include "next_best_view/space_sampler/impl/MapBasedHexagonSpaceSampler.hpp"

namespace next_best_view {
	MapBasedHexagonSpaceSampler::MapBasedHexagonSpaceSampler(const MapHelperPtr &mapUtilityPtr) :  mMapHelperPtr(mapUtilityPtr), mHexagonRadius(1.0) {
        mDebugHelperPtr = DebugHelper::getInstance();
	}

	MapBasedHexagonSpaceSampler::~MapBasedHexagonSpaceSampler() { }

    SamplePointCloudPtr MapBasedHexagonSpaceSampler::getSampledSpacePointCloud(SimpleVector3 currentSpacePosition, float contractor) {
        SamplePointCloudPtr pointCloud = SamplePointCloudPtr(new SamplePointCloud());

        SamplePoint currentSamplePoint;
        currentSamplePoint.x = currentSpacePosition[0];
        currentSamplePoint.y = currentSpacePosition[1];
        pointCloud->push_back(currentSamplePoint);

    mDebugHelperPtr->write(std::stringstream() << "MapBasedHexagonSpaceSampler::getSampledSpacePointCloud contractor " << contractor,
                DebugHelper::SPACE_SAMPLER);
	
	double width = mMapHelperPtr->getMetricWidth();
	double height = mMapHelperPtr->getMetricHeight();

    // calculate radius and horizontalSpacing of a hexagon
	float radius = pow(contractor,.382) * mHexagonRadius;
	float horizontalSpacing = radius * cos(M_PI / 6.0);
	float interPointSpacing = radius;

    // calculate width/height of sampling depending on contractor
	int spanX = ceil(contractor * .5 * width / horizontalSpacing);
	int spanY = ceil(contractor * .25 * height / radius);
	int span = std::max(spanX, spanY);

    // bounding box of all samples
        xtop = 0;
        ytop = 0;
        xbot = width;
        ybot = height;

	for (int x = -span; x < span; x++) {
	  for (int y = -span; y <= span; y++) {
         /*
         every second row we add horizontalSpacing/offsetting to our x Value so we get the following pattern:
         x   x   x   x
           x   x   x
         x   x   x   x
         while each "x" consists of 2 points, the most upper and the most lower point of a hexagon (this hexagon is rotated by 90° clockwise):
                           ______________
                          /       |      \
                         /        |       \
                        /         |hs      \   hs = horizontalSpacing ^= length of half width
                       /          |         \
                      /           |          \
         lower Point /            |           \ most upper Point
                     \                        /
                      \                      /
                       \                    /
                        \                  /
                         \                /
                          \______________/


          1       1       1       1       upper 1



            _/2\_   _/2\_   _/2\_         upper 2
          1/     \1/     \1/     \1       lower 1
          |       |       |       |
          |       |       |       |
          3\_   _/3\_   _/3\_   _/3       upper 3
             \2/     \2/     \2/          lower 2
              |       |       |
              |       |       |
              4\_   _/4\_   _/4
          3      \3/     \3/      3       lower 3

         1, 2, 3, 4 are the vertical sample points if we iterate over y with fixed x, the horizontal ones are iterated over x with a fixed y
        */
	    double offsetting = abs(y) % 2 == 0 ? horizontalSpacing : 0.0;
        // upperPoint
	    SimpleVector3 upperPoint(x * 2.0 * horizontalSpacing + offsetting, .5 * interPointSpacing + y * 1.5 * radius, 0);
	    upperPoint[0] += currentSpacePosition[0];
	    upperPoint[1] += currentSpacePosition[1];

        // lower Point
	    SimpleVector3 lowerPoint(x * 2.0 * horizontalSpacing + offsetting, -.5 * interPointSpacing + y * 1.5 * radius, 0);
	    lowerPoint[0] += currentSpacePosition[0];
	    lowerPoint[1] += currentSpacePosition[1];

        // get occupancy value of upper/lower point
	    int8_t upperOccupancyValue = mMapHelperPtr->getRaytracingMapOccupancyValue(upperPoint);
	    int8_t lowerOccupancyValue = mMapHelperPtr->getRaytracingMapOccupancyValue(lowerPoint);

        // update bounding box
	    if(lowerPoint[0] < xbot) xbot = lowerPoint[0];
	    if(lowerPoint[1] < ybot) ybot = lowerPoint[1];
	    if(upperPoint[0] > xtop) xtop = upperPoint[0];
	    if(upperPoint[1] > ytop) ytop = upperPoint[1];

        // check for occupancy value and add to pointcloud
	    if (mMapHelperPtr->isOccupancyValueAcceptable(upperOccupancyValue)) {
          SamplePoint samplePoint(upperPoint);
	      pointCloud->push_back(samplePoint);
	    }

	    if (mMapHelperPtr->isOccupancyValueAcceptable(lowerOccupancyValue)) {
          SamplePoint samplePoint(lowerPoint);
	      pointCloud->push_back(samplePoint);
	    }
	  }
	}
    mDebugHelperPtr->write(std::stringstream() << "MapBasedHexagonSpaceSampler::size " << pointCloud->size(),
                DebugHelper::SPACE_SAMPLER);
	return pointCloud;
    }

  void MapBasedHexagonSpaceSampler::setHexagonRadius(double radius) {
    mHexagonRadius = radius;
  }

  double MapBasedHexagonSpaceSampler::getHexagonRadius() {
    return mHexagonRadius;
  }
}


