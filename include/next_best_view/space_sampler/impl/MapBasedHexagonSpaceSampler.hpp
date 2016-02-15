/*
 * MapBasedHexagonSpaceSampler.hpp
 *
 *  Created on: Oct 16, 2014
 *      Author: ralfschleicher
 */

#pragma once

#include <boost/foreach.hpp>
#include "next_best_view/helper/MapHelper.hpp"
#include "next_best_view/space_sampler/SpaceSampler.hpp"

namespace next_best_view {
	/*!
     * \brief MapBasedHexagonSpaceSampler implements the space sampling with a hexagon pattern.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
	class MapBasedHexagonSpaceSampler : public SpaceSampler {
	private:
		MapHelperPtr mMapHelperPtr;
		double mHexagonRadius;
	public:
		/*!
		 * \brief constructor for CostmapBasedSpaceSampler object
		 */
		MapBasedHexagonSpaceSampler(const MapHelperPtr &mapHelperPtr);

		/*!
		 * \brief destructor for CostmapBasedSpaceSampler object
		 */
		virtual ~MapBasedHexagonSpaceSampler();

		SamplePointCloudPtr getSampledSpacePointCloud(SimpleVector3 currentSpacePosition = SimpleVector3(), float contractor = 1.0);

		void setHexagonRadius(double radius);

		double getHexagonRadius();
	};

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<MapBasedHexagonSpaceSampler> MapBasedHexagonSpaceSamplerPtr;
}
