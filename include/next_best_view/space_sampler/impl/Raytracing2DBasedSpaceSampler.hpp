/*
 * MapBasedSpaceSampler.hpp
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
     * \brief Raytracing2DBasedSpaceSampler
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
	class Raytracing2DBasedSpaceSampler : public SpaceSampler {
	private:
		MapHelperPtr mMapHelperPtr;
	public:
		/*!
		 * \brief constructor for CostmapBasedSpaceSampler object
		 */
		Raytracing2DBasedSpaceSampler(const MapHelperPtr &mapHelperPtr);

		/*!
		 * \brief destructor for CostmapBasedSpaceSampler object
		 */
		virtual ~Raytracing2DBasedSpaceSampler();

		SamplePointCloudPtr getSampledSpacePointCloud(SimpleVector3 currentSpacePosition = SimpleVector3(), float contractor = 1.0);
	};

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<Raytracing2DBasedSpaceSampler> MapBasedSpaceSamplerPtr;
}
