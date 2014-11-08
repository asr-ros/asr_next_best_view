/*
 * MapBasedSpaceSampler.hpp
 *
 *  Created on: Oct 16, 2014
 *      Author: ralfschleicher
 */

#ifndef MAPBASEDSPACESAMPLER_HPP_
#define MAPBASEDSPACESAMPLER_HPP_

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
	class MapBasedSpaceSampler : public SpaceSampler {
	private:
		MapHelperPtr mMapHelperPtr;
	public:
		/*!
		 * \brief constructor for CostmapBasedSpaceSampler object
		 */
		MapBasedSpaceSampler(const MapHelperPtr &mapHelperPtr);

		/*!
		 * \brief destructor for CostmapBasedSpaceSampler object
		 */
		virtual ~MapBasedSpaceSampler();

		SamplePointCloudPtr getSampledSpacePointCloud(SimpleVector3 currentSpacePosition = SimpleVector3(), float contractor = 1.0);
	};

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<MapBasedSpaceSampler> MapBasedSpaceSamplerPtr;
}


#endif /* MAPBASEDSPACESAMPLER_HPP_ */
