/*
 * CostmapBasedSpaceSampler.hpp
 *
 *  Created on: Oct 16, 2014
 *      Author: ralfschleicher
 */

#ifndef COSTMAPBASEDSPACESAMPLER_HPP_
#define COSTMAPBASEDSPACESAMPLER_HPP_

#include <boost/foreach.hpp>
#include "next_best_view/utility/MapUtility.hpp"
#include "next_best_view/space_sampler/SpaceSampler.hpp"

namespace next_best_view {
	/*!
	 * \brief PlaneSubSpaceSampler implements the space sampling in a plane.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
	class CostmapBasedSpaceSampler : public SpaceSampler {
	private:
		MapUtilityPtr mMapUtilityPtr;
	public:
		/*!
		 * \brief constructor for CostmapBasedSpaceSampler object
		 */
		CostmapBasedSpaceSampler(const MapUtilityPtr &mapUtilityPtr);

		/*!
		 * \brief destructor for CostmapBasedSpaceSampler object
		 */
		virtual ~CostmapBasedSpaceSampler();

		SamplePointCloudPtr getSampledSpacePointCloud(SimpleVector3 currentSpacePosition = SimpleVector3(), float contractor = 1.0);
	};

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<CostmapBasedSpaceSampler> CostmapBasedSpaceSamplerPtr;
}


#endif /* COSTMAPBASEDSPACESAMPLER_HPP_ */
