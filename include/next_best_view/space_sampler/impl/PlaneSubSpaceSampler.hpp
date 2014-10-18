/*
 * PlaneSubSpaceSampler.hpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#ifndef PLANESUBSPACESAMPLER_HPP_
#define PLANESUBSPACESAMPLER_HPP_

#include <boost/foreach.hpp>
#include "next_best_view/space_sampler/SpaceSampler.hpp"

namespace next_best_view {
	/*!
	 * \brief PlaneSubSpaceSampler implements the space sampling in a plane.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
	class PlaneSubSpaceSampler : public SpaceSampler {
	public:
		/*!
		 * \brief constructor for PlaneSubSpaceSampler object
		 */
		PlaneSubSpaceSampler();

		/*!
		 * \brief destructor for PlaneSubSpaceSampler object
		 */
		virtual ~PlaneSubSpaceSampler();

		SamplePointCloudPtr getSampledSpacePointCloud(SimpleVector3 currentSpacePosition = SimpleVector3(), float contractor = 1.0);
	};

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<PlaneSubSpaceSampler> PlaneSubSpaceSamplerPtr;
}


#endif /* PLANESUBSPACESAMPLER_HPP_ */
