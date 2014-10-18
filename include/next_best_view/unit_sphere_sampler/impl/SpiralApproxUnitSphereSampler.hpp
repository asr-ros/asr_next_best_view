/*
 * SpiralApproxUnitSphereSampler.hpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#ifndef SPIRALAPPROXUNITSPHERESAMPLER_HPP_
#define SPIRALAPPROXUNITSPHERESAMPLER_HPP_

#include "next_best_view/unit_sphere_sampler/UnitSphereSampler.hpp"

namespace next_best_view {
	/*!
	 * \brief SpiralApproxUnitSphereSampler samples a unit sphere in a spiral manner.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
	class SpiralApproxUnitSphereSampler : public UnitSphereSampler {
	public:
		/*!
		 * \brief constructor SpiralApproxUnitSphereSampler object
		 */
		SpiralApproxUnitSphereSampler();

		/*!
		 * \brief destructor SpiralApproxUnitSphereSampler object
		 */
		virtual ~SpiralApproxUnitSphereSampler();

		SimpleQuaternionCollectionPtr getSampledUnitSphere();
	};

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<SpiralApproxUnitSphereSampler> SpiralApproxUnitSphereSamplerPtr;
}


#endif /* SPIRALAPPROXUNITSPHERESAMPLER_HPP_ */
