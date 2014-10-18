/*
 * SphereSampler.hpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#ifndef UNITSPHERESAMPLER_HPP_
#define UNITSPHERESAMPLER_HPP_

#include "typedef.hpp"

namespace next_best_view {
	/*!
	 * \brief UnitSphereSampler samples a unit sphere.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
	class UnitSphereSampler {
	protected:
		/*!
		 * \brief sample count.
		 */
		uint32_t mSamples;
	public:
		/*!
		 * \brief constructor for UnitSphereSampler object
		 * \param samples the cound of camples.
		 */
		UnitSphereSampler(const uint32_t &samples = 0);

		/*!
		 * \brief destructor for UnitSphereSampler object.
		 */
		virtual ~UnitSphereSampler();

		/*!
		 * \return the sampled unit sphere
		 */
		virtual SimpleQuaternionCollectionPtr getSampledUnitSphere() = 0;

		/*!
		 * \brief sets the samples made from the unit sphere
		 * \param samples the number of samples
		 */
		void setSamples(const uint32_t &samples);

		/**
		 * \return the number of samples.
		 */
		uint32_t getSamples();
	};

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<UnitSphereSampler> UnitSphereSamplerPtr;
}

#endif /* UNITSPHERESAMPLER_HPP_ */
