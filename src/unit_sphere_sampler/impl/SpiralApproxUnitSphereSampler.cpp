/*
 * SpiralApproxUnitSphereSampler.hpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/unit_sphere_sampler/impl/SpiralApproxUnitSphereSampler.hpp"
#include "next_best_view/helper/MathHelper.hpp"

namespace next_best_view {
	SpiralApproxUnitSphereSampler::SpiralApproxUnitSphereSampler() : UnitSphereSampler() {

	}

	SpiralApproxUnitSphereSampler::~SpiralApproxUnitSphereSampler() { }

	SimpleQuaternionCollectionPtr SpiralApproxUnitSphereSampler::getSampledUnitSphere() {
		uint32_t sampleCount = this->getSamples();
		return MathHelper::getOrientationsOnUnitSphere(sampleCount);
	}
}
