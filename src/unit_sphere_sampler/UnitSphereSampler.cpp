/*
 * UnitSphereSampler.cpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/unit_sphere_sampler/UnitSphereSampler.hpp"

namespace next_best_view {
	UnitSphereSampler::UnitSphereSampler(const uint32_t &samples) : mSamples(samples) {
	}

	UnitSphereSampler::~UnitSphereSampler() { }

	void UnitSphereSampler::setSamples(const uint32_t &samples) {
		mSamples = samples;
	}

	uint32_t UnitSphereSampler::getSamples() {
		return mSamples;
	}
}
