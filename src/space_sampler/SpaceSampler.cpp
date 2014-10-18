/*
 * SpaceSampler.cpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/space_sampler/SpaceSampler.hpp"

namespace next_best_view {
	SpaceSampler::SpaceSampler() : mSamples(0) {

	}

	SpaceSampler::~SpaceSampler() { }

	void SpaceSampler::setSamples(const uint32_t &samples) {
		mSamples = samples;
	}

	uint32_t SpaceSampler::getSamples() {
		return mSamples;
	}
}

