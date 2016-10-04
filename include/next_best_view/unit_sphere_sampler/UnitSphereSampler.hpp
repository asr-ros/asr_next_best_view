/**

Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Heller Florian, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Stroh Daniel, Trautmann Jeremias, Walter Milena, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

4. The use is explicitly not permitted to any application which deliberately try to kill or do harm to any living creature.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef UNITSPHERESAMPLER_HPP_
#define UNITSPHERESAMPLER_HPP_

#include "next_best_view/common/CommonClass.hpp"
#include "typedef.hpp"

namespace next_best_view {
	/*!
	 * \brief UnitSphereSampler samples a unit sphere.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
	class UnitSphereSampler : public CommonClass {
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
