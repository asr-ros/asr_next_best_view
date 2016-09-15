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

#ifndef SPACESAMPLER_HPP_
#define SPACESAMPLER_HPP_

#include "next_best_view/common/CommonClass.hpp"

//ToDo rename & comment (xtop,xbot,ytop,ybot)

namespace next_best_view {
	/*!
	 * \brief SpaceSampler class generalizes the sampling of the space.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
	class SpaceSampler : public CommonClass {
	private:
		/*!
		 * \brief sample count
		 */
		uint32_t mSamples;
		
	protected:
	  
        double xtop,xbot,ytop,ybot;

	public:
		/*!
		 * \brief constructor for SpaceSampler object
		 */
		SpaceSampler();

		/*!
		 * \brief destructor for SpaceSampler object
		 */
		virtual ~SpaceSampler();

		/*!
		 * \return a point cloud containing a set of points in the space.
		 */
		virtual SamplePointCloudPtr getSampledSpacePointCloud(SimpleVector3 currentSpacePosition, float contractor) = 0;

		/*!
		 * \param samples the count of samples to be created
		 */
		void setSamples(const uint32_t &samples);

		/*!
		 * \return count of samples to be created.
		 */
		uint32_t getSamples();

        double getXtop() const{return this->xtop;}
        double getYtop() const{return this->ytop;}
        double getXbot() const{return this->xbot;}
        double getYbot() const{return this->ybot;}
	};

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<SpaceSampler> SpaceSamplerPtr;
}


#endif /* SPACESAMPLER_HPP_ */
