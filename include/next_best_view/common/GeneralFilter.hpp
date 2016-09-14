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

#pragma once

#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <vector>

#include "next_best_view/common/CommonClass.hpp"
#include "next_best_view/helper/DebugHelper.hpp"

namespace next_best_view {
	class GeneralFilter;
	typedef boost::shared_ptr<GeneralFilter> GeneralFilterPtr;

	class GeneralFilter : public CommonClass {
	private:
		GeneralFilterPtr mPreFilter;
        GeneralFilterPtr mPostFilter;
        DebugHelperPtr mDebugHelperPtr;
	public:
		/*!
		 * Constructor.
		 */
		GeneralFilter() : mPreFilter(), mPostFilter() {
            mDebugHelperPtr = DebugHelper::getInstance();
		}

		/*!
		 * \brief Adding a pre filter to this filter
		 * \param filterPtr the filter pointer
		 */
		void setPreFilter(GeneralFilterPtr &filterPtr) {
			mPreFilter = filterPtr;
		}

		/*!
		 * \brief Adding a post filter to this filter
		 * \param filterPtr the filter pointer
		 */
		void setPostFilter(GeneralFilterPtr &filterPtr) {
			mPostFilter = filterPtr;
		}

		/*!
		 * \brief Does the filtering on the indices
		 * \param indicesPtr the indices
		 */
		virtual void doFiltering(IndicesPtr &indicesPtr) = 0;

		/*!
		 * \brief Applies the filter on the indices
         * \param indicesPtr [out] the indices
		 */
		void filter(IndicesPtr &indicesPtr) {
			// set an empty indices list.
			indicesPtr = IndicesPtr(new Indices());

			// apply pre filter
			if (mPreFilter != GeneralFilterPtr()) {
                mDebugHelperPtr->write("Start pre filtering", DebugHelper::FILTER);
				// set the working input cloud and the relevant indices to work on.
				mPreFilter->setInputCloud(this->getInputCloud());
				mPreFilter->setIndices(this->getIndices());

				// do the filtering
				mPreFilter->doFiltering(indicesPtr);

				// set the indices of the filter to the already filtered indices and reset the return indices pointer.
				this->setIndices(indicesPtr);
				indicesPtr = IndicesPtr(new Indices());
                mDebugHelperPtr->write("Ended pre filtering", DebugHelper::FILTER);
			}

			// apply this filter
			this->doFiltering(indicesPtr);

			// apply post filter
			if (mPostFilter != GeneralFilterPtr()) {
                mDebugHelperPtr->write("Start post filtering", DebugHelper::FILTER);
				// set the working input cloud and the relevant indices to work on.
				mPostFilter->setInputCloud(this->getInputCloud());
				mPostFilter->setIndices(indicesPtr);

				indicesPtr = IndicesPtr(new Indices());
				mPostFilter->doFiltering(indicesPtr);
                mDebugHelperPtr->write("Ended post filtering", DebugHelper::FILTER);
			}
		}
	};
}
