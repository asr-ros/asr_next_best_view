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
    template<class T>
	class GeneralFilter;
    template<class T>
    using GeneralFilterPtr = boost::shared_ptr<GeneralFilter<T>>;

    template<class T>
	class GeneralFilter : public CommonClass {
	private:
        GeneralFilterPtr<T> mPreFilter;
        GeneralFilterPtr<T> mPostFilter;
        DebugHelperPtr mDebugHelperPtr;
        bool enabled;

        /*!
         * \brief shared pointer to the point cloud that should be filtered.
         */
        PointCloudPtr<T> mInputPointCloudPtr;

        /*!
         * \brief shared pointer to the active indices of the former declared point cloud.
         */
        IndicesPtr mInputPointIndicesPtr;

	public:
		/*!
		 * Constructor.
		 */
        GeneralFilter() : mPreFilter(), mPostFilter(), enabled(true) {
            mDebugHelperPtr = DebugHelper::getInstance();
		}

        /*!
         * \brief setting the input cloud.
         * \param pointCloudPtr the shared pointer to point cloud
         */
        virtual void setInputPointCloud(const PointCloudPtr<T> &pointCloudPtr) {
            mInputPointCloudPtr = pointCloudPtr;
        }

        /*!
         * \return the shared pointer to the point cloud.
         */
        virtual PointCloudPtr<T>& getInputPointCloud() {
            return mInputPointCloudPtr;
        }

        /*!
         * \brief setting the shared pointer to the active indices of the point cloud.
         * \param indicesPtr the shared pointer to indices.
         */
        void setInputPointIndices(const IndicesPtr &indicesPtr) {
            mInputPointIndicesPtr = indicesPtr;
        }

        /*!
         * \return the shared pointer to the active indices of the point cloud.
         */
        IndicesPtr& getInputPointIndices() {
            return mInputPointIndicesPtr;
        }

		/*!
		 * \brief Adding a pre filter to this filter
		 * \param filterPtr the filter pointer
		 */
        void setPreFilter(GeneralFilterPtr<T> &filterPtr) {
			mPreFilter = filterPtr;
		}

		/*!
		 * \brief Adding a post filter to this filter
		 * \param filterPtr the filter pointer
		 */
        void setPostFilter(GeneralFilterPtr<T> &filterPtr) {
			mPostFilter = filterPtr;
		}

        void enable() {
            enabled = true;
        }

        void disable() {
            enabled = false;
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
            if (mPreFilter != GeneralFilterPtr<T>()) {
                mDebugHelperPtr->write("Start pre filtering", DebugHelper::FILTER);
                // set the working input cloud and the relevant indices to work on.
                mPreFilter->setObjectPointCloud(this->getObjectPointCloud());
                mPreFilter->setObjectPointIndices(this->getObjectPointIndices());
                mPreFilter->setInputPointCloud(this->getInputPointCloud());
                mPreFilter->setInputPointIndices(this->getInputPointIndices());


				// do the filtering
                mPreFilter->filter(indicesPtr);

				// set the indices of the filter to the already filtered indices and reset the return indices pointer.
                this->setInputPointIndices(indicesPtr);
				indicesPtr = IndicesPtr(new Indices());
                mDebugHelperPtr->write("Ended pre filtering", DebugHelper::FILTER);
			}

			// apply this filter
            if (enabled) {
                this->doFiltering(indicesPtr);
            } else {
                // we didn't filter anything, so input = output
                indicesPtr = IndicesPtr(new Indices(*getInputPointIndices()));
            }

			// apply post filter
            if (mPostFilter != GeneralFilterPtr<T>()) {
                mDebugHelperPtr->write("Start post filtering", DebugHelper::FILTER);
				// set the working input cloud and the relevant indices to work on.
                mPostFilter->setObjectPointCloud(this->getObjectPointCloud());
                mPostFilter->setObjectPointIndices(this->getObjectPointIndices());
                mPostFilter->setInputPointCloud(this->getInputPointCloud());
                mPostFilter->setInputPointIndices(indicesPtr);

				indicesPtr = IndicesPtr(new Indices());
                mPostFilter->filter(indicesPtr);
                mDebugHelperPtr->write("Ended post filtering", DebugHelper::FILTER);
			}
		}
	};
}
