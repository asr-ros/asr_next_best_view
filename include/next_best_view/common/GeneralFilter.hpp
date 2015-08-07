/*
 * GeneralFilter.hpp
 *
 *  Created on: Aug 4, 2015
 *      Author: sxleixer
 */

#ifndef SRC_CONTROL_EXPLORATION_NEXT_BEST_VIEW_INCLUDE_NEXT_BEST_VIEW_COMMON_GENERALFILTER_HPP_
#define SRC_CONTROL_EXPLORATION_NEXT_BEST_VIEW_INCLUDE_NEXT_BEST_VIEW_COMMON_GENERALFILTER_HPP_

#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <vector>

#include "next_best_view/common/CommonClass.hpp"

namespace next_best_view {
	class GeneralFilter;
	typedef boost::shared_ptr<GeneralFilter> GeneralFilterPtr;

	class GeneralFilter : public CommonClass {
	private:
		GeneralFilterPtr mPreFilter;
		GeneralFilterPtr mPostFilter;
	public:
		/*!
		 * Constructor.
		 */
		GeneralFilter() : mPreFilter(), mPostFilter() {
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
		 * \param indicesPtr the indices
		 */
		void filter(IndicesPtr &indicesPtr) {
			// set an empty indices list.
			indicesPtr = IndicesPtr(new Indices());

			// apply pre filter
			if (mPreFilter != GeneralFilterPtr()) {
				ROS_DEBUG("Start pre filtering");
				// set the working input cloud and the relevant indices to work on.
				mPreFilter->setInputCloud(this->getInputCloud());
				mPreFilter->setIndices(this->getIndices());

				// do the filtering
				mPreFilter->doFiltering(indicesPtr);

				// set the indices of the filter to the already filtered indices and reset the return indices pointer.
				this->setIndices(indicesPtr);
				indicesPtr = IndicesPtr(new Indices());
				ROS_DEBUG("Ended pre filtering");
			}

			// apply this filter
			this->doFiltering(indicesPtr);

			// apply post filter
			if (mPostFilter != GeneralFilterPtr()) {
				ROS_DEBUG("Start post filtering");
				// set the working input cloud and the relevant indices to work on.
				mPostFilter->setInputCloud(this->getInputCloud());
				mPostFilter->setIndices(indicesPtr);

				indicesPtr = IndicesPtr(new Indices());
				mPostFilter->doFiltering(indicesPtr);
				ROS_DEBUG("Ended post filtering");
			}
		}
	};
}

#endif /* SRC_CONTROL_EXPLORATION_NEXT_BEST_VIEW_INCLUDE_NEXT_BEST_VIEW_COMMON_GENERALFILTER_HPP_ */
