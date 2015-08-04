/*
 * GeneralFilter.hpp
 *
 *  Created on: Aug 4, 2015
 *      Author: sxleixer
 */

#ifndef SRC_CONTROL_EXPLORATION_NEXT_BEST_VIEW_INCLUDE_NEXT_BEST_VIEW_COMMON_GENERALFILTER_HPP_
#define SRC_CONTROL_EXPLORATION_NEXT_BEST_VIEW_INCLUDE_NEXT_BEST_VIEW_COMMON_GENERALFILTER_HPP_

#include <boost/foreach.hpp>
#include <vector>

#include "next_best_view/common/CommonClass.hpp"

namespace next_best_view {
	class GeneralFilter;
	typedef boost::shared_ptr<GeneralFilter> GeneralFilterPtr;

	class GeneralFilter : public CommonClass {
	private:
		std::vector<GeneralFilterPtr> pre_filter_list;
		std::vector<GeneralFilterPtr> post_filter_list;
	public:
		/*!
		 * \brief Adding a pre filter to this filter
		 * \param filterPtr the filter pointer
		 */
		void add_pre_filter(GeneralFilterPtr filterPtr) {
			pre_filter_list.push_back(filterPtr);
		}

		/*!
		 * \brief Adding a post filter to this filter
		 * \param filterPtr the filter pointer
		 */
		void add_post_filter(GeneralFilterPtr filterPtr) {
			post_filter_list.push_back(filterPtr);
		}

		/*!
		 * \brief Does the filtering on the indices
		 * \param indicesPtr the indices
		 */
		virtual void do_filtering(IndicesPtr &indicesPtr) = 0;

		/*!
		 * \brief Applies the filter on the indices
		 * \param indicesPtr the indices
		 */
		void filter(IndicesPtr &indicesPtr) {
			// apply pre filters
			BOOST_FOREACH(GeneralFilterPtr filterPtr, pre_filter_list) {
				filterPtr->do_filtering(indicesPtr);
			}

			// apply this filter
			this->do_filtering(indicesPtr);

			// apply post filters
			BOOST_FOREACH(GeneralFilterPtr filterPtr, post_filter_list) {
				filterPtr->do_filtering(indicesPtr);
			}
		}
	};
}

#endif /* SRC_CONTROL_EXPLORATION_NEXT_BEST_VIEW_INCLUDE_NEXT_BEST_VIEW_COMMON_GENERALFILTER_HPP_ */
