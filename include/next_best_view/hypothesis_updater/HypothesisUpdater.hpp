/*
 * HypothesisUpdater.hpp
 *
 *  Created on: Oct 2, 2014
 *      Author: ralfschleicher
 */

#ifndef HYPOTHESISUPDATER_HPP_
#define HYPOTHESISUPDATER_HPP_

#include "typedef.hpp"

namespace next_best_view {
	/*!
	 * \brief HypothesisUpdater is an abstract class for updating the objects in the point cloud.
	 */
	class HypothesisUpdater {
	public:
		HypothesisUpdater();
		virtual ~HypothesisUpdater();
		virtual void update(const ViewportPoint &viewportPoint) = 0;
	};

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<HypothesisUpdater> HypothesisUpdaterPtr;
}

#endif /* HYPOTHESISUPDATER_HPP_ */
