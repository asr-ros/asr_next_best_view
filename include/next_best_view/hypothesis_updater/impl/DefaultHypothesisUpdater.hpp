/*
 * DefaultHypothesisUpdater.hpp
 *
 *  Created on: Oct 2, 2014
 *      Author: ralfschleicher
 */

#ifndef DEFAULTHYPOTHESISUPDATER_HPP_
#define DEFAULTHYPOTHESISUPDATER_HPP_

#include "next_best_view/hypothesis_updater/HypothesisUpdater.hpp"

namespace next_best_view {
	class DefaultHypothesisUpdater : public HypothesisUpdater {
	public:
		DefaultHypothesisUpdater();
		virtual ~DefaultHypothesisUpdater();

        unsigned int update(const ViewportPoint &viewportPoint);
	};

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<DefaultHypothesisUpdater> DefaultHypothesisUpdaterPtr;
}


#endif /* DEFAULTHYPOTHESISUPDATER_HPP_ */
