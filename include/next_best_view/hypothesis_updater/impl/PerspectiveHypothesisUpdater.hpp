/*
 * DefaultHypothesisUpdater.hpp
 *
 *  Created on: Oct 2, 2014
 *      Author: ralfschleicher
 */

#ifndef PERSPECTIVEHYPOTHESISUPDATER_HPP_
#define PERSPECTIVEHYPOTHESISUPDATER_HPP_

#include "next_best_view/hypothesis_updater/HypothesisUpdater.hpp"
#include "next_best_view/rating/impl/DefaultRatingModule.hpp"

namespace next_best_view {
	class PerspectiveHypothesisUpdater : public HypothesisUpdater {
	private:
		DefaultRatingModulePtr mDefaultRatingModulePtr;
	public:
		PerspectiveHypothesisUpdater();
		virtual ~PerspectiveHypothesisUpdater();

		void update(const ViewportPoint &viewportPoint);

		void setDefaultRatingModule(const DefaultRatingModulePtr &defaultRatingModulePtr);

		DefaultRatingModulePtr getDefaultRatingModule();
	};

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<PerspectiveHypothesisUpdater> PerspectiveHypothesisUpdaterPtr;
}


#endif /* PERSPECTIVEHYPOTHESISUPDATER_HPP_ */
