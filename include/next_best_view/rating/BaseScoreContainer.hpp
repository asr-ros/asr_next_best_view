/*
 * BaseScoreContainer.hpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#ifndef BASESCORE_HPP_
#define BASESCORE_HPP_

#include <boost/shared_ptr.hpp>

namespace next_best_view {
	/*!
	 * \brief BaseScore implements no such functionalities, but the corresponding RatingModule has to implement a BaseScore class itself.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 * \sa DefaultRating
	 */
	struct BaseScoreContainer {
	private:
		float mUtility;
        float mInverseCosts;
	public:
        void setWeightedNormalizedUtility(float value) {
			mUtility = value;
		}

        float getWeightedNormalizedUtility() {
			return mUtility;
		}

        void setWeightedInverseCosts(float value) {
            mInverseCosts = value;
		}

        float getWeightedInverseCosts() {
            return mInverseCosts;
		}
	public:
		BaseScoreContainer();

		/*!
		 * \brief destructor of the object.
		 */
		virtual ~BaseScoreContainer() = 0;
	};

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<BaseScoreContainer> BaseScoreContainerPtr;
}


#endif /* BASESCORE_HPP_ */
