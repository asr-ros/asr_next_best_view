/*
 * DefaultRating.hpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#ifndef DEFAULTSCORECONTAINER_HPP_
#define DEFAULTSCORECONTAINER_HPP_

#include "next_best_view/rating/BaseScoreContainer.hpp"

namespace next_best_view {
	/*!
	 * \brief DefaultScoreContainer implements the BaseScoreContainer base class.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
	struct DefaultScoreContainer : public BaseScoreContainer {
	private:
		/*!
		 * \brief the density of elements.
		 */
		float mElementDensity;

		/*!
		 * \brief the normality rating.
		 */
		float mNormality;
	public:
		void setElementDensity(float value) {
			mElementDensity = value;
		}

		float getElementDensity() {
			return mElementDensity;
		}

		void setNormality(float value) {
			mNormality = value;
		}

		float getNormality() {
			return mNormality;
		}
	public:
		/*!
		 * \brief constructor of the DefaultRating object.
		 */
		DefaultScoreContainer();

		/*!
		 * \brief destructor of the DefaultRating object.
		 */
		virtual ~DefaultScoreContainer();
	};

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<DefaultScoreContainer> DefaultScoreContainerPtr;
}


#endif /* DEFAULTSCORECONTAINER_HPP_ */
