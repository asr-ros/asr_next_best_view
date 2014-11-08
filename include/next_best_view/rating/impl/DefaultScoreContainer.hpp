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
	public:
		/*!
		 * \brief the density of elements.
		 */
		float element_density;

		/*!
		 * \brief the normality rating.
		 */
		float normality;
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
