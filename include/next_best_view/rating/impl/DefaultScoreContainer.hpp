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
         * \brief the utility of the object positions.
		 */
        float mPositionUtility;

		/*!
         * \brief the utility of the object orientations.
		 */
        float mOrientationUtility;
	public:
        /*!
         * \brief sets the utility of the object positions
         * \param value the position utility
         */
        void setPositionUtility(float value) {
            mPositionUtility = value;
		}

        /*!
         * \brief returns the utility of the object positions
         * \return the position utility
         */
        float getPositionUtility() {
            return mPositionUtility;
		}

        /*!
         * \brief sets the utility of the object orientations
         * \param value the orientation utility
         */
        void setOrientationUtility(float value) {
            mOrientationUtility = value;
		}

        /*!
         * \brief returns the utility of the object orientations
         * \return the orientation utility
         */
        float getOrientationUtility() {
            return mOrientationUtility;
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
