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
         * \brief the rating of the object positions.
		 */
        float mPositionRating;

		/*!
         * \brief the rating of the object orientations.
		 */
        float mOrientationRating;
	public:
        /*!
         * \brief sets the rating of the object positions
         * \param value the position rating
         */
        void setPositionRating(float value) {
            mPositionRating = value;
		}

        /*!
         * \brief returns the rating of the object positions
         * \return the position rating
         */
        float getPositionRating() {
            return mPositionRating;
		}

        /*!
         * \brief sets the rating of the object orientations
         * \param value the orientation rating
         */
        void setOrientationRating(float value) {
            mOrientationRating = value;
		}

        /*!
         * \brief returns the rating of the object orientations
         * \return the orientation rating
         */
        float getOrientationRating() {
            return mOrientationRating;
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
