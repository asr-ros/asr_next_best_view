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

        float mMovementCostsBaseTranslation;
        float mMovementCostsBaseRotation;
        float mMovementCostsPTU;
        float mRecognitionCosts;

	public:

        /*!
         * \brief constructor of the DefaultRating object.
         */
        DefaultScoreContainer();

        /*!
         * \brief destructor of the DefaultRating object.
         */
        virtual ~DefaultScoreContainer();

        /*!
         * \brief sets the movement costs for the base translation
         * \param value the movement costs for the base translation
         */
        void setMovementCostsBaseTranslation(float value) {
            mMovementCostsBaseTranslation = value;
        }

        /*!
         * \brief returns the movement costs for the base translation
         * \return the movement costs for the base translation
         */
        float getMovementCostsBaseTranslation() {
            return mMovementCostsBaseTranslation;
        }

        /*!
         * \brief sets the movement costs for the base rotation
         * \param value the movement costs for the base rotation
         */
        void setMovementCostsBaseRotation(float value) {
            mMovementCostsBaseRotation = value;
        }

        /*!
         * \brief returns the movement costs for the base rotation
         * \return the movement costs for the base rotation
         */
        float getMovementCostsBaseRotation() {
            return mMovementCostsBaseRotation;
        }

        /*!
         * \brief sets the costs for the PTU movement
         * \param value the costs for the PTU movement
         */
        void setMovementCostsPTU(float value) {
            mMovementCostsPTU = value;
        }

        /*!
         * \brief returns the costs for the PTU movement
         * \return the costs for the PTU movement
         */
        float getMovementCostsPTU() {
            return mMovementCostsPTU;
        }

        /*!
         * \brief sets the costs for the recognition of the objects
         * \param value the costs for the recognition of the objects
         */
        void setRecognitionCosts(float value) {
            mRecognitionCosts = value;
        }

        /*!
         * \brief returns the costs for the recognition of the objects
         * \return the costs for the recognition of the objects
         */
        float getRecognitionCosts() {
            return mRecognitionCosts;
        }
	};

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<DefaultScoreContainer> DefaultScoreContainerPtr;
}


#endif /* DEFAULTSCORECONTAINER_HPP_ */
