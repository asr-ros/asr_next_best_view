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

        float mInverseMovementCostsBaseTranslation;
        float mInverseMovementCostsBaseRotation;
        float mInverseMovementCostsPTU;
        float mInverseRecognitionCosts;

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
         * \brief sets the inverse movement costs for the base translation
         * \param value the inverse movement costs for the base translation
         */
        void setInverseMovementCostsBaseTranslation(float value) {
            mInverseMovementCostsBaseTranslation = value;
        }

        /*!
         * \brief returns the inverse movement costs for the base translation
         * \return the inverse movement costs for the base translation
         */
        float getInverseMovementCostsBaseTranslation() {
            return mInverseMovementCostsBaseTranslation;
        }

        /*!
         * \brief sets the inverse movement costs for the base rotation
         * \param value the inverse movement costs for the base rotation
         */
        void setInverseMovementCostsBaseRotation(float value) {
            mInverseMovementCostsBaseRotation = value;
        }

        /*!
         * \brief returns the inverse movement costs for the base rotation
         * \return the movement inverse costs for the base rotation
         */
        float getInverseMovementCostsBaseRotation() {
            return mInverseMovementCostsBaseRotation;
        }

        /*!
         * \brief sets the inverse costs for the PTU movement
         * \param value the inverse costs for the PTU movement
         */
        void setInverseMovementCostsPTU(float value) {
            mInverseMovementCostsPTU = value;
        }

        /*!
         * \brief returns the inverse costs for the PTU movement
         * \return the inverse costs for the PTU movement
         */
        float getInverseMovementCostsPTU() {
            return mInverseMovementCostsPTU;
        }

        /*!
         * \brief sets the inverse costs for the recognition of the objects
         * \param value the inverse costs for the recognition of the objects
         */
        void setInverseRecognitionCosts(float value) {
            mInverseRecognitionCosts = value;
        }

        /*!
         * \brief returns the inverse costs for the recognition of the objects
         * \return the inverse costs for the recognition of the objects
         */
        float getInverseRecognitionCosts() {
            return mInverseRecognitionCosts;
        }
	};

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<DefaultScoreContainer> DefaultScoreContainerPtr;
}


#endif /* DEFAULTSCORECONTAINER_HPP_ */
