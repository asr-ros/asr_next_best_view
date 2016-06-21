/*
 * DefaultRating.hpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#ifndef DEFAULTSCORECONTAINER_HPP_
#define DEFAULTSCORECONTAINER_HPP_

#include <map>
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
        // this is the UNnormalized utility, while the BaseScoreContainer contains the normalized utility
        float mUtility;
        // the utility for each object type
        std::map<std::string, float> mObjectUtilities;
        float mUtilityNormalization;

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
        void setUnweightedInverseMovementCostsBaseTranslation(float value) {
            mMovementCostsBaseTranslation = value;
        }

        /*!
         * \brief returns the inverse movement costs for the base translation
         * \return the inverse movement costs for the base translation
         */
        float getUnweightedInverseMovementCostsBaseTranslation() {
            return mMovementCostsBaseTranslation;
        }

        /*!
         * \brief sets the inverse movement costs for the base rotation
         * \param value the inverse movement costs for the base rotation
         */
        void setUnweightedInverseMovementCostsBaseRotation(float value) {
            mMovementCostsBaseRotation = value;
        }

        /*!
         * \brief returns the inverse movement costs for the base rotation
         * \return the movement inverse costs for the base rotation
         */
        float getUnweightedInverseMovementCostsBaseRotation() {
            return mMovementCostsBaseRotation;
        }

        /*!
         * \brief sets the inverse costs for the PTU movement
         * \param value the inverse costs for the PTU movement
         */
        void setUnweightedInverseMovementCostsPTU(float value) {
            mMovementCostsPTU = value;
        }

        /*!
         * \brief returns the inverse costs for the PTU movement
         * \return the inverse costs for the PTU movement
         */
        float getUnweightedInverseMovementCostsPTU() {
            return mMovementCostsPTU;
        }

        /*!
         * \brief sets the inverse costs for the recognition of the objects
         * \param value the inverse costs for the recognition of the objects
         */
        void setUnweightedInverseRecognitionCosts(float value) {
            mRecognitionCosts = value;
        }

        /*!
         * \brief returns the inverse costs for the recognition of the objects
         * \return the inverse costs for the recognition of the objects
         */
        float getUnweightedInverseRecognitionCosts() {
            return mRecognitionCosts;
        }

        /*!
         * \brief sets the weighted unnormalized utility
         * \param value the utility
         */
        void setWeightedUnnormalizedUtility(float value) {
            mUtility = value;
        }

        /*!
         * \brief returns the weighted unnormalized utility
         * \return the utility
         */
        float getWeightedUnnormalizedUtility() {
            return mUtility;
        }

        /*!
         * \brief sets the utilitiy for an object type
         * \param objectType the object type
         * \param value the utility
         */
        void setUnweightedUnnormalizedObjectUtilitiy(std::string objectType, float value) {
            mObjectUtilities[objectType] = value;
        }

        /*!
         * \brief returns the utility for a given object type
         * \param objectType the object type
         * \return the utility
         */
        float getUnweightedUnnormalizedObjectUtility(std::string objectType) {
            if (mObjectUtilities.count(objectType) == 0) {
                return 0;
            }
            return mObjectUtilities[objectType];
        }

        /*!
         * \brief sets the utility normalization
         * \param value the utility normalization
         */
        void setUtilityNormalization(float value) {
            mUtilityNormalization = value;
        }

        /*!
         * \brief returns the utility normalization
         * \return the utility normalization
         */
        float getUtilityNormalization() {
            return mUtilityNormalization;
        }
	};

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<DefaultScoreContainer> DefaultScoreContainerPtr;
}


#endif /* DEFAULTSCORECONTAINER_HPP_ */
