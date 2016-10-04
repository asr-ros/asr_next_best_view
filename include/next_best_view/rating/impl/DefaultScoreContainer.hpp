/**

Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Heller Florian, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Stroh Daniel, Trautmann Jeremias, Walter Milena, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

4. The use is explicitly not permitted to any application which deliberately try to kill or do harm to any living creature.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef DEFAULTSCORECONTAINER_HPP_
#define DEFAULTSCORECONTAINER_HPP_

#include <map>
#include <boost/make_shared.hpp>
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
        float getUnweightedInverseMovementCostsBaseTranslation() const {
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
        float getUnweightedInverseMovementCostsBaseRotation() const {
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
        float getUnweightedInverseMovementCostsPTU() const {
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
        float getUnweightedInverseRecognitionCosts() const {
            return mRecognitionCosts;
        }

        /*!
         * \brief sets the weighted unnormalized utility
         * \param value the utility
         */
        void setUnweightedUnnormalizedUtility(float value) {
            mUtility = value;
        }

        /*!
         * \brief returns the weighted unnormalized utility
         * \return the utility
         */
        float getUnweightedUnnormalizedUtility() const {
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
        float getUnweightedUnnormalizedObjectUtility(std::string objectType) const {
            if (mObjectUtilities.count(objectType) == 0) {
                return 0;
            }
            return mObjectUtilities.at(objectType);
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
        float getUtilityNormalization() const {
            return mUtilityNormalization;
        }

        /*!
         * \brief getObjectTypes
         * \return a vector of object types which contain a object utility.
         */
        boost::shared_ptr<std::vector<boost::shared_ptr<std::string>>> getObjectTypes() const {
            auto objectTypes = boost::make_shared<std::vector<boost::shared_ptr<std::string>>>();
            if (mObjectUtilities.size() == 0)
                return objectTypes;
            for (auto object : mObjectUtilities) {
                auto objectType = boost::shared_ptr<std::string>(new std::string(object.first));
                objectTypes->push_back(objectType);
            }
            return objectTypes;
        }
	};

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<DefaultScoreContainer> DefaultScoreContainerPtr;

    std::ostream& operator<<(std::ostream &strm, const next_best_view::DefaultScoreContainer &score);
    std::ostream& operator<<(std::ostream &strm, const next_best_view::DefaultScoreContainerPtr &score);
}


#endif /* DEFAULTSCORECONTAINER_HPP_ */
