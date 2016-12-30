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

#ifndef RATING_MODULE_HPP_
#define RATING_MODULE_HPP_

#include "next_best_view/common/CommonClass.hpp"
#include "next_best_view/rating/BaseScoreContainer.hpp"
#include <robot_model_services/robot_model/RobotState.hpp>

namespace next_best_view {
	/*!
	 * \brief RatingModule is a generalization for the rating of the point cloud objects and viewports.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 * \sa DefaultRatingModule
	 */
	class RatingModule : public CommonClass {
	public:
		/*!
		 * \brief constructor of the RatingModule object
		 */
		RatingModule();

		/*!
		 * \brief destructor of the RatingModule object.
		 */
		virtual ~RatingModule();

		/*!
         * \brief Sets the best score for the given candidate camera viewport by choosing
         * the best combination of objects.
         * Writes the chosen object combination and its score into the candidate viewport.
         * Does not change the pose of the candidate viewport.
         * \param currentViewport [in] the current camera viewport
         * \param candidateViewport [in,out] the candidate camera viewport.
         * If the rating is feasible, it contains the chosen object indices in
         * the child_indices member and the corresponding score in the score member
         * after this method was called
         * \return whether the rating is feasible or not
		 */
        virtual bool setBestScoreContainer(const ViewportPoint &currentViewport,
                                            ViewportPoint &candidateViewport) = 0;

        /*!
         * \brief sets the score container for one filtered camera viewport
         * \param currentViewport [in] the current camera viewport
         * \param candidateViewport [in,out] the filtered candidate camera viewport
         * \return whether the rating is feasible or not
         */
        virtual bool setSingleScoreContainer(const ViewportPoint &currentViewport,
                                        ViewportPoint &candidateViewport) = 0;

		/*!
		 * \return a shared pointer containing an empty rating object.
		 */
		virtual BaseScoreContainerPtr getScoreContainerInstance() = 0;

        /*!
         * \brief Sets the best viewport from the given viewports cloud
         * \param viewports [in] the viewports cloud
         * \param bestViewport [out] the best viewport
         * \return whether a best viewport was found
         */
        virtual bool getBestViewport(ViewportPointCloudPtr &viewports, ViewportPoint &bestViewport) = 0;

        /*!
         * \brief compares two viewports
         * \param a [in] viewport a
         * \param b [in] viewport b
         * \return whether a < b is true
         */
        virtual bool compareViewports(const ViewportPoint &a, const ViewportPoint &b) = 0;

        /*!
         * \brief compares two ratings.
         * \param a [in] comparison object A
         * \param b [in] comparison object B
         * \return whether A is smaller than B
         */
        virtual bool compareScoreContainer(const BaseScoreContainerPtr &a, const BaseScoreContainerPtr &b) = 0;

	/*!
         * \brief returns the weighted rating of a rating object.
	 * \param a [in] the rating object.
         * \return the weighted rating
	 */
        virtual float getRating(const BaseScoreContainerPtr &a) = 0;

        /**
         * @brief setRobotState sets the robot state, which might have an influence on the rating.
         * @param robotState the robot state.
         */
        virtual void setRobotState(robot_model_services::RobotStatePtr robotState) = 0;

        /*!
         * \brief resets the cached data for a call of setBestScoreContainer
         */
        virtual void resetCache() = 0;
    };

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<RatingModule> RatingModulePtr;
}

#endif /* RATING_MODULE_HPP_ */
