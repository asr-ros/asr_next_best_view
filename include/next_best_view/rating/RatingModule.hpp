/*
 * Rating.hpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#ifndef RATING_MODULE_HPP_
#define RATING_MODULE_HPP_

#include "next_best_view/common/CommonClass.hpp"
#include "next_best_view/rating/BaseScoreContainer.hpp"


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
    };

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<RatingModule> RatingModulePtr;
}

#endif /* RATING_MODULE_HPP_ */
