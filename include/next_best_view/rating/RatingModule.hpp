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
		 * \brief returns if the rating was feasible and writes the result into the ratingPtr variable
		 * \param currentViewportPoint [in] the current viewport point
		 * \param candidateViewportPoint [in] the candidate viewport point
		 * \param scoreContainerPtr [out] the shared pointer for writing a rating into it
		 * \return if the rating is feasible or not
		 */
		virtual bool getScoreContainer(const ViewportPoint &currentViewportPoint, ViewportPoint &candidateViewportPoint, BaseScoreContainerPtr &scoreContainerPtr) = 0;

		/*!
		 * \return a shared pointer containing an empty rating object.
		 */
		virtual BaseScoreContainerPtr getScoreContainerInstance() = 0;

		/*!
		 * \brief compares two ratings.
		 * \param a [in] comparison object A
		 * \param b [in] comparison object B
		 * \return if A is smaller than B
		 */
		virtual bool compareScoreContainer(const BaseScoreContainerPtr &a, const BaseScoreContainerPtr &b) = 0;
	};

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<RatingModule> RatingModulePtr;
}

#endif /* RATING_MODULE_HPP_ */
