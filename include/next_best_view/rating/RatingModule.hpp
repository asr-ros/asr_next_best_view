/*
 * Rating.hpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#ifndef RATING_MODULE_HPP_
#define RATING_MODULE_HPP_

#include "next_best_view/common/CommonPCLClass.hpp"
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
	class RatingModule : public CommonPCLClass {
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
		virtual bool getScoreContainer(const ViewportPoint &currentViewportPoint, const ViewportPoint &candidateViewportPoint, BaseScoreContainerPtr &scoreContainerPtr) = 0;

		/*!
		 * \brief normalizes a rating by a normalization rating object.
		 * \param normalizingScoreContainerPtr [in] the rating which normalized the subject rating.
		 * \param subjectScoreContainerPtr [in / out] the rating which should be normalied
		 */
		virtual void normalizeScoreContainer(const BaseScoreContainerPtr &normalizingScoreContainerPtr, BaseScoreContainerPtr &subjectScoreContainerPtr) = 0;

		/*!
		 * \brief maximizes the rating entries by taking the maximum values of rating and write it into currentMaximum.
		 * \param rating [in] the rating to compare with.
		 * \param currentMaximum [in / out] the currently maximum values.
		 */
		virtual void maximizeScoreContainer(const BaseScoreContainerPtr &scoreContainerPtr, BaseScoreContainerPtr &currentMaximumScoreContainerPtr) = 0;

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
