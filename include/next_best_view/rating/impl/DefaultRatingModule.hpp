/*
 * DefaultRatingModule.hpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#ifndef DEFAULTRATINGMODULE_HPP_
#define DEFAULTRATINGMODULE_HPP_

#include <ros/ros.h>
#include "next_best_view/rating/RatingModule.hpp"
#include "next_best_view/rating/impl/DefaultScoreContainer.hpp"

namespace next_best_view {
	/*!
	 * \brief DefaultRatingModule implements the functionlities offered by RatingModule.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
	class DefaultRatingModule : public RatingModule {
	private:
		/*!
		 * \brief the normality rating angle
		 */
		double mNormalityRatingAngle;
	public:
		DefaultRatingModule();

		virtual ~DefaultRatingModule();

		bool getScoreContainer(const ViewportPoint &currentViewport, ViewportPoint &candidateViewportPoint, BaseScoreContainerPtr &scoreContainerPtr);

		bool compareScoreContainer(const BaseScoreContainerPtr &a, const BaseScoreContainerPtr &b);

		void updateObjectPoints(const ViewportPoint &viewportPoint);

		BaseScoreContainerPtr getScoreContainerInstance();

		/*!
		 * \brief returns the normality rating between the vieportPoint and an object point.
		 * \param viewportPoint [in] the viewport point
		 * \param objectPoint [in] the object point.
		 * \return the normality rating
		 */
		float getNormalityRating(const ViewportPoint &viewportPoint, ObjectPoint &objectPoint);

		/*!
		 * \brief returns the single normality rating of two vectors.
		 * \param viewportNormalVector [in] the normal vector of the viewport which is perpendicular to the NCP/FCP
		 * \param objectSurfaceNormalVector [in] the vector which is perpendicular to the object surface
		 * \param angleThreshold the threshold on which the rating is zero
		 * \return the normality rating
		 */
		float getSingleNormalityRating(const SimpleVector3 &viewportNormalVector, const SimpleVector3 &objectSurfaceNormalVector, float angleThreshold = M_PI);

		/*!
		 * \brief returns the single normality rating of two quaternions.
		 * \param quaternionA [in] the orientation A
		 * \param quaternionB [in] the orientation B
		 * \param angleThreshold the threshold on which the rating is zero
		 * \return the normality rating
		 */
		float getSingleNormalityRatingByQuaternion(const SimpleQuaternion &quaternionA, const SimpleQuaternion &quaternionB, float angleThreshold = M_PI);

		/*!
		 * \brief the weighted rating of a rating object.
		 * \param a [in] the rating object.
		 * \return the weighted rating
		 */
		float getRating(const DefaultScoreContainerPtr &a);

		/*!
		 * \brief sets the normality rating angle.
		 * \param angle the angle for the rating
		 */
		void setNormalityRatingAngle(double angle);

		/*!
		 * \return the normality angle.
		 */
		double getNormalityRatingAngle();
	};

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<DefaultRatingModule> DefaultRatingModulePtr;
}


#endif /* DEFAULTRATINGMODULE_HPP_ */
