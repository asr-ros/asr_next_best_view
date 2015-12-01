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
        double mNormalAngleThreshold;
        double fovV,fovH;
        double fcp,ncp;

	public:
		DefaultRatingModule();

        DefaultRatingModule(double fovV, double fovH, double fcp, double ncp):fovV(fovV),fovH(fovH),fcp(fcp),ncp(ncp){}

		virtual ~DefaultRatingModule();

        bool getScoreContainer(const ViewportPoint &currentCameraViewport, ViewportPoint &candidateCameraViewport, BaseScoreContainerPtr &scoreContainerPtr);

		bool compareScoreContainer(const BaseScoreContainerPtr &a, const BaseScoreContainerPtr &b);

        /*!
         * \brief updates the object points by removing all normal vectors that are searched for in the given camera viewport.
         * \param cameraViewport the camera viewport
         */
        void updateObjectPoints(const ViewportPoint &cameraViewport);

		BaseScoreContainerPtr getScoreContainerInstance();

		/*!
         * \brief returns the orientation rating of an object point for a given camera viewport.
         * The orientation rating is always between 0.0 and 1.0.
         * \param cameraViewport [in] the camera viewport
         * \param objectPoint [in] the object point.
         * \return the orientation rating
		 */
        float getOrientationRating(const ViewportPoint &cameraViewport, ObjectPoint &objectPoint);

        /*!
         * \brief returns the rating of the position of an object in the frustum of a given camera viewport.
         * The frustum position rating is always between 0.0 and 1.0.
         * \param cameraviewport [in] the camera viewport
         * \param objectPoint [in] the object point.
         * \return the frustum position rating
         */
        float getFrustumPositionRating(const ViewportPoint &cameraViewport, ObjectPoint &objectPoint);

		/*!
         * \brief returns the rating of one object normal for a given camera viewport.
         * The normal rating is always between 0.0 and 1.0.
         * \param cameraViewport [in] the the camera viewport
         * \param objectNormalVector [in] the normalized vector which is perpendicular to the object surface
         * \return the normality rating
		 */ 
        float getNormalRating(const ViewportPoint &cameraViewport, const SimpleVector3 &objectNormalVector);

        /*!
         * \brief returns the proximity rating of a given object point for a given camera viewport.
         * The proximizy rating is always between 0.0 and 1.0.
         * \param cameraViewport the camera viewport
         * \param objectPoint the object point
         * \return
         */
        float getProximityRating(const ViewportPoint &cameraViewport, const ObjectPoint &objectPoint);

		/*!
         * \brief returns the weighted rating of a rating object.
		 * \param a [in] the rating object.
		 * \return the weighted rating
		 */
		float getRating(const DefaultScoreContainerPtr &a);

		/*!
         * \brief sets the threshold for the angle between the orientation of an object and the camera orientation.
         * \param angle the angle threshold
		 */
        void setNormalAngleThreshold(double angle);

		/*!
         * Retuns the threshold for the angle between the orientation of an object and the camera orientation.
         * \return the angle threshold.
		 */
        double getNormalAngleThreshold();

    private:
        /*!
         * \brief returns the normalized rating for a given deviation from the optimum and a threshold for the deviation.
         * The normalized rating is always between 0.0 and 1.0.
         * \param rating the actual devation
         * \param threshold the threshold for the deviation
         * \return the normalized rating
         */
        float getNormalizedRating(float deviation, float threshold);
	};

    /*!
     * \brief Definition for the shared pointer type of the class.
     */
    typedef boost::shared_ptr<DefaultRatingModule> DefaultRatingModulePtr;
}


#endif /* DEFAULTRATINGMODULE_HPP_ */
