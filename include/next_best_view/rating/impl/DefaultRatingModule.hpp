/*
 * DefaultRatingModule.hpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#pragma once

#include <ros/ros.h>
#include <boost/foreach.hpp>
#include "next_best_view/camera_model_filter/CameraModelFilter.hpp"
#include "next_best_view/helper/DebugHelper.hpp"
#include "next_best_view/helper/MathHelper.hpp"
#include "next_best_view/rating/RatingModule.hpp"
#include "next_best_view/rating/impl/DefaultScoreContainer.hpp"
#include "next_best_view/robot_model/RobotModel.hpp"

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
        /*
         * parameters to calculate the utility / costs
         */
        // the threshold for the angle between the orientation of an object and the camera orientation
        double mNormalAngleThreshold;
        double mFovV,mFovH;
        double mFcp,mNcp;
        double mOmegaUtility;
        double mOmegaPan;
        double mOmegaTilt;
        double mOmegaRot;
        double mOmegaBase;
        double mOmegaRecognition;

        /*
         * help members
         */
        RobotModelPtr mRobotModelPtr;
        CameraModelFilterPtr mCameraModelFilterPtr;
        DebugHelperPtr mDebugHelperPtr;

        /*
         * members to cache the calculated data for one call of setBaseScoreContainer
         * must be reset everytime setBaseScoreContainer is called
         */
        // the utility for each object type
        std::map<std::string, float> mObjectUtilities;
        // the full costs of the movement - does not contain the costs of the recognition
        double mInverseMovementCosts = -1;
        // the costs of the translational movement of the base
        Precision mInverseMovementCostsBaseTranslation = -1;
        // the costs of the rotational movement of the base
        Precision mInverseMovementCostsBaseRotation = -1;
        // the costs of the PTU movement
        Precision mInverseMovementCostsPTU = -1;
        // the costs of the recognition of the objects
        double mInverseRecognitionCosts = -1;
        // the normalization value for the costs
        double mCostsNormalization = -1;
        // the PTU omega parameter
        Precision mOmegaPTU = -1;
        // the target state
        RobotStatePtr mTargetState = NULL;

        /*
         * members to cache the calculated data
         */
        // the maximum recognition costs - stays the same as long as the input cloud does not change
        double mMaxRecognitionCosts;
        // whether the input cloud changed since the last time the maximum recognition costs were updated
        bool mInputCloudChanged = true;

	public:
		DefaultRatingModule();

        DefaultRatingModule(double mFovV, double mFovH, double mFcp, double mNcp,
                                const RobotModelPtr &robotModelPtr = RobotModelPtr(),
                                const CameraModelFilterPtr &cameraModelFilterPtr = CameraModelFilterPtr());

        virtual ~DefaultRatingModule();

        void setInputCloud(const ObjectPointCloudPtr &pointCloudPtr);

        bool setBestScoreContainer(const ViewportPoint &currentViewport, ViewportPoint &candidateViewport);

        bool getBestViewport(ViewportPointCloudPtr &viewports, ViewportPoint &bestViewport);

        bool compareViewports(const ViewportPoint &a, const ViewportPoint &b);

        bool compareScoreContainer(const BaseScoreContainerPtr &a, const BaseScoreContainerPtr &b);

        BaseScoreContainerPtr getScoreContainerInstance();

		/*!
         * \brief returns the orientation utility of an object point for a given camera viewport.
         * The orientation utility is always between 0.0 and 1.0.
         * \param viewport [in] the camera viewport
         * \param objectPoint [in] the object point.
         * \return the orientation utility
		 */
        float getOrientationUtility(const ViewportPoint &viewport, ObjectPoint &objectPoint);

        /*!
         * \brief returns the utility of the position of an object in the frustum of a given camera viewport.
         * The frustum position utility is always between 0.0 and 1.0.
         * \param viewport [in] the camera viewport
         * \param objectPoint [in] the object point.
         * \return the frustum position utility
         */
        float getFrustumPositionUtility(const ViewportPoint &viewport, ObjectPoint &objectPoint);

		/*!
         * \brief returns the utility of one object normal for a given camera viewport.
         * The normal utility is always between 0.0 and 1.0.
         * \param viewport [in] the the camera viewport
         * \param objectNormalVector [in] the normalized vector which is perpendicular to the object surface
         * \return the normality utility
		 */ 
        float getNormalUtility(const ViewportPoint &viewport, const SimpleVector3 &objectNormalVector);

        /*!
         * \brief returns the proximity utility of a given object point for a given camera viewport.
         * The proximizy utility is always between 0.0 and 1.0.
         * \param viewport the camera viewport
         * \param objectPoint the object point
         * \return the proximity utility
         */
        float getProximityUtility(const ViewportPoint &viewport, const ObjectPoint &objectPoint);

		/*!
         * \brief returns the weighted rating of a rating object.
		 * \param a [in] the rating object.
         * \return the weighted rating
		 */
        float getRating(const BaseScoreContainerPtr &a);

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

        void setOmegaParameters(double omegaUtility, double omegaPan, double omegaTilt, double omegaRot, double omegaBase, double omegaRecognition);

    private:
        /*!
         * \brief returns the normalized rating for a given deviation from the optimum and a threshold for the deviation.
         * The normalized rating is always between 0.0 and 1.0.
         * \param rating the actual devation
         * \param threshold the threshold for the deviation
         * \return the normalized rating
         */
        float getNormalizedRating(float deviation, float threshold);

        /*!
         * \brief sets the score container for one filtered camera viewport
         * \param currentViewport [in] the current camera viewport
         * \param candidateViewport [in,out] the filtered candidate camera viewport
         * \return whether the rating is feasible or not
         */
        bool setSingleScoreContainer(const ViewportPoint &currentViewport,
                                        ViewportPoint &candidateViewport);

        /*!
         * \brief returns the utility for the given candidate camera viewport.
         * \param candidateViewport the candidate camera viewport
         * \return the utility
         */
        double getUtility(const ViewportPoint &candidateViewport);

        /*!
         * \brief sets the mObjectUtilities member.
         * \param candidateViewport the candidate camera viewport
         * \param objectType the object type for which the utility shall be set
         */
        void setObjectUtilities(const ViewportPoint &candidateViewport, std::string objectType);

        /*!
         * \brief returns the inverse costs for the movement from the source to the target viewport
         * and the recognition of the objects
         * \param sourceViewport the source viewport
         * \param targetViewport the target viewport
         * \return the inverse movement and recognition costs
         */
        double getInverseCosts(const ViewportPoint &sourceViewport, const ViewportPoint &targetViewport);

        /*!
         * \brief returns the inverse recognition costs for the objects in the target viewport
         * \param targetViewport the target viewport
         * \return the normalized inverse recognition costs
         */
        double getInverseRecognitionCosts(const ViewportPoint &targetViewport);

        /*!
         * \brief sets the robot state and the mTargetState member.
         * Must be called before calling setOmegaPTU and setMovementCosts.
         * \param sourceViewport [in] the source viewport
         * \param targetViewport [in] the target viewport
         * \param targetState [out] the target state
         */
        void setRobotState(const ViewportPoint &sourceViewport, const ViewportPoint &targetViewport);

        /*!
         * \brief sets the mOmegaPTU member. Must be called before calling setMovementCosts and setCostsNormalization.
         */
        void setOmegaPTU();

        /*!
         * \brief sets the mMovementCosts member
         */
        void setInverseMovementCosts();

        /*!
         * \brief sets the mCostsNormalization member
         */
        void setCostsNormalization();

        /*!
         * \brief sets the mMaxRecognitionCosts member.
         * Must be called before calling getNormalizedRecognitionCosts everytime the input cloud changed.
         */
        void setMaxRecognitionCosts();

        /*!
         * \brief resets the cached data for a call of setBestScoreContainer
         */
        void resetCache();
	};

    /*!
     * \brief Definition for the shared pointer type of the class.
     */
    typedef boost::shared_ptr<DefaultRatingModule> DefaultRatingModulePtr;
}
