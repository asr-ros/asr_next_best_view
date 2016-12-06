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
         *  parameters to disable / enable parts of utility calculation
         */
        bool mUseOrientationUtility = true;
        bool mUseProximityUtility = true;
        bool mUseSideUtility = true;

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
        std::map<std::string, float> mUnweightedUnnormalizedObjectUtilities;
        // the full costs of the movement - does not contain the costs of the recognition
        double mWeightedInverseMovementCosts = -1;
        // the costs of the translational movement of the base
        Precision mUnweightedInverseMovementCostsBaseTranslation = -1;
        // the costs of the rotational movement of the base
        Precision mUnweightedInverseMovementCostsBaseRotation = -1;
        // the costs of the PTU movement
        Precision mUnweightedInverseMovementCostsPTU = -1;
        // the costs of the recognition of the objects
        double mUnweightedInverseRecognitionCosts = -1;
        // the normalization value for the rating
        double mRatingNormalization = -1;
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
         * \param objerctPosition [in] the position of the object
         * \return the normality utility
         */
        float getNormalUtility(const ViewportPoint &viewport, const SimpleVector3 &objectNormalVector, const SimpleVector3 &objectPosition);

        /*!
         * \brief returns the utility of one object normal for a given camera viewport.
         * The normal utility is always between 0.0 and 1.0.
         * \param viewport [in] the the camera viewport
         * \param objectNormalVector [in] the normalized vector which is perpendicular to the object surface
         * \param objerctPosition [in] the position of the object
         * \param angleThreshold [in] the angle threshold
         * \return the normality utility
		 */ 
        float getNormalUtility(const ViewportPoint &viewport, const SimpleVector3 &objectNormalVector, const SimpleVector3 &objectPosition, double angleThreshold);

        /*!
         * \brief returns the proximity utility of a given object point for a given camera viewport.
         * The proximizy utility is always between 0.0 and 1.0.
         * Depends on the distance between the object and the camera.
         * \param viewport the camera viewport
         * \param objectPoint the object point
         * \return the proximity utility
         */
        float getProximityUtility(const ViewportPoint &viewport, const ObjectPoint &objectPoint);

        /*!
         * \brief returns the side utility of a given object point for a given camera viewport.
         * The side utility is always between 0.0 and 1.0.
         * Depends on how far the object is to the side of the camera view/srustum.
         * \param viewport the camera viewport
         * \param objectPoint objectPoint the object point
         * \return the side utility
         */
        float getSideUtility(const ViewportPoint &viewport, const ObjectPoint &objectPoint);

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

        /*!
         * \brief returns a rating for the difference between the two given normals.
         * \param v1 first normal
         * \param v2 second normal
         * \param angleThreshold
         * \return rating for the difference of v1 and v2
         */
        float getNormalizedAngleUtility(const SimpleVector3 v1, const SimpleVector3 v2, double angleThreshold);

        void setOmegaParameters(double omegaUtility, double omegaPan, double omegaTilt, double omegaRot, double omegaBase, double omegaRecognition);

        void setUtilityParameters(bool useOrientationUtility = true, bool useProximityUtility = true, bool useSideUtility = true);

        bool setSingleScoreContainer(const ViewportPoint &currentViewport,
                                        ViewportPoint &candidateViewport);

        /**
         * @brief setRobotState sets mRobotModelPtr's state.
         * @param robotState
         */
        void setRobotState(RobotStatePtr robotState);

        /*!
         * \brief resets the cached data for a call of setBestScoreContainer
         */
        void resetCache();

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
         * \brief returns the utility for the given candidate camera viewport.
         * \param candidateViewport the candidate camera viewport
         * \return the utility
         */
        double getUnweightedUnnormalizedUtility(const ViewportPoint &candidateViewport);

        /*!
         * \brief sets the mObjectUtilities member.
         * \param candidateViewport the candidate camera viewport
         * \param objectType the object type for which the utility shall be set
         */
        void setUnweightedUnnormalizedObjectUtilities(const ViewportPoint &candidateViewport, std::string objectType);

        /*!
         * \brief returns the inverse costs for the movement from the source to the target viewport
         * and the recognition of the objects
         * \param sourceViewport the source viewport
         * \param targetViewport the target viewport
         * \return the inverse movement and recognition costs
         */
        double getWeightedInverseCosts(const ViewportPoint &sourceViewport, const ViewportPoint &targetViewport);

        /*!
         * \brief returns the inverse recognition costs for the objects in the target viewport
         * \param targetViewport the target viewport
         * \return the normalized inverse recognition costs
         */
        double getUnweightedInverseRecognitionCosts(const ViewportPoint &targetViewport);

        /*!
         * \brief sets the robot state and the mTargetState member.
         * Must be called before calling setOmegaPTU and setMovementCosts.
         * \param sourceViewport [in] the source viewport
         * \param targetViewport [in] the target viewport
         * \param targetState [out] the target state
         */
        void setRobotState(const ViewportPoint &sourceViewport, const ViewportPoint &targetViewport);

        /*!
         * \brief sets the mMovementCosts member. Must be called before calling setRatingNormalization.
         */
        void setWeightedInverseMovementCosts();

        /*!
         * \brief sets the mRatingNormalization member
         */
        void setRatingNormalization();

        /*!
         * \brief sets the mMaxRecognitionCosts member.
         * Must be called before calling getNormalizedRecognitionCosts everytime the input cloud changed.
         */
        void setMaxRecognitionCosts();

	};

    /*!
     * \brief Definition for the shared pointer type of the class.
     */
    typedef boost::shared_ptr<DefaultRatingModule> DefaultRatingModulePtr;
}
