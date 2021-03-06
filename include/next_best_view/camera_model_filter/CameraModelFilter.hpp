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

#ifndef CAMERAMODELFILTER_HPP_
#define CAMERAMODELFILTER_HPP_

#include "next_best_view/common/GeneralFilter.hpp"
#include <visualization_msgs/MarkerArray.h>

namespace next_best_view {
    namespace viz = visualization_msgs;

	/*!
	 * \brief CameraModelFilter class was built to generalize the filtering for different camera models.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
	class CameraModelFilter : public GeneralFilter {
	private:
		/*!
		 * \brief marks if the parameters changed.
		 */
		bool mParametersChanged;

		/*!
		 * \brief the horizontal field of view.
		 */
		double mFOVX;

		/*!
		 * \brief the vertical field of view.
		 */
		double mFOVY;

		/*!
		 * \brief the near clipping distance.
		 */
		double mNcp;

		/*!
		 * \brief the far clipping distance.
		 */
		double mFcp;

		/*!
		 * \brief the position of the pivot point.
		 */
		SimpleVector3 mPivotPointPosition;

		/**
		 * \brief the orientation of the pivot point.
		 */
		SimpleQuaternion mPivotPointOrientation;
        // TODO different recognizerCosts for different objects
		/**
		 * \brief The time it takes to recognize an object (in seconds)
		 */
		float recognizerCosts;
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	public:
		/*!
		 * \brief constructs a new camera model filter.
		 */
		CameraModelFilter();

		/*!
		 * \brief destructs the camera model filter.
		 */
		virtual ~CameraModelFilter() = 0;

		/*!
		 * \brief returns a array of markers containing the visualization of the camera frustum.
		 * \param sequence [in / out] a number used for unique identifying of the markers in the array
		 * \param lifetime the lifetime of each marker
		 * \return the visualization for that camera filter setting
		 */
		virtual viz::MarkerArrayPtr getVisualizationMarkerArray(uint32_t &sequence, double lifetime = 30.0) = 0;

        /*!
		 * \param recognizerCosts the costs of the recognition of the object
         * \param objectType the object to recognize.
		 */
        virtual void setRecognizerCosts(float recognizerCosts, std::string objectType);
		
		/*!
         * \param objectType the object to recognize.
		 * \returns the costs of the recognition of the object
		 */
        virtual float getRecognizerCosts(std::string objectType);

		/*!
		 * \return if the parameters changed
		 */
		inline bool haveParametersChanged() {
			return mParametersChanged;
		}

		/*!
		 * \param value set to changed or not.
		 */
		inline void setParametersChanged(bool value) {
			mParametersChanged = value;
		}

		/*!
		 * \brief sets the pose of the pivot point.
		 * \param position [in] a 3d vector pointing to a position
		 * \param orientation [in] a quaternion which denotes the orientation
		 */
		void setPivotPointPose(const SimpleVector3 &position, const SimpleQuaternion &orientation);

		/*!
		 * \brief sets the position of the pivot point.
		 * \param position [in] a 3d vector pointing to a position
		 */
		void setPivotPointPosition(const SimpleVector3 &position);

		/*!
		 * \return the position of the pivot point
		 */
		SimpleVector3 getPivotPointPosition();

		/*!
		 * \brief sets the orientation of the pivot point
		 * \param orientation [in] a quaternion which denotes the orientation
		 */
		void setOrientation(const SimpleQuaternion &orientation);

		/*!
		 * \return the orientation of the pivot point as quaternion
		 */
		SimpleQuaternion getOrientation();
	protected:
		/*!
		 * \brief convenience function to get a camera pose matrix.
		 * \param position [in] the position of the camera
		 * \param orientation [in] the orientation of the camera
		 */
		static SimpleMatrix4 getCameraPoseMatrix(const SimpleVector3 &position, const SimpleQuaternion &orientation);
	public:
		/*!
		 * \brief sets the horizontal field of view.
		 * \param fov the field of view in degrees
		 */
		void setHorizontalFOV(double fovDegrees);

		/*!
		 * \return the fov in degrees
		 */
		double getHorizontalFOV();

		/*!
		 * \brief sets the vertical field of view.
		 * \param fov the field of view in degrees
		 */
		void setVerticalFOV(double fovDegrees);

		/**
		 * \return the fov in degrees
		 */
		double getVerticalFOV();

		/*!
		 * \brief sets the near clipping plane.
		 * \param ncp the near clipping distance
		 */
		void setNearClippingPlane(double ncp);

		/**
		 * \return the near clipping distance
		 */
		double getNearClippingPlane();

		/*!
		 * \brief sets the far clipping plane.
		 * \paramparam fcp the far clipping distance
		 */
		void setFarClippingPlane(double fcp);

		/*!
		 * \return the far clipping distance
		 */
		double getFarClippingPlane();
	};

	/*!
	 * \brief Definition for the shared pointer type of the class.
	 */
	typedef boost::shared_ptr<CameraModelFilter> CameraModelFilterPtr;
}


#endif /* CAMERAMODELFILTER_HPP_ */
