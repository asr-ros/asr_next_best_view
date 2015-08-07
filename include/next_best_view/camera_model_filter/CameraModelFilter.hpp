/*
 * CameraModel.hpp
 *
 *  Created on: Sep 25, 2014
 *      Author: ralfschleicher
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
