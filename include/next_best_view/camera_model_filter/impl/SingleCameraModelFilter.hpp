/*
 * SingleCameraModelFilter.hpp
 *
 *  Created on: Sep 25, 2014
 *      Author: ralfschleicher
 */

#pragma once

#include "next_best_view/camera_model_filter/CameraModelFilter.hpp"
#include <pcl/filters/impl/frustum_culling.hpp>

namespace next_best_view {
	/*!
	 * \brief SingleCameraModelFilter class implements the frustum filter for a single camera.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
	class SingleCameraModelFilter : public CameraModelFilter {
	private:
		/*!
		 * \brief the offset to the pivot point.
		 */
		SimpleVector3 mPivotPointOffset;

		/*!
		 * \brief shared pointer to the frustum culling object.
		 */
		FrustumCullingPtr mFrustumCullingPtr;
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	public:
		/*!
		 * \brief constructs a new camera model filter object.
		 * \param pivotPointOffset the offset to the pivot point
		 */
		SingleCameraModelFilter(const SimpleVector3 pivotPointOffset = SimpleVector3());

		/*!
		 * \brief destructs the single camera model filter object.
		 */
		virtual ~SingleCameraModelFilter();

        void doFiltering(IndicesPtr &indicesPtr);

		/*!
		 * \brief sets the offset between the pivot point and the actual camera position.
		 * \param pivotPointOffset [in] the pivot point offset.
		 */
		void setPivotPointOffset(const SimpleVector3 &pivotPointOffset);

		/**
		 * \return the offset to the pivot point
		 */
		SimpleVector3 getPivotPointOffset();

		viz::MarkerArrayPtr getVisualizationMarkerArray(uint32_t &sequence, double lifetime = 30.0);
	};

	/*!
	 * \brief the type definition for the corresponding shared pointer of the class.
	 */
	typedef boost::shared_ptr<SingleCameraModelFilter> SingleCameraModelFilterPtr;
}
