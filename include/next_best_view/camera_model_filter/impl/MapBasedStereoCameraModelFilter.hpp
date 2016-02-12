/*
 * CostmapBasedStereoCameraModelFilter.hpp
 *
 *  Created on: Oct 17, 2014
 *      Author: ralfschleicher
 */

#pragma once

#include "next_best_view/camera_model_filter/impl/StereoCameraModelFilter.hpp"
#include "next_best_view/helper/MapHelper.hpp"

namespace next_best_view {
	/*!
     * \brief Raytracing2DBasedStereoCameraModelFilter class implements the frustum filtering for stereo cameras  with 2D raytracing.
	 * \details The stereo camera model makes actually twice the use of the SingleCameraModelFilter. Therefore it may be possible to implement different camera orientations in here either.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
    class Raytracing2DBasedStereoCameraModelFilter : public StereoCameraModelFilter {
	private:
		/*!
		 *
		 */
		MapHelperPtr mMapHelperPtr;
	public:
		/*!
         * \brief constructor for the Raytracing2DBasedStereoCameraModelFilter object
		 * \param leftCameraPivotPointOffset [in] the offset to the pivot point for the left camera
		 * \param rightCameraPivotPointOffset [in] the offset to the pivot point for the right camera
		 */
        Raytracing2DBasedStereoCameraModelFilter(const MapHelperPtr &mapHelperPtr, const SimpleVector3 &leftCameraPivotPointOffset = SimpleVector3(), const SimpleVector3 &rightCameraPivotPointOffset = SimpleVector3());
	public:
		void doFiltering(IndicesPtr &indicesPtr);

		viz::MarkerArrayPtr getVisualizationMarkerArray(uint32_t &sequence, double lifetime = 30.0);
	};

	/*!
	 * \brief the type definition for the corresponding shared pointer of the class.
	 */
    typedef boost::shared_ptr<Raytracing2DBasedStereoCameraModelFilter> MapBasedStereoCameraModelFilterPtr;
}
