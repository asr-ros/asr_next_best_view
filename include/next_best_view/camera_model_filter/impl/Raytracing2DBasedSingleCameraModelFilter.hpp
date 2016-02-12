/*
 * CostmapBasedStereoCameraModelFilter.hpp
 *
 *  Created on: Oct 17, 2014
 *      Author: ralfschleicher
 */

#pragma once

#include "next_best_view/camera_model_filter/impl/SingleCameraModelFilter.hpp"
#include "next_best_view/helper/MapHelper.hpp"

namespace next_best_view {
	/*!
     * \brief Raytracing2DBasedSingleCameraModelFilter class implements the frustum filtering for a single camera with 2D raytracing.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
    class Raytracing2DBasedSingleCameraModelFilter : public SingleCameraModelFilter {
	private:
		/*!
		 *
		 */
		MapHelperPtr mMapHelperPtr;
	public:
		/*!
         * \brief constructor for the Raytracing2DBasedSingleCameraModelFilter object
		 * \param leftCameraPivotPointOffset [in] the offset to the pivot point for the left camera
		 * \param rightCameraPivotPointOffset [in] the offset to the pivot point for the right camera
		 */
        Raytracing2DBasedSingleCameraModelFilter(const MapHelperPtr &mapHelperPtr, const SimpleVector3 &pivotPointOffset = SimpleVector3());
	public:
		void doFiltering(IndicesPtr &indicesPtr);

		viz::MarkerArrayPtr getVisualizationMarkerArray(uint32_t &sequence, double lifetime = 30.0);
	};

	/*!
	 * \brief the type definition for the corresponding shared pointer of the class.
	 */
    typedef boost::shared_ptr<Raytracing2DBasedSingleCameraModelFilter> MapBasedSingleCameraModelFilterPtr;
}
