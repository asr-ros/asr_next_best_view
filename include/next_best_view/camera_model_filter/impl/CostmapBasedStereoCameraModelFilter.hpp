/*
 * CostmapBasedStereoCameraModelFilter.hpp
 *
 *  Created on: Oct 17, 2014
 *      Author: ralfschleicher
 */

#ifndef COSTMAPBASEDSTEREOCAMERAMODELFILTER_HPP_
#define COSTMAPBASEDSTEREOCAMERAMODELFILTER_HPP_

#include "next_best_view/camera_model_filter/CameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/impl/StereoCameraModelFilter.hpp"
#include "next_best_view/utility/MapUtility.hpp"

namespace next_best_view {
	/*!
	 * \brief StereoCameraModelFilter class implements the frustum filtering for stereo cameras.
	 * \details The stereo camera model makes actually twice the use of the SingleCameraModelFilter. Therefore it may be possible to implement different camera orientations in here either.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
	class CostmapBasedStereoCameraModelFilter : public CameraModelFilter {
	private:
		/*!
		 *
		 */
		MapUtilityPtr mMapUtilityPtr;

		/*!
		 * \brief the left camera model filter.
		 */
		StereoCameraModelFilter mCameraModelFilter;
	public:
		/*!
		 * \brief constructor for the StereoCameraModelFilter object
		 * \param leftCameraPivotPointOffset [in] the offset to the pivot point for the left camera
		 * \param rightCameraPivotPointOffset [in] the offset to the pivot point for the right camera
		 */
		CostmapBasedStereoCameraModelFilter(const MapUtilityPtr &mapUtilPtr, const SimpleVector3 &leftCameraPivotPointOffset = SimpleVector3(), const SimpleVector3 &rightCameraPivotPointOffset = SimpleVector3());
	private:
		/*!
		 * \brief copies the settings of this objects to the settings of the two SingeCameraModelFilter objects.
		 */
		void copySettings();
	public:
		void filter(IndicesPtr &indicesPtr);

		viz::MarkerArrayPtr getVisualizationMarkerArray(uint32_t &sequence, double lifetime = 30.0);
	};

	/*!
	 * \brief the type definition for the corresponding shared pointer of the class.
	 */
	typedef boost::shared_ptr<CostmapBasedStereoCameraModelFilter> CostmapBasedStereoCameraModelFilterPtr;
}


#endif /* COSTMAPBASEDSTEREOCAMERAMODELFILTER_HPP_ */
