/*
 * CostmapBasedStereoCameraModelFilter.hpp
 *
 *  Created on: Oct 17, 2014
 *      Author: ralfschleicher
 */

#ifndef MAPBASEDSTEREOCAMERAMODELFILTER_HPP_
#define MAPBASEDSTEREOCAMERAMODELFILTER_HPP_

#include "next_best_view/camera_model_filter/impl/StereoCameraModelFilter.hpp"
#include "next_best_view/helper/MapHelper.hpp"

namespace next_best_view {
	/*!
	 * \brief StereoCameraModelFilter class implements the frustum filtering for stereo cameras.
	 * \details The stereo camera model makes actually twice the use of the SingleCameraModelFilter. Therefore it may be possible to implement different camera orientations in here either.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
	class MapBasedStereoCameraModelFilter : public StereoCameraModelFilter {
	private:
		/*!
		 *
		 */
		MapHelperPtr mMapHelperPtr;
	public:
		/*!
		 * \brief constructor for the StereoCameraModelFilter object
		 * \param leftCameraPivotPointOffset [in] the offset to the pivot point for the left camera
		 * \param rightCameraPivotPointOffset [in] the offset to the pivot point for the right camera
		 */
		MapBasedStereoCameraModelFilter(const MapHelperPtr &mapHelperPtr, const SimpleVector3 &leftCameraPivotPointOffset = SimpleVector3(), const SimpleVector3 &rightCameraPivotPointOffset = SimpleVector3());
	public:
		void do_filtering(IndicesPtr &indicesPtr);

		viz::MarkerArrayPtr getVisualizationMarkerArray(uint32_t &sequence, double lifetime = 30.0);
	};

	/*!
	 * \brief the type definition for the corresponding shared pointer of the class.
	 */
	typedef boost::shared_ptr<MapBasedStereoCameraModelFilter> MapBasedStereoCameraModelFilterPtr;
}


#endif /* MAPBASEDSTEREOCAMERAMODELFILTER_HPP_ */
