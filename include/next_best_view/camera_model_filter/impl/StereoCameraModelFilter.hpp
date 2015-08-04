/*
 * StereoCameraModelFilter.hpp
 *
 *  Created on: Sep 25, 2014
 *      Author: ralfschleicher
 */

#ifndef STEREOCAMERAMODELFILTER_HPP_
#define STEREOCAMERAMODELFILTER_HPP_

#include "next_best_view/camera_model_filter/CameraModelFilter.hpp"
#include "next_best_view/camera_model_filter/impl/SingleCameraModelFilter.hpp"

namespace next_best_view {
	/*!
	 * \brief StereoCameraModelFilter class implements the frustum filtering for stereo cameras.
	 * \details The stereo camera model makes actually twice the use of the SingleCameraModelFilter. Therefore it may be possible to implement different camera orientations in here either.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
	class StereoCameraModelFilter : public CameraModelFilter {
	public:
		/*!
		 * \brief describes the filtering type of the filter.
		 * \details If LEFT or RIGHT is used, only the camera filter of the left or the right camera will be used. On BOTH the results of left and right will be combined. And on intersection only the points which are in the left and the right frustum will be minded.
		 */
		enum FilteringType {
			LEFT,
			RIGHT,
			BOTH,
			INTERSECTION
		};
	private:
		/*!
		 * \brief the filtering type.
		 */
		FilteringType mFilteringType;

		/*!
		 * \brief the left camera model filter.
		 */
		SingleCameraModelFilter mLeftCameraModelFilter;

		/*!
		 * \brief the right camera model filter.
		 */
		SingleCameraModelFilter mRightCameraModelFilter;
	public:
		/*!
		 * \brief constructor for the StereoCameraModelFilter object
		 * \param leftCameraPivotPointOffset [in] the offset to the pivot point for the left camera
		 * \param rightCameraPivotPointOffset [in] the offset to the pivot point for the right camera
		 */
		StereoCameraModelFilter(const SimpleVector3 &leftCameraPivotPointOffset = SimpleVector3(), const SimpleVector3 &rightCameraPivotPointOffset = SimpleVector3());
	protected:
		/*!
		 * \brief copies the settings of this objects to the settings of the two SingeCameraModelFilter objects.
		 */
		void copySettings();
	public:
		void do_filtering(IndicesPtr &indicesPtr);

		/*!
		 * \brief sets the filtering type.
		 * \param type the type of filtering
		 */
		void setFilteringType(FilteringType type);

		/**
		 * \result the filtering type
		 */
		FilteringType getFilteringType();

		/*!
		 * \brief sets the left camera pivot offset
		 * \param the offset of the left camera to the pivot point
		 */
		void setLeftCameraPivotPointOffset(const SimpleVector3 &cameraPivotPointOffset);

		/*!
		 * \return the left camera pivot offset
		 */
		SimpleVector3 getLeftCameraPivotPointOffset();

		/*!
		 * \brief sets the left camera pivot offset
		 * \param the offset of the left camera to the pivot point
		 */
		void setRightCameraPivotPointOffset(const SimpleVector3 &cameraPivotPointOffset);

		/*!
		 * \return the left camera pivot offset
		 */
		SimpleVector3 getRightCameraPivotPointOffset();

		viz::MarkerArrayPtr getVisualizationMarkerArray(uint32_t &sequence, double lifetime = 30.0);
	};

	/*!
	 * \brief the type definition for the corresponding shared pointer of the class.
	 */
	typedef boost::shared_ptr<StereoCameraModelFilter> StereoCameraModelFilterPtr;
}


#endif /* STEREOCAMERAMODELFILTER_HPP_ */
