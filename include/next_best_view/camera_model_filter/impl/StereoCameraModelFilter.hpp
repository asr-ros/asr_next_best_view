/**

Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Braun Kai, Heller Florian, Hutmacher Robin, Karrenbauer Oliver, Marek Felix, Mayr Matthias, Mehlhaus Jonas, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Stroh Daniel, Trautmann Jeremias, Walter Milena
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

4. The use is explicitly not permitted to any application which deliberately try to kill or do harm to any living creature.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#pragma once

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
        void doFiltering(IndicesPtr &indicesPtr);

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
