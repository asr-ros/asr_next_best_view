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
