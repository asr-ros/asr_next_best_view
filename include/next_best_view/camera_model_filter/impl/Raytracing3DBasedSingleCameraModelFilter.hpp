#pragma once

#include <boost/foreach.hpp>
#include "next_best_view/camera_model_filter/impl/SingleCameraModelFilter.hpp"
#include "next_best_view/helper/WorldHelper.hpp"
#include "typedef.hpp"

namespace next_best_view {
	/*!
     * \brief Raytracing3DBasedSingleCameraModelFilter class implements the frustum filtering for a single camera with 2D raytracing.
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 */
    class Raytracing3DBasedSingleCameraModelFilter : public SingleCameraModelFilter {
	private:
		/*!
		 *
		 */
        WorldHelperPtr mWorldHelperPtr;
	public:
		/*!
         * \brief constructor for the Raytracing3DBasedSingleCameraModelFilter object
		 * \param leftCameraPivotPointOffset [in] the offset to the pivot point for the left camera
		 * \param rightCameraPivotPointOffset [in] the offset to the pivot point for the right camera
		 */
        Raytracing3DBasedSingleCameraModelFilter(const WorldHelperPtr &worldHelperPtr, const SimpleVector3 &pivotPointOffset = SimpleVector3());
	public:
		void doFiltering(IndicesPtr &indicesPtr);

	};

	/*!
	 * \brief the type definition for the corresponding shared pointer of the class.
	 */
    typedef boost::shared_ptr<Raytracing3DBasedSingleCameraModelFilter> Raytracing3DBasedSingleCameraModelFilterPtr;
}
