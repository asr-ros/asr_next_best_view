/*
 * CommonPCLClass.hpp
 *
 *  Created on: Oct 2, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/common/CommonPCLClass.hpp"

namespace next_best_view {
	/*!
	 * \brief setting the input cloud.
	 * \param pointCloudPtr the shared pointer to point cloud
	 */
	void CommonPCLClass::setInputCloud(const ObjectPointCloudPtr &pointCloudPtr) {
		mPointCloudPtr = pointCloudPtr;
	}

	/*!
	 * \return the shared pointer to the point cloud.
	 */
	ObjectPointCloudPtr CommonPCLClass::getInputCloud() {
		return mPointCloudPtr;
	}

	/*!
	 * \brief setting the shared pointer to the active indices of the point cloud.
	 * \param indicesPtr the shared pointer to indices.
	 */
	void CommonPCLClass::setIndices(const IndicesPtr &indicesPtr) {
		mIndicesPtr = indicesPtr;
	}

	/*!
	 * \return the shared pointer to the active indices of the point cloud.
	 */
	IndicesPtr CommonPCLClass::getIndices() {
		return mIndicesPtr;
	}
}
