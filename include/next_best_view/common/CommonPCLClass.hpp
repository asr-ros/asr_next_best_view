/*
 * CommonPCLClass.hpp
 *
 *  Created on: Oct 2, 2014
 *      Author: ralfschleicher
 */

#ifndef COMMONPCLCLASS_HPP_
#define COMMONPCLCLASS_HPP_

#include "typedef.hpp"

namespace next_best_view {
	class CommonPCLClass {
	private:
		/*!
		 * \brief shared pointer to the object point cloud.
		 */
		ObjectPointCloudPtr mPointCloudPtr;

		/*!
		 * \brief shared pointer to the active indices of the former declared object point cloud.
		 */
		IndicesPtr mIndicesPtr;

	public:
		/*!
		 * \brief setting the input cloud.
		 * \param pointCloudPtr the shared pointer to point cloud
		 */
		void setInputCloud(const ObjectPointCloudPtr &pointCloudPtr);

		/*!
		 * \return the shared pointer to the point cloud.
		 */
		ObjectPointCloudPtr getInputCloud();

		/*!
		 * \brief setting the shared pointer to the active indices of the point cloud.
		 * \param indicesPtr the shared pointer to indices.
		 */
		void setIndices(const IndicesPtr &indicesPtr);

		/*!
		 * \return the shared pointer to the active indices of the point cloud.
		 */
		IndicesPtr getIndices();
	};
}


#endif /* COMMONPCLCLASS_HPP_ */
