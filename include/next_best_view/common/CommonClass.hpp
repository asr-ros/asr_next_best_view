/*
 * CommonPCLClass.hpp
 *
 *  Created on: Oct 2, 2014
 *      Author: ralfschleicher
 */

#ifndef COMMONCLASS_HPP_
#define COMMONCLASS_HPP_

#include "typedef.hpp"

namespace next_best_view {
	class CommonClass {
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
		 * \brief ctor
		 */
		CommonClass();

		/*!
		 * \brief pure virtual dtor.
		 */
		virtual ~CommonClass() = 0;

		/*!
		 * \brief setting the input cloud.
		 * \param pointCloudPtr the shared pointer to point cloud
		 */
        virtual void setInputCloud(const ObjectPointCloudPtr &pointCloudPtr);

		/*!
		 * \return the shared pointer to the point cloud.
		 */
        virtual ObjectPointCloudPtr& getInputCloud();

		/*!
		 * \brief setting the shared pointer to the active indices of the point cloud.
		 * \param indicesPtr the shared pointer to indices.
		 */
        virtual void setIndices(const IndicesPtr &indicesPtr);

		/*!
		 * \return the shared pointer to the active indices of the point cloud.
		 */
        virtual IndicesPtr& getIndices();
	};
}


#endif /* COMMONPCLCLASS_HPP_ */
