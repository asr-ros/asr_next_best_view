/*
 * CommonPCLClass.hpp
 *
 *  Created on: Oct 2, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/common/CommonClass.hpp"

namespace next_best_view {
	CommonClass::CommonClass() { }

	CommonClass::~CommonClass() { }

	void CommonClass::setInputCloud(const ObjectPointCloudPtr &pointCloudPtr) {
		mPointCloudPtr = pointCloudPtr;
	}

	ObjectPointCloudPtr& CommonClass::getInputCloud() {
		return mPointCloudPtr;
	}

	void CommonClass::setIndices(const IndicesPtr &indicesPtr) {
		mIndicesPtr = indicesPtr;
	}

	IndicesPtr& CommonClass::getIndices() {
		return mIndicesPtr;
	}
}
