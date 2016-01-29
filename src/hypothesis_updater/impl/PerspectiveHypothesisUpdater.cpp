/*
 * PerspectiveDependantHypothesisUpdater.cpp
 *
 *  Created on: Oct 2, 2014
 *      Author: ralfschleicher
 */

#include <boost/foreach.hpp>
#include "next_best_view/hypothesis_updater/impl/PerspectiveHypothesisUpdater.hpp"

#include "next_best_view/helper/MathHelper.hpp"
#include "typedef.hpp"
#include <ros/ros.h>

namespace next_best_view {
	PerspectiveHypothesisUpdater::PerspectiveHypothesisUpdater() { }
	PerspectiveHypothesisUpdater::~PerspectiveHypothesisUpdater() { }

    unsigned int PerspectiveHypothesisUpdater::update(const ViewportPoint &viewportPoint) {
        unsigned int counter = 0;

        BOOST_FOREACH(int index, *viewportPoint.child_indices) {
			ObjectPoint &objectPoint = viewportPoint.child_point_cloud->at(index);

			Indices::iterator begin = objectPoint.active_normal_vectors->begin();
			Indices::iterator end = objectPoint.active_normal_vectors->end();
			for (Indices::iterator iter = begin; iter != end;) {
				int normalIndex = *iter;
				SimpleVector3 normalVector = objectPoint.normal_vectors->at(normalIndex);

                float rating = mDefaultRatingModulePtr->getNormalUtility(viewportPoint, normalVector);
				if (rating != 0.0) {
					end--;
					std::iter_swap(iter, end);
                    counter++;
					continue;
				}

				iter++;
			}

			std::size_t iteratorDistance = std::distance(begin, end);
			//ROS_INFO("Resizing active normal vectors from %d to %d at index %d", objectPoint.active_normal_vectors->size(), iteratorDistance, index);
			objectPoint.active_normal_vectors->resize(iteratorDistance);

			// RGB
			double ratio = double(objectPoint.active_normal_vectors->size()) / double(objectPoint.normal_vectors->size());

			objectPoint.r = ratio > .5 ? int((2.0 - 2 * ratio) * 255) : 255;
			objectPoint.g = ratio > .5 ? 255 : int(2.0  * ratio * 255);
			objectPoint.b = 0;
		}

        return counter;
	}

	void PerspectiveHypothesisUpdater::setDefaultRatingModule(const DefaultRatingModulePtr &defaultRatingModulePtr) {
		mDefaultRatingModulePtr = defaultRatingModulePtr;
	}

	DefaultRatingModulePtr PerspectiveHypothesisUpdater::getDefaultRatingModule() {
		return mDefaultRatingModulePtr;
	}
}
