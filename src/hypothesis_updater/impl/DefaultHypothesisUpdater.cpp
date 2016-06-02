/*
 * DefaultHypothesisUpdater.cpp
 *
 *  Created on: Oct 2, 2014
 *      Author: ralfschleicher
 */

#include "next_best_view/hypothesis_updater/impl/DefaultHypothesisUpdater.hpp"

namespace next_best_view {
	DefaultHypothesisUpdater::DefaultHypothesisUpdater() { }
	DefaultHypothesisUpdater::~DefaultHypothesisUpdater() { }

    unsigned int DefaultHypothesisUpdater::update(const ObjectTypeSetPtr &objectTypeSetPtr, const ViewportPoint &viewportPoint) {
        unsigned int count = 0;

        BOOST_FOREACH(int index, *viewportPoint.child_indices) {
			ObjectPoint &objectPoint = viewportPoint.child_point_cloud->at(index);
            if (objectTypeSetPtr->find(objectPoint.type) == objectTypeSetPtr->end()) {
                continue;
            }
            count += objectPoint.active_normal_vectors->size();
			objectPoint.active_normal_vectors->clear();
			//objectPoint.normal_vectors = SimpleVector3CollectionPtr(new SimpleVector3Collection());
		}

        return count;
	}
}
