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

#include <boost/foreach.hpp>
#include "next_best_view/hypothesis_updater/impl/PerspectiveHypothesisUpdater.hpp"

#include "next_best_view/helper/MathHelper.hpp"
#include "typedef.hpp"
#include <ros/ros.h>

namespace next_best_view {
    PerspectiveHypothesisUpdater::PerspectiveHypothesisUpdater() {
        mDebugHelperPtr = DebugHelper::getInstance();
    }

    PerspectiveHypothesisUpdater::~PerspectiveHypothesisUpdater() { }

    unsigned int PerspectiveHypothesisUpdater::update(const ObjectTypeSetPtr &objectTypeSetPtr, const ViewportPoint &viewportPoint) {
        mDebugHelperPtr->writeNoticeably("STARTING UPDATE METHOD", DebugHelper::HYPOTHESIS_UPDATER);

        unsigned int counter = 0;

        mDebugHelperPtr->write(std::stringstream() << "Child indices in viewport: " << viewportPoint.child_indices->size(), DebugHelper::HYPOTHESIS_UPDATER);

        BOOST_FOREACH(int index, *viewportPoint.child_indices) {
			ObjectPoint &objectPoint = viewportPoint.child_point_cloud->at(index);

            if (objectTypeSetPtr->find(objectPoint.type) == objectTypeSetPtr->end()) {
                continue;
            }

			Indices::iterator begin = objectPoint.active_normal_vectors->begin();
			Indices::iterator end = objectPoint.active_normal_vectors->end();
			for (Indices::iterator iter = begin; iter != end;) {
				int normalIndex = *iter;
				SimpleVector3 normalVector = objectPoint.normal_vectors->at(normalIndex);

                // rate angle of hypothesis
                SimpleQuaternion cameraOrientation = viewportPoint.getSimpleQuaternion();
                SimpleVector3 cameraOrientationVector = MathHelper::getVisualAxis(cameraOrientation);

                // rate the angle between the camera orientation and the object normal
                float rating = mDefaultRatingModulePtr->getNormalizedAngleUtility(-cameraOrientationVector, normalVector, mNormalAngleThreshold);

                mDebugHelperPtr->write(std::stringstream() << "Normal utility: " << rating, DebugHelper::HYPOTHESIS_UPDATER);

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

        mDebugHelperPtr->writeNoticeably("ENDING UPDATE METHOD", DebugHelper::HYPOTHESIS_UPDATER);

        return counter;
	}

    void PerspectiveHypothesisUpdater::setDefaultRatingModule(DefaultRatingModulePtr defaultRatingModulePtr) {
		mDefaultRatingModulePtr = defaultRatingModulePtr;
	}

	DefaultRatingModulePtr PerspectiveHypothesisUpdater::getDefaultRatingModule() {
		return mDefaultRatingModulePtr;
	}

    void PerspectiveHypothesisUpdater::setNormalAngleThreshold(double angle) {
        mNormalAngleThreshold = angle;
    }

    double PerspectiveHypothesisUpdater::getNormalAngleThreshold() {
        return mNormalAngleThreshold;
    }
}
