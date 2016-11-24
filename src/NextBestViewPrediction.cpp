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

#include "next_best_view/NextBestViewPrediction.hpp"

namespace next_best_view {


    NextBestViewPrediction::NextBestViewPrediction(NextBestViewCalculatorPtr nbvCalcPtr, const VisualizationHelperPtr &visHelperPtr)
        : mNBVCalcPtr(nbvCalcPtr),
          mVisHelperPtr(visHelperPtr)
    { }

    std::vector<Indices> NextBestViewPrediction::saveNormals() {
        ObjectPointCloudPtr objectPointCloudPtr = mNBVCalcPtr->getPointCloudPtr();
        IndicesPtr activeIndicesPtr = mNBVCalcPtr->getActiveIndices();
        std::vector<Indices> activeNormalsPerObject;
        for (int objIdx : *activeIndicesPtr) {
            // add copy of active indics to activeNormalsPerObject
            activeNormalsPerObject.push_back(Indices(*objectPointCloudPtr->at(objIdx).active_normal_vectors));
        }
        return activeNormalsPerObject;
    }

    void NextBestViewPrediction::restoreNormals(std::vector<Indices> activeNormalsPerObject) {
        ObjectPointCloudPtr objectPointCloudPtr = mNBVCalcPtr->getPointCloudPtr();
        IndicesPtr activeIndicesPtr = mNBVCalcPtr->getActiveIndices();
        unsigned int i = 0;
        for (int objIdx : *activeIndicesPtr) {
            // add copy of active indics to activeNormalsPerObject
            if (i < activeNormalsPerObject.size()) {
                objectPointCloudPtr->at(objIdx).active_normal_vectors = IndicesPtr(new Indices(activeNormalsPerObject[i]));
            } else {
                ROS_ERROR_STREAM("idx should be valid, otherwise pc was modified");
            }
            i++;
        }
    }

    ViewportPointCloudPtr NextBestViewPrediction::getNBVPredictions(ViewportPoint startCameraViewport) {
        // save normals, to restore later
        std::vector<Indices> normals = saveNormals();
        ViewportPointCloudPtr result(new ViewportPointCloud());
        ViewportPoint currentCameraViewport = startCameraViewport;
        Indices totalSeenHypotheses;
        float totalNumberHypotheses = mNBVCalcPtr->getActiveIndices()->size();
        if (totalNumberHypotheses == 0.0) {
            ROS_ERROR_STREAM("no hypothesis");
            return ViewportPointCloudPtr();
        }

        // TODO: maybe per object type? /filter obejct if they have 0 normals
        // TODO: maybe try to avoid to look at same hypothesis multiple times/get intersection totalSeenHypothesis and currentSeenHypotheses
        // TODO: parameter 0.7
        while (static_cast<float>(totalSeenHypotheses.size()) / totalNumberHypotheses < 0.7) {
            // get nbv
            ViewportPoint resultViewport;
            if (!mNBVCalcPtr->calculateNextBestView(currentCameraViewport, resultViewport)) {
                // no nbv found
                break;
            }

            // set nbv as current view
            currentCameraViewport = resultViewport;

            // remove normals
            mNBVCalcPtr->updateObjectPointCloud(resultViewport.object_type_set, resultViewport);

            // update seen hypothesis
            Indices currentSeenHypotheses = *resultViewport.child_indices;
            std::sort(currentSeenHypotheses.begin(), currentSeenHypotheses.end());
            if (totalSeenHypotheses.empty()) {
                totalSeenHypotheses = currentSeenHypotheses;
            } else {
                Indices newTotalSeenHypotheses(currentSeenHypotheses.size() + totalSeenHypotheses.size());
                auto it = std::set_union(totalSeenHypotheses.begin(), totalSeenHypotheses.end(), currentSeenHypotheses.begin(), currentSeenHypotheses.end(), newTotalSeenHypotheses.begin());
                // shrink hypothesis size
                newTotalSeenHypotheses.resize(it - newTotalSeenHypotheses.begin());
                totalSeenHypotheses = newTotalSeenHypotheses;
            }
            ROS_INFO_STREAM("prediction sees now " << static_cast<float>(totalSeenHypotheses.size()) / totalNumberHypotheses << "% of all hypotheses");

            // save nbv to list of nbvs
            result->push_back(resultViewport);
        }

        ROS_INFO_STREAM("result prediction sees " << static_cast<float>(totalSeenHypotheses.size()) / totalNumberHypotheses << "% of all hypotheses");

        // restore normals, so it looks like we never called updateObjectPointCloud
        restoreNormals(normals);

        // visualize
        mVisHelperPtr->triggerViewportsVisualization(result, Color(1, 1, 1, 1), "nnbvs");
        return result;
    }
}

