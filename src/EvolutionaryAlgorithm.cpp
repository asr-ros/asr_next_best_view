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

#include "next_best_view/EvolutionaryAlgorithm.hpp"
#include "boost/range/irange.hpp"
#include <chrono>
#include <algorithm>

namespace next_best_view {

    EvolutionaryAlgorithm::EvolutionaryAlgorithm(NextBestViewCachePtr nbvCachePtr, const MapHelperPtr &mapHelperPtr, ClusterExtractionPtr clusterExtractionPtr, int improvementIterations, float improvementAngle, float radius, int minIterationEA, int nViewportsPerBB)
        : mNBVCachePtr(nbvCachePtr),
          mMapHelperPtr(mapHelperPtr),
          mClusterExtractionPtr(clusterExtractionPtr),
          mMinIterationEA(minIterationEA),
          mNViewportsPerBB(nViewportsPerBB) {
        setRadius(radius);
        setMaxAngle(improvementAngle);
        setImprovementIterationSteps(improvementIterations);
        setPositionOffsets();
    }

    ViewportPointCloudPtr EvolutionaryAlgorithm::selectAndMutate(const ViewportPointCloudPtr &samples, int iterationStep) {
        auto selectedViewports = select(samples, iterationStep);
        auto newViewports = mutate(selectedViewports, iterationStep);
        return newViewports;
    }

    ViewportPointCloudPtr EvolutionaryAlgorithm::select(const ViewportPointCloudPtr &in, int iterationStep) {
        ViewportPointCloudPtr result(new ViewportPointCloud());
        // TODO: we ignore in, which is the alternative if nbvCache is not used.
        ViewportPointCloudPtr bestUtilityViewports = mNBVCachePtr->getAllBestUtilityViewports();
        ViewportPointCloudPtr bestRatingViewports = mNBVCachePtr->getAllBestRatingViewports();
        *result += *selectFromSortedViewports(bestRatingViewports, iterationStep);
        *result += *selectFromSortedViewports(bestUtilityViewports, iterationStep);
        return result;
    }

    ViewportPointCloudPtr EvolutionaryAlgorithm::selectFromSortedViewports(const ViewportPointCloudPtr &population, int iterationStep) {
        ViewportPointCloudPtr selection(new ViewportPointCloud());

        // bbs, the location where selection must be in
        BoundingBoxPtrsPtr bbs = mClusterExtractionPtr->getClusters();

        // samplesPerBB[bbIdx] --> selected viewports in bb as vector<ViewportPoint>
        std::vector<std::vector<ViewportPoint>> samplesPerBB(bbs->size());
        for (unsigned int i = 0; i < bbs->size(); i++) {
            samplesPerBB[i] = std::vector<ViewportPoint>();
        }

        // this is a sorted vector of the best bbs, bbs are described by their index.
        // we just consider the best viewport of a bb.
        std::vector<int> bbIdxRatingOrder;

        // all selected viewports of all bbs, to check we don't select some that are too close to each other
        ViewportPointCloudPtr allSelectedViewports(new ViewportPointCloud());


        // some parameters to converge faster with increasing iterations.
        float selectedViewportsInterspacing = 0.2 * pow(2, -(iterationStep - mMinIterationEA));
        selectedViewportsInterspacing = max(0.2f, selectedViewportsInterspacing);
        unsigned int nViewportsToSelectPerBB = mNViewportsPerBB * pow(2, -(iterationStep - mMinIterationEA));
        nViewportsToSelectPerBB = max(static_cast<unsigned int>(1), nViewportsToSelectPerBB);
        BOOST_REVERSE_FOREACH (ViewportPoint &p, *population) {
            int bbIdx = 0;
            bool pHasHypothesisInMissingBB = false;
            bool haveEnoughSamples = true;
            // make sure new viewport is not too close to other selected viewports
            bool pIsTooCloseToAnotherSelected = false;
            for (ViewportPoint &goodHeuristicViewport : *allSelectedViewports) {
                if ((goodHeuristicViewport.getPosition() - p.getPosition()).lpNorm<2>() < selectedViewportsInterspacing) {
                    pIsTooCloseToAnotherSelected = true;
                    break;
                }
            }
            if (pIsTooCloseToAnotherSelected) {
                continue;
            }
            // check if viewport p covers a hypothesis in a bb with idx bbIdx where we need more samples.
            for (BoundingBoxPtr &bbPtr : *bbs) {
                if (samplesPerBB[bbIdx].size() >= nViewportsToSelectPerBB) {
                    bbIdx ++;
                    continue; // we have enough samples for the bb with idx bbIdx
                }
                for (int hypothesisIndex : *p.child_indices) {
                    ObjectPoint hypothesis = p.child_point_cloud->at(hypothesisIndex);
                    if (bbPtr->contains(hypothesis.getPosition())) {
                        if (std::find(bbIdxRatingOrder.begin(), bbIdxRatingOrder.end(), bbIdx) == bbIdxRatingOrder.end()) {
                            // if we havn't added a viewport to bb with idx bbIdx we add bbIdx to the end of bbIdxRatingOrder to get the order of rated bbs.
                            bbIdxRatingOrder.push_back(bbIdx);
                        }
                        samplesPerBB[bbIdx].push_back(p);
                        allSelectedViewports->push_back(p);
                        pHasHypothesisInMissingBB = true;
                        break;
                    }
                }
                if (pHasHypothesisInMissingBB) {
                    // we have found a bb that needs a viewport, so we skip other bbs
                    // since we assume bbs don't intersect.
                    // If they would intersect, they should be in the same cluster/bb.
                    break;
                }
                bbIdx ++;
            }
            // check if we have enough samples and can stop selection
            for (unsigned int i = 0; i < bbs->size(); i++) {
                if (samplesPerBB[i].size() < nViewportsToSelectPerBB) {
                    // if bb i does not have enough viewports, we still have to continue
                    haveEnoughSamples = false;
                    break;
                }
            }
            if (haveEnoughSamples) {
                break;
            }
        }
        // number of bbs where we select from
        int nBBsToSelectFrom = INT_MAX;
        if (iterationStep - mMinIterationEA > 3) {
            // if we iterated 4 or more times we generate samples from only 1 bb
            nBBsToSelectFrom = 1;
        }
        // number of bbs added to selection
        int nBBsAdded = 0;
        for (int &bbIdx : bbIdxRatingOrder) {
            std::vector<ViewportPoint> &samplesOfBB = samplesPerBB[bbIdx];
            if (nBBsAdded >= nBBsToSelectFrom) {
                // we selected enough bbs
                break;
            }
            // add all samples of the bb to selection.
            for (ViewportPoint vp : samplesOfBB) {
                selection->push_back(vp);
            }
            nBBsAdded++;
        }
        return selection;
    }

    ViewportPointCloudPtr EvolutionaryAlgorithm::mutate(const ViewportPointCloudPtr &selection, int iterationStep) {
        ViewportPointCloudPtr mutatedSamples(new ViewportPointCloud());
        for (ViewportPoint &goodRatedViewportPoint : *selection) {

            // get direction vector and rotationTransformations
            SimpleVector3 dirVector = MathHelper::quatToDir(goodRatedViewportPoint.getSimpleQuaternion());

            // with this loop we can adapt the range of modifications,
            // (0, mImprovementIterationSteps) would do all modifications in each step which is pretty costy and not really worth it < 0.01% improvement
            for (int i : boost::irange(iterationStep, iterationStep + 1)) {
                std::vector<SimpleQuaternion> rotationTransformations = calculateRotationTransformations(dirVector, i);
                std::vector<SimpleVector3> positionOffsets = mPositionOffsetsPerRadius.at(i).second;

                // do the mutation on goodRatedViewport
                for (SimpleVector3 &positionOffset : positionOffsets) { // for each position transformation
                    SimpleVector3 pos = goodRatedViewportPoint.getPosition() + positionOffset;
                    if (!mMapHelperPtr->isOccupancyValueAcceptable(mMapHelperPtr->getRaytracingMapOccupancyValue(pos))) {
                        // if the new position can't be reached we should not keep the mutation
                        continue;
                    }

                    for (SimpleQuaternion &rotationTransformation : rotationTransformations) { // for each rotation transformation
                        // SimpleVector3 orientation = rotationTransformation * dirVector;
                        SimpleQuaternion quat = rotationTransformation * goodRatedViewportPoint.getSimpleQuaternion();//MathHelper::dirToQuat(orientation);

                        // copy viewportPoint, but change quat
                        ViewportPoint viewport(pos, quat);
                        // determine real hypothesis/use kdtreefilter/is it worth? no it is not, no improvement at all
                        viewport.child_indices = goodRatedViewportPoint.child_indices;
                        viewport.point_cloud = goodRatedViewportPoint.point_cloud;
                        viewport.object_type_set = goodRatedViewportPoint.object_type_set;
                        viewport.oldIdx = 0;
                        mutatedSamples->push_back(viewport);
                    }
                }
            }
        }
        return mutatedSamples;
    }

    std::vector<SimpleQuaternion> EvolutionaryAlgorithm::calculateRotationTransformations(SimpleVector3 dirVector, int iterationStep) {
        double r = pow(1.4, -static_cast<double>(iterationStep)) * mMaxAngle;
        float degToRad = M_PI / 180.0;
        SimpleVector3 up(0, 0, 1);
        // upVector x dirVector is the vector where we have to rotate around
        SimpleVector3 rotAxis = up.cross(dirVector);
        std::vector<SimpleQuaternion> rotationTransformations;
        SimpleQuaternion h1, h2, v1, v2, id;
        h1 = Eigen::AngleAxisf(-r * degToRad, rotAxis);
        h2 = Eigen::AngleAxisf( r * degToRad, rotAxis);
        v1 = Eigen::AngleAxisf(-r * degToRad, up);
        v2 = Eigen::AngleAxisf( r * degToRad, up);
        id = Eigen::AngleAxisf( 0, up);
        rotationTransformations.push_back(h1);
        rotationTransformations.push_back(h2);
        rotationTransformations.push_back(v1);
        rotationTransformations.push_back(v2);
        rotationTransformations.push_back(h1 * v1);
        rotationTransformations.push_back(h2 * v1);
        rotationTransformations.push_back(h1 * v2);
        rotationTransformations.push_back(h2 * v2);
        rotationTransformations.push_back(id);
        return rotationTransformations;
    }

    void EvolutionaryAlgorithm::setPositionOffsets() {
        // to transform degree to radians, because degree angles are more readable
        float degToRad = M_PI / 180.0;

        // cos (30°), used to generate width of hexagon
        float cos30 = cos(30 * degToRad);

        for (int i : boost::irange(0, mImprovementIterationSteps)) {
            // we reduce radius exponentially
            double r = pow(1.4, -static_cast<double>(i)) * mRadius;

            // width of hexagon with radius r
            float horizontalSpacing = cos30 * r;

            // up is horizontal rotation/movement (x value)
            // ortho is vertical rotation/movement (y value)
            // this creates rotation matrices that rotate 2d around 2 axis
            // the angles per axis are based on the (x,y) position values of a hexagon
            std::vector<SimpleVector3> positionOffsets;
            positionOffsets.push_back(SimpleVector3(0,  r, 0));
            positionOffsets.push_back(SimpleVector3(0, -r, 0));
            positionOffsets.push_back(SimpleVector3( horizontalSpacing,  r / 2.0, 0));
            positionOffsets.push_back(SimpleVector3( horizontalSpacing, -r / 2.0, 0));
            positionOffsets.push_back(SimpleVector3(-horizontalSpacing,  r / 2.0, 0));
            positionOffsets.push_back(SimpleVector3(-horizontalSpacing, -r / 2.0, 0));
            positionOffsets.push_back(SimpleVector3(0, 0, 0));

            mPositionOffsetsPerRadius.push_back(std::make_pair(r, positionOffsets));
        }
    }

    float EvolutionaryAlgorithm::getMaxAngle() const {
        return mMaxAngle;
    }

    void EvolutionaryAlgorithm::setMaxAngle(float maxAngle) {
        mMaxAngle = maxAngle;
    }

    float EvolutionaryAlgorithm::getRadius() const {
        return mRadius;
    }

    void EvolutionaryAlgorithm::setRadius(float radius) {
        mRadius = radius;
    }

    int EvolutionaryAlgorithm::getImprovementIterationSteps() const {
        return mImprovementIterationSteps;
    }

    void EvolutionaryAlgorithm::setImprovementIterationSteps(int iterationImprovmentSteps) {
        mImprovementIterationSteps = iterationImprovmentSteps;
    }

    int EvolutionaryAlgorithm::getNViewportsPerBB() const {
        return mNViewportsPerBB;
    }

    void EvolutionaryAlgorithm::setNViewportsPerBB(int nViewportsPerBB) {
        mNViewportsPerBB = nViewportsPerBB;
    }
}
