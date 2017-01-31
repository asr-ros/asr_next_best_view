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


#include "next_best_view/space_sampler/impl/HypothesisSpaceSampler.hpp"
#include "next_best_view/pcl/BoundingBox.hpp"

#include "next_best_view/pcl/BoundingBox.hpp"
#include <pcl-1.7/pcl/kdtree/kdtree.h>
#include <pcl-1.7/pcl/segmentation/extract_clusters.h>

#include <chrono>

namespace next_best_view {
    HypothesisSpaceSampler::HypothesisSpaceSampler(const ClusterExtractionPtr &clusterExtractionPtr, const SpaceSamplePatternPtr &spaceSamplePattern, double offset)
        : mHypothesesClusterExtractionPtr(clusterExtractionPtr),
          mSpaceSamplePattern(spaceSamplePattern),
          mOffset(offset) {
        mDebugHelperPtr = DebugHelper::getInstance();
    }

    HypothesisSpaceSampler::~HypothesisSpaceSampler() { }

    SamplePointCloudPtr HypothesisSpaceSampler::getSampledSpacePointCloud(SimpleVector3 currentSpacePosition, float contractor) {
        SamplePointCloudPtr pointCloud = SamplePointCloudPtr(new SamplePointCloud());

        BoundingBoxPtrsPtr hypothesesBBPtrsPtr = mHypothesesClusterExtractionPtr->getClusters();
        // expand clusters
        BoundingBoxPtrsPtr expandedBBPtrsPtr(new BoundingBoxPtrs());
        for (BoundingBoxPtr &bbPtr : *hypothesesBBPtrsPtr) {
            BoundingBoxPtr expandedBBPtr(new BoundingBox(*bbPtr));
            expandedBBPtr->expand(SimpleVector3(mOffset, mOffset, 0));
            expandedBBPtr->ignoreAxis(2);
            expandedBBPtrsPtr->push_back(expandedBBPtr);
        }

        // a bounding box over all bounding boxes to determine the area where we create samples
        // it might be a bit more efficient if we sample inside the bounding boxes and filter overlapping samples
        // worst case is a sparse point cloud with far distributed samples
        BoundingBoxPtr samplingBB(new BoundingBox(expandedBBPtrsPtr));
        pointCloud = this->generateSamples(samplingBB, currentSpacePosition, contractor);
        pointCloud->push_back(currentSpacePosition);

        // enable this if you have clusters which can hardly be reached by the robot.
//        addSamplesIfTooFew(expandedBBPtrsPtr, pointCloud, currentSpacePosition, contractor);

        return pointCloud;
    }

    SamplePointCloudPtr HypothesisSpaceSampler::generateSamples(const BoundingBoxPtr &bb, SimpleVector3 currentSpacePosition, float factor) {
        SamplePointCloudPtr samples;
        mSpaceSamplePattern->setPatternSizeFactor(factor);
        samples = mSpaceSamplePattern->getSampledSpacePointCloud(currentSpacePosition, bb);
        if (bb->contains(currentSpacePosition)) {
            samples->push_back(currentSpacePosition);
        }
        return samples;
    }

    void HypothesisSpaceSampler::addSamplesIfTooFew(BoundingBoxPtrsPtr expandedBBPtrsPtr, SamplePointCloudPtr pointCloud, SimpleVector3 currentSpacePosition, float contractor) {
        // number of generated samples per bounding box
        // sum of those might be > pointCloud.size()
        std::vector<int> nSamplesPerBB(expandedBBPtrsPtr->size());
        for (SamplePoint sample : *pointCloud) {
            int bbIdx = 0;
            // since bbs might be overlapping we go through all bbs for each added point
            for (BoundingBoxPtr &bbPtr : *expandedBBPtrsPtr) {
                if (bbPtr->contains(sample.getSimpleVector3())) {
                    nSamplesPerBB[bbIdx] ++;
                }
                bbIdx++;
            }
        }

        // we add additional samples, if we have too few for a BB
        int bbIdx = 0;
        for (int nSamples : nSamplesPerBB) {
            if (nSamples < 5) {
                ROS_INFO_STREAM("we have too few samples for a bb");
                ROS_INFO_STREAM("bb with idx " << bbIdx << " has " << nSamplesPerBB[bbIdx] << " samples.");
                ROS_INFO_STREAM("bb: " << expandedBBPtrsPtr->at(bbIdx));
                // generate new samples and add them to pointcloud/result
                SamplePointCloudPtr samples = this->generateSamples(expandedBBPtrsPtr->at(bbIdx), currentSpacePosition, contractor * 0.5);
                nSamplesPerBB[bbIdx] += samples->size();
                (*pointCloud) += (*samples);
            }
            bbIdx ++;
        }
    }
}


