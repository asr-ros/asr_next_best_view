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
    HypothesisSpaceSampler::HypothesisSpaceSampler(const MapHelperPtr &mapUtilityPtr, const SpaceSamplePatternPtr &spaceSamplePattern, double offset) :  mMapHelperPtr(mapUtilityPtr), mSpaceSamplePattern(spaceSamplePattern), mOffset(offset) {
        mDebugHelperPtr = DebugHelper::getInstance();
    }

    HypothesisSpaceSampler::~HypothesisSpaceSampler() { }

    SamplePointCloudPtr HypothesisSpaceSampler::getSampledSpacePointCloud(SimpleVector3 currentSpacePosition, float contractor) {
        SamplePointCloudPtr pointCloud = SamplePointCloudPtr(new SamplePointCloud());

        SamplePointCloudPtr samples = this->generateSamples(mSamplingBB, currentSpacePosition, contractor);
        samples->push_back(currentSpacePosition);

        // filter samples by map and bb
        for (SamplePoint generatedSample : *samples) {
            // check if generatedSample is in a bb
            bool generatedSampleIsInABB = false;
            for (BoundingBoxPtr &bbPtr : mBBs) {
                if (bbPtr->contains(generatedSample.getSimpleVector3())) {
                    generatedSampleIsInABB = true;
                    break;
                }
            }
            // check for occupancy value
            if (generatedSampleIsInABB &&
                    mMapHelperPtr->isOccupancyValueAcceptable(mMapHelperPtr->getRaytracingMapOccupancyValue(generatedSample.getSimpleVector3()))) {
                pointCloud->push_back(generatedSample);
            }
        }

        // number of generated samples per bounding box
        // sum of those is more than pointCloud.size()
        std::vector<int> nSamplesPerBB(mBBs.size());
        for (SamplePoint sample : *pointCloud) {
            int bbIdx = 0;
            // since bbs might be overlapping we go through all bbs for each added point
            for (BoundingBoxPtr &bbPtr : mBBs) {
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
                ROS_INFO_STREAM("bb: " << mBBs[bbIdx]);
                samples = this->generateSamples(mBBs[bbIdx], currentSpacePosition, contractor * 0.5);
                if (mBBs[bbIdx]->contains(currentSpacePosition)) {
                    samples->push_back(currentSpacePosition);
                }
                for (SamplePoint generatedSample : *samples) {
                    if (mMapHelperPtr->isOccupancyValueAcceptable(mMapHelperPtr->getRaytracingMapOccupancyValue(generatedSample.getSimpleVector3()))) {
                        nSamplesPerBB[bbIdx] ++;
                        pointCloud->push_back(generatedSample);
                    }
                }
            }
            bbIdx ++;
        }

        return pointCloud;
    }

    SamplePointCloudPtr HypothesisSpaceSampler::generateSamples(const BoundingBoxPtr &bb, SimpleVector3 currentSpacePosition, float factor) {
        SamplePointCloudPtr samples;
        mSpaceSamplePattern->setPatternSizeFactor(factor);
        samples = mSpaceSamplePattern->getSampledSpacePointCloud(currentSpacePosition, bb);
        return samples;
    }

    void HypothesisSpaceSampler::setInputCloud(const ObjectPointCloudPtr &pointCloudPtr) {
        CommonClass::setInputCloud(pointCloudPtr);

        // kd tree, used by cluster extraction to search nearby hypothesis
        pcl::search::KdTree<ObjectPoint>::Ptr tree (new pcl::search::KdTree<ObjectPoint>);
        tree->setInputCloud(pointCloudPtr);

        // find hypothesis clusters
        auto begin = std::chrono::high_resolution_clock::now();
        std::vector<pcl::PointIndices> clusterIndices;
        pcl::EuclideanClusterExtraction<ObjectPoint> euclideanClusterExtractor;
        euclideanClusterExtractor.setClusterTolerance (0.3); // 30cm
        euclideanClusterExtractor.setMinClusterSize (10);
        euclideanClusterExtractor.setMaxClusterSize (25000);
        euclideanClusterExtractor.setSearchMethod(tree);
        euclideanClusterExtractor.setInputCloud(pointCloudPtr);
        euclideanClusterExtractor.extract(clusterIndices);
        auto finish = std::chrono::high_resolution_clock::now();
        mDebugHelperPtr->writeNoticeably(std::stringstream() << "cluster extraction took " << std::chrono::duration<float>(finish - begin).count() << " seconds.", DebugHelper::CALCULATION);
        mDebugHelperPtr->writeNoticeably(std::stringstream() << "number of clusters: " << clusterIndices.size(), DebugHelper::CALCULATION);
        ROS_INFO_STREAM("number of clusters: " << clusterIndices.size());

        // determine bb per cluster
        int i = 0;
        for (pcl::PointIndices &indices : clusterIndices) {
            ObjectPointCloudPtr clusterPC(new ObjectPointCloud(*pointCloudPtr, indices.indices));
            ObjectPoint minPoint, maxPoint;
            pcl::getMinMax3D(*clusterPC, minPoint, maxPoint);
            mDebugHelperPtr->writeNoticeably(std::stringstream() << "cluster " << i << ": has " << indices.indices.size() << " object points", DebugHelper::CALCULATION);
            mDebugHelperPtr->writeNoticeably(std::stringstream() << "minPoint: " << minPoint, DebugHelper::CALCULATION);
            mDebugHelperPtr->writeNoticeably(std::stringstream() << "maxPoint: " << maxPoint, DebugHelper::CALCULATION);
            // we are only interested in 2d map (x/y)
            minPoint.z = 0;
            maxPoint.z = 0;
            // increase bb of cluster by an offset (2 * fcp)
            mBBs.push_back(BoundingBoxPtr(new BoundingBox(minPoint.getPosition() - SimpleVector3(mOffset, mOffset, 0), maxPoint.getPosition() + SimpleVector3(mOffset, mOffset, 0))));
            i++;
        }

        // get bb of all cluster bounding boxes
        float minX = mBBs[0]->minPos[0];
        float minY = mBBs[0]->minPos[1];
        float maxX = mBBs[0]->maxPos[0];
        float maxY = mBBs[0]->maxPos[1];
        for (BoundingBoxPtr &bbPtr : mBBs) {
            minX = min(bbPtr->minPos[0], minX);
            minY = min(bbPtr->minPos[1], minY);
            maxX = min(bbPtr->maxPos[0], maxX);
            maxY = min(bbPtr->maxPos[1], maxY);
        }
        mSamplingBB = BoundingBoxPtr(new BoundingBox(SimpleVector3(minX, minY, 0), SimpleVector3(maxX, maxY, 0)));
    }
}


