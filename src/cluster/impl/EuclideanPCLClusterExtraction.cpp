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

#include "next_best_view/cluster/impl/EuclideanPCLClusterExtraction.hpp"
#include <chrono>
#include <pcl-1.7/pcl/kdtree/kdtree.h>
#include <pcl-1.7/pcl/segmentation/extract_clusters.h>

namespace next_best_view {

    EuclideanPCLClusterExtraction::EuclideanPCLClusterExtraction() {
        mDebugHelperPtr = DebugHelper::getInstance();
    }

    EuclideanPCLClusterExtraction::~EuclideanPCLClusterExtraction() { }

    BoundingBoxPtrsPtr EuclideanPCLClusterExtraction::getClusters() {
        // if cache is invalid
        if (!mClustersCachePtr) {
            mClustersCachePtr = BoundingBoxPtrsPtr(new BoundingBoxPtrs());

            // kdtree, use by EuclideanClusterExtraction
            pcl::search::KdTree<ObjectPoint>::Ptr tree (new pcl::search::KdTree<ObjectPoint>);
            tree->setInputCloud(getObjectPointCloud());

            // find hypothesis clusters
            auto begin = std::chrono::high_resolution_clock::now();
            std::vector<pcl::PointIndices> clusterIndices;
            pcl::EuclideanClusterExtraction<ObjectPoint> euclideanClusterExtractor;
            euclideanClusterExtractor.setClusterTolerance (0.3); // 30cm
            euclideanClusterExtractor.setMinClusterSize (1);
            euclideanClusterExtractor.setMaxClusterSize (25000);
            euclideanClusterExtractor.setSearchMethod(tree);
            euclideanClusterExtractor.setInputCloud(getObjectPointCloud());
            euclideanClusterExtractor.setIndices(getObjectPointIndices());
            euclideanClusterExtractor.extract(clusterIndices);
            auto finish = std::chrono::high_resolution_clock::now();

            mDebugHelperPtr->writeNoticeably(std::stringstream() << "cluster extraction took " << std::chrono::duration<float>(finish - begin).count() << " seconds.", DebugHelper::CALCULATION);
            mDebugHelperPtr->writeNoticeably(std::stringstream() << "number of clusters: " << clusterIndices.size(), DebugHelper::CALCULATION);

            // determine bb per cluster
            mDebugHelperPtr->write("Clusters:", DebugHelper::CALCULATION);
            int i = 0;
            for (pcl::PointIndices &indices : clusterIndices) {
                // create cluster pc
                ObjectPointCloudPtr clusterPC(new ObjectPointCloud(*getObjectPointCloud(), indices.indices));

                // find min/max/bb of cluster
                ObjectPoint minPoint, maxPoint;
                pcl::getMinMax3D(*clusterPC, minPoint, maxPoint);

                mDebugHelperPtr->writeNoticeably(std::stringstream() << "cluster " << i << ": has " << indices.indices.size() << " object points", DebugHelper::CALCULATION);
                mDebugHelperPtr->writeNoticeably(std::stringstream() << "minPoint: " << minPoint, DebugHelper::CALCULATION);
                mDebugHelperPtr->writeNoticeably(std::stringstream() << "maxPoint: " << maxPoint, DebugHelper::CALCULATION);

                mClustersCachePtr->push_back(BoundingBoxPtr(new BoundingBox(minPoint.getPosition(), maxPoint.getPosition())));
                i++;
                mDebugHelperPtr->write(std::stringstream() << *(mClustersCachePtr->back()), DebugHelper::CALCULATION);
            }
        }
        return mClustersCachePtr;
    }

    void EuclideanPCLClusterExtraction::setObjectPointCloud(const ObjectPointCloudPtr &pointCloudPtr) {
        CommonClass::setObjectPointCloud(pointCloudPtr);
        // invalidate cache
        mClustersCachePtr.reset();
    }
}
