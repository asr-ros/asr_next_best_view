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


#include "next_best_view/NextBestViewCache.hpp"

namespace next_best_view {

    NextBestViewCache::NextBestViewCache(float gridSize) : mGridSize(gridSize) { }

    /**
     * @brief getCachedGridIndices
     * @return
     */
    std::vector<CacheIndex> NextBestViewCache::getCacheGridIndices() {
        if (cachedCacheGridIndices.empty()) {
            // for a fixed x we get a y col/row
            for (auto &yRowIt : mBestViewportPerGridElem) {
                for (auto &gridElem : yRowIt.second) {
                    cachedCacheGridIndices.push_back(std::make_tuple(yRowIt.first, gridElem.first));
                }
            }
        }
        return cachedCacheGridIndices;
    }

    /**
     * @brief getCacheIdx
     * @param v
     * @return
     */
    CacheIndex NextBestViewCache::getCacheIdx(SimpleVector3 v) {
        return std::make_tuple(floor(v[0] / mGridSize), floor(v[1] / mGridSize));
    }

    /**
     * @brief getBestViewportAt
     * @param xIdx
     * @param yIdx
     * @param viewport
     * @return
     */
    bool NextBestViewCache::getBestViewportAt(int xIdx, int yIdx, ViewportPoint &viewport) {
        if (mBestViewportPerGridElem.find(xIdx) == mBestViewportPerGridElem.end()) {
            return false;
        }
        if (mBestViewportPerGridElem[xIdx].find(yIdx) == mBestViewportPerGridElem[xIdx].end()) {
            return false;
        }
        viewport = mBestViewportPerGridElem[xIdx][yIdx];
        return true;
    }

    /**
     * @brief getAllBestViewports
     * @return
     */
    ViewportPointCloudPtr NextBestViewCache::getAllBestViewports() {
        ViewportPointCloudPtr bestViewports(new ViewportPointCloud());
        std::vector<CacheIndex> cacheIndices = getCacheGridIndices();
        for (CacheIndex idx : cacheIndices) {
            ViewportPoint viewport;
            if (!getBestViewportAt(std::get<0>(idx), std::get<1>(idx), viewport)) {
                ROS_ERROR_STREAM("idx of getCachedGridIndices was invalid");
            }
            bestViewports->push_back(viewport);
        }
        // only keep best ones according utility
        std::sort(bestViewports->begin(), bestViewports->end(), compareViewportsUtilitywise);
        return bestViewports;
    }

    /**
     * @brief updateCache adds new rated viewports to be cached.
     * @param ratedNextBestViewportsPtr
     */
    void NextBestViewCache::updateCache(const ViewportPointCloudPtr &ratedNextBestViewportsPtr) {
        std::sort(ratedNextBestViewportsPtr->begin(), ratedNextBestViewportsPtr->end(), compareViewportsUtilitywise);
        BOOST_REVERSE_FOREACH (ViewportPoint &ratedViewport, *ratedNextBestViewportsPtr) {
            CacheIndex idx = getCacheIdx(ratedViewport.getPosition());
            int xIdx = std::get<0>(idx);
            int yIdx = std::get<1>(idx);
            if (mBestViewportPerGridElem.find(xIdx) == mBestViewportPerGridElem.end()) {
                // xIdx not found
                mBestViewportPerGridElem.insert(std::make_pair(xIdx, std::map<int, ViewportPoint>()));
            }
            if (mBestViewportPerGridElem[xIdx].find(yIdx) == mBestViewportPerGridElem[xIdx].end()) {
                // yIdx not found
                mBestViewportPerGridElem[xIdx].insert(std::make_pair(yIdx, ratedViewport));
            } else {
                if (ratedViewport.score->getUnweightedUnnormalizedUtility() > mBestViewportPerGridElem[xIdx][yIdx].score->getUnweightedUnnormalizedUtility()) {
                    mBestViewportPerGridElem[xIdx][yIdx] = ratedViewport;
                }
            }
        }
        cachedCacheGridIndices.clear();
    }

    /**
     * @brief clearCache
     */
    void NextBestViewCache::clearCache() {
        mBestViewportPerGridElem.clear();
        cachedCacheGridIndices.clear();
    }

    /**
     * @brief size
     * @return
     */
    int NextBestViewCache::size() {
        return getCacheGridIndices().size();
    }

    /**
     * @brief isEmpty
     * @return
     */
    bool NextBestViewCache::isEmpty() {
        return mBestViewportPerGridElem.empty();
    }

    bool NextBestViewCache::compareViewportsUtilitywise(const ViewportPoint &a, const ViewportPoint &b) {
        // a < b
        return a.score->getUnweightedUnnormalizedUtility() < b.score->getUnweightedUnnormalizedUtility();
    }
}
