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

    template<class T>
    Grid<T>::Grid(float gridSize)
        : mGridSize(gridSize)
    { }

    template<class T>
    bool Grid<T>::hasElementAt(CacheIndex idx) {
        int xIdx = std::get<0>(idx);
        int yIdx = std::get<0>(idx);
        if (mElementPerGridTile.find(xIdx) == mElementPerGridTile.end()) {
            return false;
        }
        if (mElementPerGridTile[xIdx].find(yIdx) == mElementPerGridTile[xIdx].end()) {
            return false;
        }
        return true;
    }

    template<class T>
    T Grid<T>::getElementAt(CacheIndex idx) {
        int xIdx = std::get<0>(idx);
        int yIdx = std::get<0>(idx);
        if (!hasElementAt(idx)) {
            return T(); // TODO
        }
        T element = mElementPerGridTile[xIdx][yIdx];
        return element;
    }

    template<class T>
    void Grid<T>::setElementAt(CacheIndex idx, T &element) {
        if (!hasElementAt(idx)) {
            mGridIndices.push_back(idx);
        }
        int xIdx = std::get<0>(idx);
        int yIdx = std::get<0>(idx);
        if (mElementPerGridTile.find(xIdx) == mElementPerGridTile.end()) {
            // xIdx not found
            mElementPerGridTile.insert(std::make_pair(xIdx, std::map<int, T>()));
        }
        if (mElementPerGridTile[xIdx].find(yIdx) == mElementPerGridTile[xIdx].end()) {
            // yIdx not found
            mElementPerGridTile[xIdx].insert(std::make_pair(yIdx, element));
        } else {
            mElementPerGridTile[xIdx][yIdx] = element;
        }
    }

    template<class T>
    std::vector<CacheIndex> Grid<T>::getGridIndices() {
        return mGridIndices;
    }

    template<class T>
    CacheIndex Grid<T>::getCacheIdx(SimpleVector3 v) {
        return std::make_tuple(floor(v[0] / mGridSize), floor(v[1] / mGridSize));
    }

    template<class T>
    void Grid<T>::clear() {
        mGridIndices.clear();
        mElementPerGridTile.clear();
    }

    template<class T>
    unsigned int Grid<T>::size() {
        return mGridIndices.size();
    }

    template<class T>
    bool Grid<T>::empty() {
        return mGridIndices.empty();
    }

    // static memebr declaration
    RatingModulePtr NextBestViewCache::mRatingModulePtr;

    NextBestViewCache::NextBestViewCache(float gridSize)
        : mGridSize(gridSize) {
        mUtilityGrid = GridPtr<ViewportPoint>(new Grid<ViewportPoint>(gridSize));
        mRatingGrid = GridPtr<ViewportPoint>(new Grid<ViewportPoint>(gridSize));
    }

    bool NextBestViewCache::getBestUtilityViewportAt(int xIdx, int yIdx, ViewportPoint &viewport) {
        if (!mUtilityGrid->hasElementAt(std::make_tuple(xIdx, yIdx))) {
            return false;
        }
        viewport = mUtilityGrid->getElementAt(std::make_tuple(xIdx, yIdx));
        return true;
    }

    ViewportPointCloudPtr NextBestViewCache::getAllBestUtilityViewports() {
        ViewportPointCloudPtr bestViewports(new ViewportPointCloud());
        std::vector<CacheIndex> cacheIndices = mUtilityGrid->getGridIndices();
        for (CacheIndex idx : cacheIndices) {
            ViewportPoint viewport;
            if (!getBestUtilityViewportAt(std::get<0>(idx), std::get<1>(idx), viewport)) {
                ROS_ERROR_STREAM("idx of getCachedGridIndices was invalid");
            }
            bestViewports->push_back(viewport);
        }
        // only keep best ones according utility
        std::sort(bestViewports->begin(), bestViewports->end(), compareViewportsUtilitywise);
        return bestViewports;
    }

    void NextBestViewCache::updateCache(const ViewportPointCloudPtr &ratedNextBestViewportsPtr) {
        // utilityGrid
        std::sort(ratedNextBestViewportsPtr->begin(), ratedNextBestViewportsPtr->end(), compareViewportsUtilitywise);
        BOOST_REVERSE_FOREACH (ViewportPoint &ratedViewport, *ratedNextBestViewportsPtr) {
            CacheIndex idx = mUtilityGrid->getCacheIdx(ratedViewport.getPosition());
            if (mUtilityGrid->hasElementAt(idx)) {
                ViewportPoint curBestViewport = mUtilityGrid->getElementAt(idx);
                if (ratedViewport.score->getUnweightedUnnormalizedUtility() > curBestViewport.score->getUnweightedUnnormalizedUtility()) {
                    mUtilityGrid->setElementAt(idx, ratedViewport);
                }
            } else {
                mUtilityGrid->setElementAt(idx, ratedViewport);
            }
        }
        // ratingGrid
        std::sort(ratedNextBestViewportsPtr->begin(), ratedNextBestViewportsPtr->end(), compareViewportsUtilitywise);
        BOOST_REVERSE_FOREACH (ViewportPoint &ratedViewport, *ratedNextBestViewportsPtr) {
            CacheIndex idx = mRatingGrid->getCacheIdx(ratedViewport.getPosition());
            if (mRatingGrid->hasElementAt(idx)) {
                ViewportPoint curBestViewport = mRatingGrid->getElementAt(idx);
                if (mRatingModulePtr->getRating(ratedViewport.score) > mRatingModulePtr->getRating(curBestViewport.score)) {
                    mRatingGrid->setElementAt(idx, ratedViewport);
                }
            } else {
                mRatingGrid->setElementAt(idx, ratedViewport);
            }
        }
    }

    void NextBestViewCache::clearUtilityCache() {
        mUtilityGrid->clear();
    }

    void NextBestViewCache::clearRatingCache() {
        mRatingGrid->clear();
    }

    int NextBestViewCache::size() {
        return mUtilityGrid->size(); // == mRatingGrid->size() is not always true
    }

    bool NextBestViewCache::isEmpty() {
        return mUtilityGrid->empty(); // == mRatingGrid->empty() is not always true
    }

    RatingModulePtr NextBestViewCache::getRatingModulePtr() {
        return mRatingModulePtr;
    }

    void NextBestViewCache::setRatingModulePtr(const RatingModulePtr &ratingModulePtr) {
        mRatingModulePtr = ratingModulePtr;
    }

    bool NextBestViewCache::compareViewportsUtilitywise(const ViewportPoint &a, const ViewportPoint &b) {
        // a < b
        return a.score->getUnweightedUnnormalizedUtility() < b.score->getUnweightedUnnormalizedUtility();
    }

    bool NextBestViewCache::compareViewportsRatingwise(const ViewportPoint &a, const ViewportPoint &b) {
        // a < b
        return mRatingModulePtr->compareViewports(a, b);
    }
}
