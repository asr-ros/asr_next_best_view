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

#pragma once

#include "typedef.hpp"
#include <vector>
#include <tuple>
#include <math.h>

namespace next_best_view {

    typedef std::tuple<int, int> CacheIndex;

    // TODO maybe implement some kind of filter so we don't sample with too low resolution
    class NextBestViewCache {
    private:
        float mGridSize;
        std::map<int, std::map<int, ViewportPoint>> mBestViewportPerGridElem; // these 2 maps represent a sparse 2d grid
        std::vector<CacheIndex> cachedCacheGridIndices;

    public:
        NextBestViewCache(float gridSize = 0.1);

        /**
         * @brief getCachedGridIndices
         * @return
         */
        std::vector<CacheIndex> getCacheGridIndices();

        /**
         * @brief getCacheIdx
         * @param v
         * @return
         */
        CacheIndex getCacheIdx(SimpleVector3 v);

        /**
         * @brief getBestViewportAt
         * @param xIdx
         * @param yIdx
         * @param viewport
         * @return
         */
        bool getBestViewportAt(int xIdx, int yIdx, ViewportPoint &viewport);

        /**
         * @brief getAllBestViewports
         * @return
         */
        ViewportPointCloudPtr getAllBestViewports();

        /**
         * @brief updateCache adds new rated viewports to be cached.
         * @param ratedNextBestViewportsPtr
         */
        void updateCache(const ViewportPointCloudPtr &ratedNextBestViewportsPtr);

        /**
         * @brief clearCache
         */
        void clearCache();

        /**
         * @brief size
         * @return
         */
        int size();

        /**
         * @brief isEmpty
         * @return
         */
        bool isEmpty();

    private:
        static bool compareViewportsUtilitywise(const ViewportPoint &a, const ViewportPoint &b);
    };
    typedef boost::shared_ptr<NextBestViewCache> NextBestViewCachePtr;
}
