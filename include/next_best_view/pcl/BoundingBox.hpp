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

namespace next_best_view {

    class BoundingBox {
    public:
        SimpleVector3 minPos, maxPos;
        float xWidth, yWidth, height;

    public:
        BoundingBox(SimpleVector3 minPos, SimpleVector3 maxPos) : minPos(minPos), maxPos(maxPos) {
            xWidth = abs(maxPos[0] - minPos[0]);
            yWidth = abs(maxPos[1] - minPos[1]);
            height = abs(maxPos[2] - minPos[2]);
            refreshSize();
        }

        BoundingBox(BoundingBox &bb) {
            minPos = bb.minPos;
            maxPos = bb.maxPos;
            refreshSize();
        }

        BoundingBox(const BoundingBoxPtrsPtr &boundingBoxes) {
            float minX = (*boundingBoxes)[0]->minPos[0];
            float minY = (*boundingBoxes)[0]->minPos[1];
            float minZ = (*boundingBoxes)[0]->minPos[2];
            float maxX = (*boundingBoxes)[0]->maxPos[0];
            float maxY = (*boundingBoxes)[0]->maxPos[1];
            float maxZ = (*boundingBoxes)[0]->maxPos[2];
            for (BoundingBoxPtr &bb : *boundingBoxes) {
                minX = min(bb->minPos[0], minX);
                minY = min(bb->minPos[1], minY);
                minZ = min(bb->minPos[2], minZ);
                maxX = max(bb->maxPos[0], maxX);
                maxY = max(bb->maxPos[1], maxY);
                maxZ = max(bb->maxPos[2], maxZ);
            }
            minPos = SimpleVector3(minX, minY, minZ);
            maxPos = SimpleVector3(maxX, maxY, maxZ);
            refreshSize();
        }

        BoundingBox(SimpleVector3CollectionPtr vertices) {
            // first vertex x/y/z value
            float minX = (*vertices)[0][0];
            float minY = (*vertices)[0][1];
            float minZ = (*vertices)[0][2];
            float maxX = (*vertices)[0][0];
            float maxY = (*vertices)[0][1];
            float maxZ = (*vertices)[0][2];
            for (SimpleVector3 &v : *vertices) {
                minX = min(v[0], minX);
                minY = min(v[1], minY);
                minZ = min(v[2], minZ);
                maxX = max(v[0], maxX);
                maxY = max(v[1], maxY);
                maxZ = max(v[2], maxZ);
            }
            minPos = SimpleVector3(minX, minY, minZ);
            maxPos = SimpleVector3(maxX, maxY, maxZ);
            refreshSize();
        }

        bool contains(SimpleVector3 v) {
            return (xWidth < 0.00001 || (minPos[0] <= v[0] && v[0] <= maxPos[0])) &&
                    (yWidth < 0.00001 || (minPos[1] <= v[1] && v[1] <= maxPos[1])) &&
                    (height < 0.00001 || (minPos[2] <= v[2] && v[2] <= maxPos[2]));
        }

        SimpleVector3 getCentroid() const {
            return SimpleVector3(minPos[0] + xWidth / 2.0, minPos[1] + yWidth / 2.0, minPos[2] + height / 2.0);
        }

        /**
         * @brief expand expands this bounding box by an vector, that determines the size to expand per surface.
         * @param v
         */
        void expand(SimpleVector3 v) {
            minPos[0] -= v[0];
            minPos[1] -= v[1];
            minPos[2] -= v[2];
            maxPos[0] += v[0];
            maxPos[1] += v[1];
            maxPos[2] += v[2];
            refreshSize();
        }

        void ignoreAxis(int axis) {
            maxPos[axis] = 0;
            minPos[axis] = 0;
            refreshSize();
        }

        void refreshSize() {
            xWidth = abs(maxPos[0] - minPos[0]);
            yWidth = abs(maxPos[1] - minPos[1]);
            height = abs(maxPos[2] - minPos[2]);
        }
    };
    typedef boost::shared_ptr<BoundingBox> BoundingBoxPtr;

    std::ostream& operator<<(std::ostream &strm, const next_best_view::BoundingBox &bb);
    std::ostream& operator<<(std::ostream &strm, const next_best_view::BoundingBoxPtr &bb);
}
