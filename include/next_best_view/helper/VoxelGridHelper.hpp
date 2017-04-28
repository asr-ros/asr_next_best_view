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

#include "next_best_view/helper/DebugHelper.hpp"
#include "typedef.hpp"

namespace next_best_view {
    class VoxelGridHelper {
    private:
        const unsigned int VOXEL_GRID_MAX_SIZE_Z = 16;

        DebugHelperPtr mDebugHelperPtr;

        GridVector3 mGridSize;

        // voxel grid only supports size_z <= 16 => use more than one voxel grid if necessary
        std::vector<VoxelGridPtr> mVoxelGrids;

    public:
        VoxelGridHelper(GridVector3 size);

        /*!
         * \brief returns whether the given voxel coordinate is in the voxel grid
         * \param [in] gridPos the voxel coordinate
         * \return whether gridPos is in the voxel grid
         */
        bool inVoxelGrid(const GridVector3& gridPos);

        /*!
         * \brief returns whetehr the given voxel is marked
         * \param [in] gridPosition the voxel coordinate
         * \return whetehr the voxel at gridPosition is marked
         */
        bool isMarked(GridVector3 gridPosition);

        /*!
         * \brief marks the given voxel
         * \param [in] gridPos the voxel coordinate
         */
        void markVoxel(const GridVector3& gridPos);

        /*!
         * \brief returns the voxel grid size for each axis
         * \return the voxel grid size for each axis
         */
        const GridVector3& getVoxelGridSize();

    private:
        /*!
         * \brief initializes the voxel grid
         */
        void initializeVoxelGrid();

        /*!
         * \brief retrieves the internal voxel grid and the local grid position in that voxel grid for the grid position in the big voxel grid
         * \param [in] totalGridPos the grid position in the big voxel grid
         * \param [out] voxelGridPtr the internal voxel grid
         * \param [out] localGridPos the local grid position
         */
        void getVoxel(const GridVector3& totalGridPos, VoxelGridPtr& voxelGridPtr, GridVector3& localGridPos);
    };

    typedef boost::shared_ptr<VoxelGridHelper> VoxelGridHelperPtr;
}
