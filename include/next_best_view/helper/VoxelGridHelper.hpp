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

        bool inVoxelGrid(const GridVector3& gridPos);

        bool isMarked(GridVector3 gridPosition);

        void markVoxel(const GridVector3& gridPos);

        const GridVector3& getVoxelGridSize();

    private:
        void initializeVoxelGrid();

        void getVoxel(const GridVector3& totalGridPos, VoxelGridPtr& voxelGridPtr, GridVector3& localGridPos);
    };

    typedef boost::shared_ptr<VoxelGridHelper> VoxelGridHelperPtr;
}
