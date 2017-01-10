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
