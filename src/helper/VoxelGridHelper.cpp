#include "next_best_view/helper/VoxelGridHelper.hpp"

namespace next_best_view {

VoxelGridHelper::VoxelGridHelper(GridVector3 size)
{
    mDebugHelperPtr = DebugHelper::getInstance();
    mGridSize = size;
    initializeVoxelGrid();
}

bool VoxelGridHelper::inVoxelGrid(const GridVector3 &gridPos)
{
    if (gridPos[0] < 0 || gridPos[1] < 0 || gridPos[2] < 0)
        return false;

    if (gridPos[0] >= mGridSize[0] || gridPos[1] >= mGridSize[1] || gridPos[2] >= mGridSize[2])
        return false;

    return true;
}

bool VoxelGridHelper::isMarked(GridVector3 gridPosition)
{
    if (!inVoxelGrid(gridPosition))
        return false;

    VoxelGridPtr voxelGridPtr;
    GridVector3 localGridPos;
    getVoxel(gridPosition, voxelGridPtr, localGridPos);

    voxel_grid::VoxelStatus vs = voxelGridPtr->getVoxel(localGridPos[0], localGridPos[1], localGridPos[2]);
    return vs == voxel_grid::VoxelStatus::MARKED;
}

void VoxelGridHelper::markVoxel(const GridVector3 &gridPos)
{
    if (!inVoxelGrid(gridPos))
    {
        mDebugHelperPtr->write(std::stringstream() << "Cannot mark voxel outside grid: " << gridPos << ".", DebugHelper::VOXEL_GRID);
        return;
    }
    mDebugHelperPtr->write(std::stringstream() << "Marking voxel " << gridPos << ".", DebugHelper::VOXEL_GRID);

    VoxelGridPtr voxelGridPtr;
    GridVector3 localGridPos;
    getVoxel(gridPos, voxelGridPtr, localGridPos);

    voxelGridPtr->markVoxel(localGridPos[0], localGridPos[1], localGridPos[2]);
}

const GridVector3 &VoxelGridHelper::getVoxelGridSize()
{
    return mGridSize;
}

void VoxelGridHelper::initializeVoxelGrid()
{
    for (unsigned int i = 0; i < ceil((double) mGridSize[2] / VOXEL_GRID_MAX_SIZE_Z); i++)
    {
        unsigned int remainingSizeZ = mGridSize[2] - i * VOXEL_GRID_MAX_SIZE_Z;
        if (remainingSizeZ > VOXEL_GRID_MAX_SIZE_Z)
            remainingSizeZ = VOXEL_GRID_MAX_SIZE_Z;
        VoxelGridPtr voxelGridPtr(new VoxelGrid(mGridSize[0], mGridSize[1], remainingSizeZ));
        mVoxelGrids.push_back(voxelGridPtr);
    }
}

void VoxelGridHelper::getVoxel(const GridVector3 &totalGridPos, VoxelGridPtr &voxelGridPtr, GridVector3 &localGridPos)
{
    int voxelGridIndex = (int) (totalGridPos[2] / VOXEL_GRID_MAX_SIZE_Z);

    voxelGridPtr = mVoxelGrids.at(voxelGridIndex);

    int gridPosZ = totalGridPos[2] % VOXEL_GRID_MAX_SIZE_Z;

    localGridPos = GridVector3(totalGridPos[0], totalGridPos[1], gridPosZ);
}

}
