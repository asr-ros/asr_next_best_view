#include "next_best_view/helper/WorldHelper.hpp"

namespace next_best_view {

WorldHelper::WorldHelper(next_best_view::MapHelperPtr mapHelperPtr, std::string pcdFilePath, double voxelSize, double worldHeight) :
    mMapHelperPtr(mapHelperPtr), mVoxelSize(voxelSize)
{
    initializeVoxelGrid(worldHeight);
    loadVoxelGrid(pcdFilePath);
}

bool WorldHelper::isOccluded(SimpleVector3 cameraPosition, SimpleVector3 objectPosition)
{
    GridVector3 cameraGridPos, objectGridPos;

    mapToVoxelGridCoordinates(cameraPosition, cameraGridPos);
    mapToVoxelGridCoordinates(objectPosition, objectGridPos);

    return isOccluded(cameraPosition, objectPosition, cameraGridPos, objectGridPos, 0);
}

bool WorldHelper::isOccluded(SimpleVector3 cameraPos, SimpleVector3 objectPos,
                             GridVector3 currGridPos, GridVector3 objectGridPos,
                             double tStart)
{
    if (isMarked(currGridPos))
        return true;

    if (currGridPos[0] == objectGridPos[0] && currGridPos[1] == objectGridPos[1]
            && currGridPos[2] == objectGridPos[2])
        return false;

    // get next grid
    SimpleVector3 direction = objectPos - cameraPos;

    SimpleVector3 min, max;

    for (int i = 0; i < min.rows(); i++)
        min[i] = currGridPos[i] * mVoxelSize;

    max = min + SimpleVector3(mVoxelSize, mVoxelSize, mVoxelSize);

    double tMin = 2 * mVoxelSize;
    GridVector3 nextGrid = currGridPos;

    for(int i = 0; i < min.rows(); i++)
    {
        double tCurrent = (min[i] - cameraPos[i]) / direction[i];

        if (abs(tCurrent - tMin) < 10e-6)
            nextGrid[i]--;
        else if (tStart < tCurrent && tCurrent < tMin)
        {
            tMin = tCurrent;
            nextGrid = currGridPos;
            nextGrid[i]--;
        }
    }

    for(int i = 0; i < max.rows(); i++)
    {
        double tCurrent = (max[i] - cameraPos[i]) / direction[i];

        if (abs(tCurrent - tMin) < 10e-6)
            nextGrid[i]++;
        else if (tStart < tCurrent && tCurrent < tMin)
        {
            tMin = tCurrent;
            nextGrid = currGridPos;
            nextGrid[i]++;
        }
    }

    return isOccluded(cameraPos, objectPos, nextGrid, objectGridPos, tMin);
}

bool WorldHelper::isMarked(GridVector3 gridPosition)
{
    if (gridPosition[0] < 0 || gridPosition[1] < 0 || gridPosition[2] < 0)
        return false;

    if (gridPosition[0] >= mVoxelGridPtr->sizeX() || gridPosition[1] >= mVoxelGridPtr->sizeY() ||
            gridPosition[2] >= mVoxelGridPtr->sizeZ())
        return false;

    voxel_grid::VoxelStatus vs = mVoxelGridPtr->getVoxel(gridPosition[0], gridPosition[1], gridPosition[2]);
    if (vs == voxel_grid::VoxelStatus::MARKED)
        return true;

    return false;
}

void WorldHelper::mapToVoxelGridCoordinates(const SimpleVector3 &mapPos, GridVector3 &result)
{
    // map to world
    SimpleVector3 worldPosition;
    mMapHelperPtr->mapToWorldCoordinates(mapPos[0], mapPos[1], worldPosition);
    worldPosition[2] = mapPos[2];

    // world to grid
    for (int i = 0; i < result.rows(); i++)
    {
        result[i] = (int) worldPosition[i] / mVoxelSize;
    }
}

void WorldHelper::initializeVoxelGrid(double worldHeight)
{
    unsigned int size_x, size_y, size_z;
    size_x = ceil((double) mMapHelperPtr->getMetricWidth() / mVoxelSize);
    size_y = ceil((double) mMapHelperPtr->getMetricHeight() / mVoxelSize);
    size_z = ceil(worldHeight / mVoxelSize);
    mVoxelGridPtr = VoxelGridPtr(new VoxelGrid(size_x, size_y, size_z));
}

void WorldHelper::loadVoxelGrid(std::string pcdFilePath)
{
    WorldPointCloudPtr pointCloudPtr(new WorldPointCloud());

    if (pcl::io::loadPCDFile<WorldPoint>(pcdFilePath, *pointCloudPtr) != 0)
    {
        ROS_ERROR_STREAM("Could not load file " << pcdFilePath);
        return;
    }

    BOOST_FOREACH(WorldPoint point, *pointCloudPtr)
    {
        SimpleVector3 mapPos(point.x, point.y, point.z);
        GridVector3 gridPos;
        mapToVoxelGridCoordinates(mapPos, gridPos);
        mVoxelGridPtr->markVoxel(gridPos[0], gridPos[1], gridPos[2]);
    }
}

}
