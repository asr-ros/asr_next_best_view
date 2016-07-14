#pragma once

#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include "next_best_view/helper/MapHelper.hpp"
#include "typedef.hpp"

namespace next_best_view {
    class WorldHelper {
    private:
        MapHelperPtr mMapHelperPtr;
        VoxelGridPtr mVoxelGridPtr;

        double mVoxelSize;

    public:
        WorldHelper(MapHelperPtr mapHelperPtr, std::string pcdFilePath, double voxelSize, double worldHeight);

        bool isOccluded(SimpleVector3 cameraPosition, SimpleVector3 objectPosition);

    private:
        bool isOccluded(SimpleVector3 cameraPos, SimpleVector3 objectPos,
                            GridVector3 currGridPos, GridVector3 objectGridPos, double tStart);

        bool isMarked(GridVector3 gridPosition);

        void mapToVoxelGridCoordinates(const SimpleVector3 &mapPos, GridVector3 &result);

        void initializeVoxelGrid(double worldHeight);

        void loadVoxelGrid(std::string pcdFilePath);

    };

    typedef boost::shared_ptr<WorldHelper> WorldHelperPtr;
}
