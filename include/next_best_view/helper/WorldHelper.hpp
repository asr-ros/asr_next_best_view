#pragma once

#include <ros/package.h>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <rapidxml.hpp>
#include <rapidxml_utils.hpp>
#include <assimp_devel/Importer.hpp>
#include <assimp_devel/scene.h>
#include <assimp_devel/postprocess.h>
#include "next_best_view/helper/MapHelper.hpp"
#include "next_best_view/helper/TypeHelper.hpp"
#include "next_best_view/helper/VoxelGridHelper.hpp"
#include "next_best_view/helper/VisualizationsHelper.hpp"
#include "typedef.hpp"

namespace next_best_view {
    class WorldHelper {
    private:
        const double EPSILON = 10e-6;

        MapHelperPtr mMapHelperPtr;
        VoxelGridHelperPtr mVoxelGridHelperPtr;
        VisualizationHelperPtr mVisHelperPtr;
        DebugHelperPtr mDebugHelperPtr;

        double mMapVoxelSize, mWorldVoxelSize;

    public:
        WorldHelper(MapHelperPtr mapHelperPtr, std::string filePath, double voxelSize, double worldHeight);

        bool isOccluded(SimpleVector3 cameraPosition, SimpleVector3 objectPosition);

    private:
        bool isOccluded(SimpleVector3 cameraPos, SimpleVector3 objectPos,
                            GridVector3 currVoxelPos, GridVector3 objectGridPos, double tStart, std::vector<GridVector3> &traversedVoxels);

        void worldToVoxelGridCoordinates(const SimpleVector3 &worldPos, GridVector3 &result);

        void worldToVoxelGridCoordinates(const std::vector<SimpleVector3> &worldPositions, std::vector<GridVector3> &results);

        void mapToWorldCoordinates(const SimpleVector3 &worldPos, SimpleVector3 &result);

        void initializeVoxelGridHelper(double worldHeight);

        void loadVoxelGrid(std::string filePath);

        void parseXMLFile(const string &filePath, std::vector<string> &meshResources, std::vector<SimpleVector3> &positions,
                                                    std::vector<SimpleQuaternion> &orientations, std::vector<SimpleVector3> &scales);

        bool parsePoseString(const std::string &poseString, std::vector<double> &poseVec);

        std::string getAbsolutePath(std::string filePath);

        void loadMeshFile(const std::string &meshResource, const SimpleVector3 &position, const SimpleQuaternion &orientation,
                                                const SimpleVector3 &scale, std::vector<std::vector<SimpleVector3>> &faces);

        void addPoint(const SimpleVector3& point);

        void addLine(const std::vector<SimpleVector3>& vertices);

        void addTriangle(const std::vector<SimpleVector3>& vertices);

        void voxelToWorldBox(const GridVector3& gridPos, SimpleVector3& min, SimpleVector3& max);

        void voxelGridBox(const std::vector<GridVector3> &gridPositions, GridVector3 &min, GridVector3 &max);

        bool voxelVerticesAreNeighbours(const SimpleVector3 &vertexA, const SimpleVector3 &vertexB);

        bool lineIntersectsVoxel(const SimpleVector3& lineStartPos, const SimpleVector3& lineEndPos, const GridVector3& gridPos);

        bool lineIntersectsVoxel(const SimpleVector3 &lineStartPos, const SimpleVector3 &lineEndPos, const SimpleVector3 &voxelMin, const SimpleVector3 &voxelMax);

        bool lineIntersectsVoxelHelper(const SimpleVector3 &startPos, const SimpleVector3 &direction, const SimpleVector3 &voxelCoord,
                                                                            const SimpleVector3 &voxelMin, const SimpleVector3 &voxelMax);

        bool lineIntersectsTriangle(const SimpleVector3 &lineStartPos, const SimpleVector3 &lineEndPos, const std::vector<SimpleVector3> triangleVertices);

        bool equalVoxels(const GridVector3& a, const GridVector3& b);

    };

    typedef boost::shared_ptr<WorldHelper> WorldHelperPtr;
}
