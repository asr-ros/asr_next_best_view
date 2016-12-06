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
#include <Eigen/Dense>
#include <iostream>
#include <tuple>
#include <chrono>
#include "next_best_view/helper/MapHelper.hpp"
#include "next_best_view/helper/TypeHelper.hpp"
#include "next_best_view/helper/VoxelGridHelper.hpp"
#include "next_best_view/helper/VisualizationsHelper.hpp"
#include "typedef.hpp"

namespace next_best_view {
    class WorldHelper {
    private:
        typedef std::tuple<int,int,int> VoxelTuple;
        typedef std::vector<VoxelTuple> VoxelTuples;
        typedef boost::shared_ptr<VoxelTuples> VoxelTuplesPtr;
        typedef std::map<VoxelTuple, VoxelTuplesPtr> CameraVoxelToObjectVoxels;
        typedef std::map<VoxelTuple, IndicesPtr> ObjectVoxelToIndices;
        typedef std::pair<VoxelTuple, IndicesPtr> ObjectVoxelAndIndices;

        const double EPSILON = 10e-6;

        MapHelperPtr mMapHelperPtr;
        VoxelGridHelperPtr mVoxelGridHelperPtr;
        VisualizationHelperPtr mVisHelperPtr;
        DebugHelperPtr mDebugHelperPtr;

        // the voxel size in map and world coordinates
        double mMapVoxelSize, mWorldVoxelSize;

        // whether the raytracing shall be visualized
        bool mVisualizeRaytracing;

        // cached data
        CameraVoxelToObjectVoxels cameraVoxelToOccludedVoxels;
        CameraVoxelToObjectVoxels cameraVoxelToUnoccludedVoxels;

    public:
        /*!
         * \brief constructor for a WorldHelper object
         * \param mapHelperPtr [in] an instance of the MapHelper
         * \param filePath [in] the path to the XML file describing the environment
         * \param voxelSize [in] the length of an edge of a voxel
         * \param worldHeight [in] the height of the world in meters
         * \param visualizeRaytracing [in] whether the raytracing shall be visualized
         */
        WorldHelper(MapHelperPtr mapHelperPtr, std::string filePath, double voxelSize, double worldHeight, bool visualizeRaytracing);

        /*!
         * \brief filters objects that are occluded by the environment from the view of the camera
         * \param cameraPosition [in] the position of the camera
         * \param objectPointCloud [in] the object point cloud
         * \param indices [in] the indices of the objects that shall be checked
         * \param filteredIndices [out] the indices of the objects which are not occluded by the environment. Only indices from parameter "indices" are included.
         */
        void filterOccludedObjects(SimpleVector3 cameraPosition, ObjectPointCloudPtr objectPointCloud, IndicesPtr indices,
                                                                                                            IndicesPtr &filteredIndices);

        /*!
         * \brief returns whether the given target position is occluded by the environment from the view of the camera
         * \param cameraPosition [in] the position of the camera
         * \param targetPosition [in] the position of the target
         * \return whether the given target position is occluded by the environment from the view of the camera
         */
        bool isOccluded(SimpleVector3 cameraPosition, SimpleVector3 targetPosition);

    private:
        /*!
         * \brief returns whether the given target position is occluded by the environment from the view of the camera. Computes the occlusion recursively.
         * \param cameraPos [in] the position of the camera
         * \param targetPos [in] the position of the target
         * \param currVoxelPos [in] the current voxel to look at
         * \param traversedVoxels [in,out] the voxels which already were traversed in this recursion
         * \return whether the given target position is occluded by the environment from the view of the camera
         */
        bool isOccluded(SimpleVector3 cameraPos, SimpleVector3 targetPos, GridVector3 currVoxelPos, std::vector<GridVector3> &traversedVoxels);

        /*!
         * \brief computes the voxels which are occupied by the given point
         * \param worldPos [in] the point in world coordinates
         * \param result [out] the list of voxels which are occupied by the given point
         */
        void worldToVoxelGridCoordinates(const SimpleVector3 &worldPos, std::vector<GridVector3> &result);

        /*!
         * \brief computes the voxels which are occupied by the given points
         * \param worldPositions [in] the list of points in world coordinates
         * \param results [out] the list of voxels which are occupied by the given points
         */
        void worldToVoxelGridCoordinates(const std::vector<SimpleVector3> &worldPositions, std::vector<GridVector3> &results);

        /*!
         * \brief initializes the VoxelGridHelper with the given world height
         * \param worldHeight the world height in meters
         */
        void initializeVoxelGridHelper(double worldHeight);

        /*!
         * \brief loads the given environment into the voxel grid
         * \param filePath [in] the path to the XML file describing the environment
         */
        void loadVoxelGrid(std::string filePath);

        /*!
         * \brief parses the given XML file describing the environment. The output parameters cover the same world object for the same index
         * \param filePath [in] the path to the XML file describing the environment
         * \param meshResources [out] the mesh resources listed in the file
         * \param positions [out] the positions listed in the file
         * \param orientations [out] the orientations listed in the file
         * \param scales [out] the scales listed in the file
         */
        void parseXMLFile(const string &filePath, std::vector<string> &meshResources, std::vector<SimpleVector3> &positions,
                                                    std::vector<SimpleQuaternion> &orientations, std::vector<SimpleVector3> &scales);

        /*!
         * \brief extracts a pose from a given string
         * \param poseString [in] the string containing the information about the pose
         * \param poseVec [out] the list of values that build the pose
         * \return whether no error was thrown
         */
        bool parsePoseString(const std::string &poseString, std::vector<double> &poseVec);

        /*!
         * \brief returns the absolute path for the given file path
         * \param filePath
         * \return the absolute path for the given file path
         */
        std::string getAbsolutePath(std::string filePath);

        /*!
         * \brief loads a mesh file into the voxel grid
         * \param meshResource [in] the path to the mesh resource
         * \param position [in] the position of the instance of the mesh file
         * \param orientation [in] the orientation of the instance of the mesh file
         * \param scale [in] the scale of the instance of the mesh file
         * \param faces [out] a list of the vertices for each face that is part of the mesh file
         */
        void loadMeshFile(const std::string &meshResource, const SimpleVector3 &position, const SimpleQuaternion &orientation,
                                                const SimpleVector3 &scale, std::vector<std::vector<SimpleVector3>> &faces);

        /*!
         * \brief adds the given point to the voxel grid
         * \param point [in] the point
         */
        void addPoint(const SimpleVector3& point);

        /*!
         * \brief adds the given line to the voxel grid
         * \param vertices [in] the vertices of the line
         */
        void addLine(const std::vector<SimpleVector3>& vertices);

        /*!
         * \brief adds the given triangle to the voxel grid
         * \param vertices [in] the vertices of the triangle
         */
        void addTriangle(const std::vector<SimpleVector3>& vertices);

        /*!
         * \brief returns the center point of the given voxel in world coordinates
         * \param [in] gridPos the voxel coordinates
         * \return the center point of the given voxel in world coordinates
         */
        SimpleVector3 voxelToWorldPosition(const GridVector3& gridPos);

        /*!
         * \brief computes the bounding box of the given voxel in world coordinates
         * \param gridPos [in] the voxel coordinates
         * \param min [out] the world coordinates of the vertex with the smallest coordinates
         * \param max [out] the world coordinates of the vertex with the biggest coordinates
         */
        void voxelToWorldBox(const GridVector3& gridPos, SimpleVector3& min, SimpleVector3& max);

        /*!
         * \brief computes the bounding box of the given list of voxels
         * \param gridPositions [in] the list of voxels
         * \param min [out] the voxel in the bounding box with the smallest coordinates
         * \param max [out] the voxel in the bounding box with the biggest coordinates
         */
        void voxelGridBox(const std::vector<GridVector3> &gridPositions, GridVector3 &min, GridVector3 &max);

        /*!
         * \brief returns whether the given vertices of a voxel are connected by an edge of the voxel
         * \param vertexA [in] the first vertex of the voxel
         * \param vertexB [in] the second vertex of the voxel
         * \return whether the two vertices are connected by an edge of the voxel
         */
        bool voxelVerticesAreNeighbours(const SimpleVector3 &vertexA, const SimpleVector3 &vertexB);

        /*!
         * \brief returns whether the given line intersects the given voxel
         * \param lineStartPos [in] the start position of the line
         * \param lineEndPos [in] the end position of the line
         * \param gridPos [in] the voxel coordinates
         * \return whether the given line intersects the given voxel
         */
        bool lineIntersectsVoxel(const SimpleVector3& lineStartPos, const SimpleVector3& lineEndPos, const GridVector3& gridPos);

        /*!
         * \brief returns whether the given line intersects the given voxel
         * \param lineStartPos [in] the original start position of the line
         * \param lineEndPos [in] the end position of the line
         * \param gridPos [in] the voxel coordinates
         * \param tLowerBound [in] the interpolation position between lineStartPos and lineEndPos where the line shall start
         * \return whether the given line intersects the given voxel
         */
        bool lineIntersectsVoxel(const SimpleVector3& lineStartPos, const SimpleVector3& lineEndPos, const GridVector3& gridPos,
                                                                                                                double tLowerBound);

        /*!
         * \brief returns whether the given line intersects the given voxel
         * \param lineStartPos [in] the original start position of the line
         * \param lineEndPos [in] the end position of the line
         * \param gridPos [in] the voxel coordinates
         * \param tLowerBound [in] the interpolation position between lineStartPos and lineEndPos where the line shall start
         * \param tMin [out] the minimum interpolation position between lineStartPos and lineEndPos that intersects the voxel
         * \return whether the given line intersects the given voxel
         */
        bool lineIntersectsVoxel(const SimpleVector3& lineStartPos, const SimpleVector3& lineEndPos, const GridVector3& gridPos,
                                                                                                    double tLowerBound, double &tMin);

        bool lineIntersectsVoxelHelper(const SimpleVector3 &lineStartPos, const SimpleVector3 &lineEndPos, const SimpleVector3 &voxelCoord,
                                                const SimpleVector3 &voxelMin, const SimpleVector3 &voxelMax, double tLowerBound, double &tMin);

        /*!
         * \brief returns whether the given line intersects the given triangle
         * \param lineStartPos [in] the start position of the line
         * \param lineEndPos [in] the end position of the line
         * \param triangleVertices [in] the vertices of the triangle
         * \return whether the given line intersects the given triangle
         */
        bool lineIntersectsTriangle(const SimpleVector3 &lineStartPos, const SimpleVector3 &lineEndPos,
                                                        const std::vector<SimpleVector3> triangleVertices);

        /*!
         * \brief returns whether the two given voxels are equal
         * \param a [in] the first voxel
         * \param b [in] the second voxel
         * \return whether the two voxels are equal
         */
        bool equalVoxels(const GridVector3& a, const GridVector3& b);

        /*!
         * \brief inserts each index from the source list into the target list if the target list does not contain that index already
         * \param source [in] the source index list
         * \param target [in,out] the target index list
         */
        void insert(IndicesPtr source, IndicesPtr &target);

    };

    typedef boost::shared_ptr<WorldHelper> WorldHelperPtr;
}
