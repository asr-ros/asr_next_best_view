#include "next_best_view/helper/WorldHelper.hpp"

namespace next_best_view {

WorldHelper::WorldHelper(next_best_view::MapHelperPtr mapHelperPtr, std::string filePath, double voxelSize, double worldHeight) :
    mMapHelperPtr(mapHelperPtr), mWorldVoxelSize(voxelSize)
{
    mDebugHelper = DebugHelper::getInstance();

    mMapHelperPtr->worldToMapSize(voxelSize, mMapVoxelSize);

    initializeVoxelGrid(worldHeight);
    loadVoxelGrid(filePath);
}

bool WorldHelper::isOccluded(SimpleVector3 cameraPosition, SimpleVector3 objectPosition)
{
    GridVector3 cameraGridPos, objectGridPos;

    worldToVoxelGridCoordinates(cameraPosition, cameraGridPos);
    worldToVoxelGridCoordinates(objectPosition, objectGridPos);

    return isOccluded(cameraPosition, objectPosition, cameraGridPos, objectGridPos, 0);
}

bool WorldHelper::isOccluded(SimpleVector3 cameraPos, SimpleVector3 objectPos,
                             GridVector3 currGridPos, GridVector3 objectGridPos,
                             double tStart)
{
    if (isMarked(currGridPos))
        return true;

    if (equalVoxels(currGridPos, objectGridPos))
        return false;

    // get next grid
    SimpleVector3 direction = objectPos - cameraPos;

    SimpleVector3 min, max;

    voxelToWorldBox(currGridPos, min, max);

    double tMin = 2 * mWorldVoxelSize;
    GridVector3 nextGrid = currGridPos;

    for(int i = 0; i < min.rows(); i++)
    {
        if (abs(direction[i]) < EPSILON)
            continue;

        double tCurrent = (min[i] - cameraPos[i]) / direction[i];

        if (abs(tCurrent - tMin) < EPSILON)
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

        if (abs(tCurrent - tMin) < EPSILON)
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
    if (!inVoxelGrid(gridPosition))
        return false;

    voxel_grid::VoxelStatus vs = mVoxelGridPtr->getVoxel(gridPosition[0], gridPosition[1], gridPosition[2]);
    return vs == voxel_grid::VoxelStatus::MARKED;
}

void WorldHelper::worldToVoxelGridCoordinates(const SimpleVector3 &worldPos, GridVector3 &result)
{
    // world to map
    SimpleVector3 mapPosition;
    mMapHelperPtr->worldToMapCoordinates(worldPos, mapPosition);

    // map to grid
    for (int i = 0; i < result.rows(); i++)
    {
        result[i] = (int) (mapPosition[i] / mMapVoxelSize);
    }
}

void WorldHelper::mapToWorldCoordinates(const SimpleVector3 &worldPos, SimpleVector3 &result)
{
    mMapHelperPtr->mapToWorldCoordinates(worldPos, result);
}

void WorldHelper::initializeVoxelGrid(double worldHeight)
{
    mDebugHelper->write("Initializing voxel grid.", DebugHelper::WORLD);
    unsigned int size_x, size_y, size_z;
    size_x = ceil((double) mMapHelperPtr->getMetricWidth() / mWorldVoxelSize);
    size_y = ceil((double) mMapHelperPtr->getMetricHeight() / mWorldVoxelSize);
    size_z = ceil(worldHeight / mWorldVoxelSize);
    mVoxelGridPtr = VoxelGridPtr(new VoxelGrid(size_x, size_y, size_z));
    mDebugHelper->write(std::stringstream() << "Initialized voxel grid with size " << size_x << "x" << size_y << "x" << size_z << ".",
                                                                                                                    DebugHelper::WORLD);
}

void WorldHelper::loadVoxelGrid(std::string filePath)
{
    loadOBJFile(filePath, SimpleVector3(0,0,1), SimpleQuaternion(1,0,0,0));

    mVisHelper.triggerVoxelGridVisualization(mVoxelGridPtr, mWorldVoxelSize, mMapHelperPtr);
}

void WorldHelper::loadOBJFile(string meshResource, SimpleVector3 position, SimpleQuaternion orientation)
{
    mDebugHelper->write(std::stringstream() << "Loading OBJ file " << meshResource, DebugHelper::WORLD);

    mVisHelper.triggerWorldMeshVisualization(meshResource, position, orientation);

    // get file path
    std::string filePath = meshResource.substr(10, meshResource.size() - 10);
    std::vector<std::string> split;
    boost::split(split, filePath, boost::is_any_of("/"));
    std::string packagePath = ros::package::getPath(split[0]);
    filePath = packagePath.substr(0, packagePath.length() - split[0].length()) + filePath;

    // import mesh
    const aiScene* currentScene = NULL;
    Assimp::Importer meshImporter;

    meshImporter.SetPropertyInteger("AI_CONFIG_PP_SBP_REMOVE", aiPrimitiveType_LINE);
    currentScene = meshImporter.ReadFile(filePath, aiProcess_Triangulate |
                                                   aiProcess_ValidateDataStructure |
                                                   aiProcess_SortByPType);

    if (!currentScene)
    {
        ROS_ERROR_STREAM("Could not load file " << filePath);
        return;
    }

    aiMesh* mesh = currentScene->mMeshes[0];

    mDebugHelper->write(std::stringstream() << "Loaded mesh with " << mesh->mNumVertices << " vertices.",
                                                                                        DebugHelper::WORLD);



    // transform mesh and put new vertices in vertices vector
    std::vector<SimpleVector3> vertices;
    for (unsigned int i = 0; i < mesh->mNumVertices; i++)
    {
        aiVector3D aiVertex = mesh->mVertices[i];
        SimpleVector3 vertex = TypeHelper::getSimpleVector3(aiVertex);
        vertex += position;
        vertex = orientation.toRotationMatrix() * vertex;
        vertices.push_back(vertex);
    }

    // load mesh into voxel grid
    mDebugHelper->write("Adding mesh to voxel grid.", DebugHelper::WORLD);
    std::vector<std::vector<SimpleVector3>> faces;
    for (unsigned int i = 0; i < mesh->mNumFaces; i++)
    {
        aiFace face = mesh->mFaces[i];

        std::vector<SimpleVector3> faceVertices;

        for (unsigned int j = 0; j < face.mNumIndices; j++)
        {
            unsigned int index = face.mIndices[j];
            faceVertices.push_back(vertices[index]);
        }

        faces.push_back(faceVertices);

        if (face.mNumIndices == 3)
            addTriangle(faceVertices);
        else if (face.mNumIndices == 1)
            addPoint(faceVertices[0]);
    }

    mVisHelper.triggerWorldTrianglesVisualization(faces);

    mDebugHelper->write(std::stringstream() << "Loaded OBJ file " << filePath, DebugHelper::WORLD);
}

void WorldHelper::addPoint(const SimpleVector3 &point)
{
    mDebugHelper->write("Adding point to voxel grid.", DebugHelper::WORLD);
    GridVector3 gridPos;
    worldToVoxelGridCoordinates(point, gridPos);
    markVoxel(gridPos);
}

void WorldHelper::addTriangle(const std::vector<SimpleVector3> vertices)
{
    mDebugHelper->write("Adding triangle to voxel grid.", DebugHelper::WORLD);
    mDebugHelper->write(std::stringstream() << "Triangle vertices: " << vertices[0] << ", " << vertices[1] << ", " << vertices[2],
                                                                                                                DebugHelper::WORLD);
    // get grid positions of vertices
    GridVector3 verticesGridPositions[3];

    for (unsigned int i = 0; i < 3; i++)
    {
        SimpleVector3 vertex = vertices[i];
        GridVector3 gridPos;
        worldToVoxelGridCoordinates(vertex, gridPos);
        verticesGridPositions[i] = gridPos;
    }

    mDebugHelper->write(std::stringstream() << "Triangle vertices grid positions: " << verticesGridPositions[0] << ", "
                                                        << verticesGridPositions[1] << ", " << verticesGridPositions[2],
                                                        DebugHelper::WORLD);

    // get voxel bounding box of triangle spanning from minPos to maxPos
    GridVector3 minPos(mVoxelGridPtr->sizeX() - 1, mVoxelGridPtr->sizeY() - 1, mVoxelGridPtr->sizeZ() - 1);
    GridVector3 maxPos(0,0,0);
    for (unsigned int i = 0; i < 3; i++)
    {
        GridVector3 gridPos = verticesGridPositions[i];
        for (unsigned int j = 0; j < 3; j++)
        {
            if (gridPos[j] < minPos[j])
                minPos[j] = gridPos[j];
            if (gridPos[j] > maxPos[j])
                maxPos[j] = gridPos[j];
        }
    }

    mDebugHelper->write(std::stringstream() << "Bounding box of triangle spanning from " << minPos << " to " << maxPos, DebugHelper::WORLD);

    // go through each voxel in bounding box and check if it intersects the triangle
    for (int i = minPos[0]; i <= maxPos[0]; i++)
        for (int j = minPos[1]; j <= maxPos[1]; j++)
            for (int k = minPos[2]; k <= maxPos[2]; k++)
            {
                GridVector3 voxel(i,j,k);

                /*
                 * ------------------------------------------------
                 * voxel already marked => continue with next voxel
                 * ------------------------------------------------
                 */
                if (isMarked(voxel))
                    continue;

                /*
                 * ----------------------------------------
                 * 1 or 2 vertices in voxel => intersection
                 * ----------------------------------------
                 */
                unsigned int count = 0;

                for (unsigned int i = 0; i < 3; i++)
                {
                    GridVector3 vertexGridPos = verticesGridPositions[i];
                    if (equalVoxels(voxel, vertexGridPos))
                        count++;
                }

                if (0 < count && count < 3)
                {
                    markVoxel(voxel);
                    continue;
                }

                /*
                 * --------------------------------------
                 * 3 vertices in voxel => no intersection
                 * --------------------------------------
                 */
                if (count == 3)
                    continue;

                /*
                 * ------------------------------------------------------------
                 * at least 1 edge of triangle intersects voxel => intersection
                 * ------------------------------------------------------------
                 */
                // get voxel map box
                SimpleVector3 min, max;
                voxelToWorldBox(voxel, min, max);

                // go through all  triangle edges
                bool marked = false;

                for (unsigned int l = 0; l < 2; l++)
                {
                    for (unsigned int m = l+1; m < 3; m++)
                    {
                        SimpleVector3 vertex1 = vertices[l];
                        SimpleVector3 vertex2 = vertices[m];

                        if (lineIntersectsVoxel(vertex1, vertex2, min, max))
                        {
                            markVoxel(voxel);
                            marked = true;
                            break;
                        }
                    }
                    if (marked)
                        break;
                }

                if (marked)
                    continue;

                /*
                 * ------------------------------------------------------------
                 * at least 1 edge of voxel intersects triangle => intersection
                 * ------------------------------------------------------------
                 */
                // create list of voxel vertices
                SimpleVector3 voxelVertices[8];

                count = 0;

                bool xMin = true;
                while (true)
                {
                    bool yMin = true;
                    while(true)
                    {
                        bool zMin = true;
                        while(true)
                        {
                            double vertexX = xMin ? min[0] : max[0];
                            double vertexY = yMin ? min[1] : max[1];
                            double vertexZ = zMin ? min[2] : max[2];
                            voxelVertices[count] = SimpleVector3(vertexX, vertexY, vertexZ);
                            count++;

                            if (!zMin)
                                break;

                            zMin = false;
                        }

                        if (!yMin)
                            break;

                        yMin = false;
                    }

                    if (!xMin)
                        break;

                    xMin = false;
                }

                assert(count == 8);

                for (unsigned int l = 0; l < 7; l++)
                {
                    for (unsigned int m = l+1; m < 8; m++)
                    {
                        if (voxelVerticesAreNeighbours(voxelVertices[l], voxelVertices[m]))
                        {
                            if (lineIntersectsTriangle(voxelVertices[l], voxelVertices[m], vertices))
                            {
                                markVoxel(voxel);
                                marked = true;
                                break;
                            }
                        }
                    }
                    if (marked)
                        break;
                }

                if (marked)
                    continue;

            }

}

void WorldHelper::markVoxel(const GridVector3 &gridPos)
{
    if (!inVoxelGrid(gridPos))
        return;
    mDebugHelper->write(std::stringstream() << "Marking voxel " << gridPos << ".", DebugHelper::WORLD);
    mVoxelGridPtr->markVoxel(gridPos[0], gridPos[1], gridPos[2]);
}

void WorldHelper::voxelToWorldBox(const GridVector3 &gridPos, SimpleVector3 &min, SimpleVector3 &max)
{
    SimpleVector3 tempMin;
    for (int i = 0; i < tempMin.rows(); i++)
        tempMin[i] = gridPos[i] * mMapVoxelSize;

    SimpleVector3 tempMax = tempMin + SimpleVector3(mMapVoxelSize, mMapVoxelSize, mMapVoxelSize);

    mapToWorldCoordinates(tempMin, min);
    mapToWorldCoordinates(tempMax, max);
}

bool WorldHelper::voxelVerticesAreNeighbours(const SimpleVector3& vertexA, const SimpleVector3& vertexB)
{
    int count = 0;

    for (unsigned int i = 0; i < 3; i++)
    {
        if (abs(vertexA[i] - vertexB[i]) > EPSILON)
            count++;
    }

    return count == 1;
}

bool WorldHelper::lineIntersectsVoxel(const SimpleVector3& lineStartPos, const SimpleVector3& lineEndPos,
                                                const SimpleVector3& voxelMin, const SimpleVector3& voxelMax)
{
    SimpleVector3 direction = lineEndPos - lineStartPos;

    return lineIntersectsVoxelHelper(lineStartPos, direction, voxelMin, voxelMin, voxelMax)
            || lineIntersectsVoxelHelper(lineStartPos, direction, voxelMax, voxelMin, voxelMax);
}

bool WorldHelper::lineIntersectsVoxelHelper(const SimpleVector3& startPos, const SimpleVector3& direction, const SimpleVector3& voxelCoord,
                                                                                const SimpleVector3& voxelMin, const SimpleVector3& voxelMax)
{
    double t = -1.0;
    for(unsigned int i = 0; i < 3; i++)
    {
        // coordinate number i does not change => check if it is in bounds
        if (abs(direction[i]) < EPSILON)
        {
            if (startPos[i] >= voxelMin[i] - EPSILON && startPos[i] <= voxelMax[i] + EPSILON)
                continue;
            else
            {
                t = -1.0;
                break;
            }
        }

        // t was not set yet => set it now (i == 0 or the coordinates before did not change)
        if (t < 0.0)
        {
            t = (voxelCoord[i] - startPos[i]) / direction[i];

            if (t < -EPSILON || t > 1.0 + EPSILON)
            {
                t = -1.0;
                break;
            }
            else
                continue;
        }

        // t was set and coordinate number i does change => check if coordinate number i for given t is in bounds
        double coordinate = startPos[i] + t * direction[i];

        if (coordinate >= voxelMin[i] - EPSILON && coordinate <= voxelMax[i] + EPSILON)
            continue;
        else
        {
            t = -1.0;
            break;
        }
    }

    return t >= -EPSILON;
}

bool WorldHelper::lineIntersectsTriangle(const SimpleVector3& lineStartPos, const SimpleVector3& lineEndPos, const std::vector<SimpleVector3> triangleVertices)
{
    // get normal form of triangle plane (p dot normal + d = 0)
    SimpleVector3 direction1 = triangleVertices[1] - triangleVertices[0];
    SimpleVector3 direction2 = triangleVertices[2] - triangleVertices[0];

    SimpleVector3 normal = direction1.cross(direction2);
    double d = -lineStartPos.dot(normal);

    // get intersection point between line and plane
    SimpleVector3 intersection;

    SimpleVector3 lineDirection = lineEndPos - lineStartPos;

    double a = lineDirection.dot(normal);
    double b = lineStartPos.dot(normal) + d;

    if (abs(a) < EPSILON)
    {
        if (abs(b) < EPSILON)
            intersection = lineStartPos;
        else
            return false;
    }
    else
    {
        double t = -b / a;
        intersection = lineStartPos + t * lineDirection;
    }

    // check if intersection point is inside triangle
    for (unsigned int i = 0; i < 3; i++)
    {
        int j = (i+1) % 3;
        // TODO check if vector order is right (v1 and v2)
        SimpleVector3 v1 = triangleVertices[i] - lineStartPos;
        SimpleVector3 v2 = triangleVertices[j] - lineStartPos;
        SimpleVector3 v3 = intersection - lineStartPos;

        if (v1.cross(v2).dot(v3) < 0)
            return false;
    }

    return true;
}

bool WorldHelper::inVoxelGrid(const GridVector3 &gridPos)
{
    if (gridPos[0] < 0 || gridPos[1] < 0 || gridPos[2] < 0)
        return false;

    if (gridPos[0] >= mVoxelGridPtr->sizeX() || gridPos[1] >= mVoxelGridPtr->sizeY() ||
            gridPos[2] >= mVoxelGridPtr->sizeZ())
        return false;

    return true;
}

bool WorldHelper::equalVoxels(const GridVector3 &a, const GridVector3 &b)
{
    return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}

}
