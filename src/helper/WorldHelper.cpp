#include "next_best_view/helper/WorldHelper.hpp"

namespace next_best_view {

WorldHelper::WorldHelper(next_best_view::MapHelperPtr mapHelperPtr, std::string filePath, double voxelSize, double worldHeight) :
    mMapHelperPtr(mapHelperPtr), mWorldVoxelSize(voxelSize)
{
    mDebugHelperPtr = DebugHelper::getInstance();

    mMapHelperPtr->worldToMapSize(voxelSize, mMapVoxelSize);

    initializeVoxelGridHelper(worldHeight);
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
    if (mVoxelGridHelperPtr->isMarked(currGridPos))
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

void WorldHelper::worldToVoxelGridCoordinates(const std::vector<SimpleVector3> &worldPositions, std::vector<GridVector3> &results)
{
    // get grid positions of vertices
    results.clear();

    for (unsigned int i = 0; i < worldPositions.size(); i++)
    {
        SimpleVector3 worldPos = worldPositions[i];
        GridVector3 gridPos;
        worldToVoxelGridCoordinates(worldPos, gridPos);
        results.push_back(gridPos);
    }
}

void WorldHelper::mapToWorldCoordinates(const SimpleVector3 &worldPos, SimpleVector3 &result)
{
    mMapHelperPtr->mapToWorldCoordinates(worldPos, result);
}

void WorldHelper::initializeVoxelGridHelper(double worldHeight)
{
    mDebugHelperPtr->write("Initializing voxel grid.", DebugHelper::WORLD);
    unsigned int size_x, size_y, size_z;
    size_x = ceil((double) mMapHelperPtr->getMetricWidth() / mWorldVoxelSize);
    size_y = ceil((double) mMapHelperPtr->getMetricHeight() / mWorldVoxelSize);
    size_z = ceil(worldHeight / mWorldVoxelSize);

    mVoxelGridHelperPtr = VoxelGridHelperPtr(new VoxelGridHelper(GridVector3(size_x, size_y, size_z)));

    mDebugHelperPtr->write(std::stringstream() << "Initialized voxel grid with size " << size_x << "x" << size_y << "x" << size_z << ".",
                                                                                                                    DebugHelper::WORLD);
}

void WorldHelper::loadVoxelGrid(std::string filePath)
{
    std::vector<std::string> meshResources;
    std::vector<SimpleVector3> positions;
    std::vector<SimpleQuaternion> orientations;
    std::vector<SimpleVector3> scales;

    parseXMLFile(filePath, meshResources, positions, orientations, scales);

    mVisHelper.triggerWorldMeshesVisualization(meshResources, positions, orientations, scales);

    std::vector<std::vector<SimpleVector3>> faces;

    for (unsigned int i = 0; i < meshResources.size(); i++)
    {
        loadMeshFile(meshResources[i], positions[i], orientations[i], scales[i], faces);
    }

    mVisHelper.triggerWorldTrianglesVisualization(faces);

    mVisHelper.triggerVoxelGridVisualization(mVoxelGridHelperPtr, mWorldVoxelSize, mMapHelperPtr);
}

void WorldHelper::parseXMLFile(const string &filePath, std::vector<std::string> &meshResources,
                                                            std::vector<SimpleVector3> &positions,
                                                            std::vector<SimpleQuaternion> &orientations,
                                                            std::vector<SimpleVector3> &scales)
{
    meshResources.clear();
    positions.clear();
    orientations.clear();

    std::string absFilePath = getAbsolutePath(filePath);

    mDebugHelperPtr->write(std::stringstream() << "Parsing XML file: " << absFilePath, DebugHelper::WORLD);

    try
    {
        rapidxml::file<> xmlFile(absFilePath.c_str());
        rapidxml::xml_document<> xmlDoc;
        xmlDoc.parse<0>(xmlFile.data());

        rapidxml::xml_node<> *rootNode = xmlDoc.first_node();
        if (rootNode)
        {
            rapidxml::xml_node<> *childNode = rootNode->first_node();

            while (childNode)
            {
                rapidxml::xml_attribute<> *scaleAttribute = childNode->first_attribute("scale");
                rapidxml::xml_attribute<> *meshAttribute = childNode->first_attribute("mesh");
                if (scaleAttribute && meshAttribute)
                {
                    std::string meshResource = meshAttribute->value();
                    std::string scale = scaleAttribute->value();
                    std::string poseString = childNode->value();
                    std::vector<double> poseVec;
                    std::vector<double> scaleVec;
                    if (parsePoseString(poseString, poseVec) && poseVec.size() == 7)
                    {
                        if (parsePoseString(scale, scaleVec) && scaleVec.size() == 3)
                        {
                            SimpleVector3 position(poseVec[0], poseVec[1], poseVec[2]);
                            SimpleQuaternion orientation(poseVec[3], poseVec[4], poseVec[5], poseVec[6]);
                            SimpleVector3 scale(scaleVec[0], scaleVec[1], scaleVec[2]);

                            meshResources.push_back(meshResource);
                            positions.push_back(position);
                            orientations.push_back(orientation);
                            scales.push_back(scale);
                        }
                    }

                }
                childNode = childNode->next_sibling();
            }
        }
    }
    catch(std::runtime_error err)
    {
        ROS_ERROR_STREAM("Can't parse xml-file. Runtime error: " << err.what());
    }
    catch (rapidxml::parse_error err)
    {
        ROS_ERROR_STREAM("Can't parse xml-file Parse error: " << err.what());
    }
    catch (boost::bad_lexical_cast err)
    {
        ROS_ERROR_STREAM("Can't cast use_mat. Cast error: " << err.what());
    }

    assert(meshResources.size() == positions.size() && positions.size() == orientations.size() && orientations.size() == scales.size());

    mDebugHelperPtr->write(std::stringstream() << "Got " << meshResources.size() << " objects from XML file.", DebugHelper::WORLD);

}

bool WorldHelper::parsePoseString(const string &poseString, std::vector<double> &poseVec)
{
    poseVec.clear();

    std::vector<std::string> strVec;
    boost::algorithm::split(strVec, poseString, boost::algorithm::is_any_of(","));

    BOOST_FOREACH(std::string str, strVec)
    {
        try
        {
            poseVec.push_back(boost::lexical_cast<double>(str));
        }
        catch (boost::bad_lexical_cast err)
        {
            ROS_ERROR_STREAM("Can't cast node-value. Cast error: " << err.what());
            poseVec.clear();
            return false;
        }
    }

    return true;
}

string WorldHelper::getAbsolutePath(string filePath)
{
    std::string result;
    if (boost::starts_with(filePath, "package://"))
    {
        result = filePath.substr(10);
        std::vector<std::string> split;
        boost::split(split, result, boost::is_any_of("/"));
        std::string packagePath = ros::package::getPath(split[0]);
        result = packagePath.substr(0, packagePath.length() - split[0].length()) + result;
    }
    else if (boost::starts_with(filePath, "."))
    {
        result = ros::package::getPath("next_best_view") + filePath.substr(1);
    }
    else
    {
        result = filePath;
    }

    return result;
}

void WorldHelper::loadMeshFile(const std::string &meshResource, const SimpleVector3 &position,
                                const SimpleQuaternion &orientation, const SimpleVector3 &scale,
                                std::vector<std::vector<SimpleVector3>> &faces)
{
    mDebugHelperPtr->write(std::stringstream() << "Loading OBJ file " << meshResource, DebugHelper::WORLD);

    // get file path
    std::string filePath = getAbsolutePath(meshResource);

    // import mesh
    const aiScene* currentScene = NULL;
    Assimp::Importer meshImporter;

    currentScene = meshImporter.ReadFile(filePath, aiProcess_Triangulate |
                                                   aiProcess_ValidateDataStructure);

    if (!currentScene)
    {
        ROS_ERROR_STREAM("Could not load file " << filePath);
        return;
    }

    aiMesh* mesh = currentScene->mMeshes[0];

    mDebugHelperPtr->write(std::stringstream() << "Loaded mesh with " << mesh->mNumVertices << " vertices.",
                                                                                        DebugHelper::WORLD);

    SimpleQuaternion normalizedOrientation(0,0,0,0);
    if (!(orientation.norm() < EPSILON))
        normalizedOrientation = orientation.normalized();

    // transform mesh and put new vertices in vertices vector
    std::vector<SimpleVector3> vertices;
    for (unsigned int i = 0; i < mesh->mNumVertices; i++)
    {
        aiVector3D aiVertex = mesh->mVertices[i];
        SimpleVector3 vertex = TypeHelper::getSimpleVector3(aiVertex);
        vertex = vertex.cwiseProduct(scale);
        if (!orientation.norm() < EPSILON)
            vertex = normalizedOrientation.toRotationMatrix() * vertex;
        vertex += position;
        vertices.push_back(vertex);
    }

    // load mesh into voxel grid
    mDebugHelperPtr->write("Adding mesh to voxel grid.", DebugHelper::WORLD);
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
        else if (face.mNumIndices == 2)
            addLine(faceVertices);
        else if (face.mNumIndices == 1)
            addPoint(faceVertices[0]);
    }

    mDebugHelperPtr->write(std::stringstream() << "Loaded OBJ file " << filePath, DebugHelper::WORLD);
}

void WorldHelper::addPoint(const SimpleVector3 &point)
{
    GridVector3 gridPos;
    worldToVoxelGridCoordinates(point, gridPos);
    mVoxelGridHelperPtr->markVoxel(gridPos);
}

void WorldHelper::addLine(const std::vector<SimpleVector3> &vertices)
{
    std::vector<GridVector3> verticesGridPositions;
    worldToVoxelGridCoordinates(vertices, verticesGridPositions);

    // get voxel bounding box of triangle spanning from minPos to maxPos
    GridVector3 minPos, maxPos;

    voxelGridBox(verticesGridPositions, minPos, maxPos);

    // go through each voxel in bounding box and check if it intersects the line
    for (int i = minPos[0]; i <= maxPos[0]; i++)
        for (int j = minPos[1]; j <= maxPos[1]; j++)
            for (int k = minPos[2]; k <= maxPos[2]; k++)
            {
                GridVector3 voxel(i,j,k);

                /*
                 * -----------------------------------------------------------
                 * voxel outside voxel grid bounds => continue with next voxel
                 * -----------------------------------------------------------
                 */
                if (!mVoxelGridHelperPtr->inVoxelGrid(voxel))
                    continue;

                /*
                 * ------------------------------------------------
                 * voxel already marked => continue with next voxel
                 * ------------------------------------------------
                 */
                if (mVoxelGridHelperPtr->isMarked(voxel))
                    continue;

                /*
                 * ----------------------------------------
                 * 1 vertex in voxel => intersection
                 * ----------------------------------------
                 */
                unsigned int count = 0;

                for (unsigned int i = 0; i < 2; i++)
                {
                    GridVector3 vertexGridPos = verticesGridPositions[i];
                    if (equalVoxels(voxel, vertexGridPos))
                        count++;
                }

                if (count == 1)
                {
                    mVoxelGridHelperPtr->markVoxel(voxel);
                    continue;
                }

                /*
                 * --------------------------------------
                 * 2 vertices in voxel => no intersection
                 * --------------------------------------
                 */
                if (count == 2)
                    continue;

                /*
                 * -------------------------------
                 * general check for interesection
                 * -------------------------------
                 */
                if (lineIntersectsVoxel(vertices[0], vertices[1], voxel))
                {
                    mVoxelGridHelperPtr->markVoxel(voxel);
                }
            }
}

void WorldHelper::addTriangle(const std::vector<SimpleVector3>& vertices)
{
    // get grid positions of vertices
    std::vector<GridVector3> verticesGridPositions;

    worldToVoxelGridCoordinates(vertices, verticesGridPositions);

    // get voxel bounding box of triangle spanning from minPos to maxPos
    GridVector3 minPos, maxPos;

    voxelGridBox(verticesGridPositions, minPos, maxPos);

    // go through each voxel in bounding box and check if it intersects the triangle
    for (int i = minPos[0]; i <= maxPos[0]; i++)
        for (int j = minPos[1]; j <= maxPos[1]; j++)
            for (int k = minPos[2]; k <= maxPos[2]; k++)
            {
                GridVector3 voxel(i,j,k);

                /*
                 * -----------------------------------------------------------
                 * voxel outside voxel grid bounds => continue with next voxel
                 * -----------------------------------------------------------
                 */
                if (!mVoxelGridHelperPtr->inVoxelGrid(voxel))
                    continue;

                /*
                 * ------------------------------------------------
                 * voxel already marked => continue with next voxel
                 * ------------------------------------------------
                 */
                if (mVoxelGridHelperPtr->isMarked(voxel))
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
                    mVoxelGridHelperPtr->markVoxel(voxel);
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
                            mVoxelGridHelperPtr->markVoxel(voxel);
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
                                mVoxelGridHelperPtr->markVoxel(voxel);
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

void WorldHelper::voxelToWorldBox(const GridVector3 &gridPos, SimpleVector3 &min, SimpleVector3 &max)
{
    SimpleVector3 tempMin;
    for (int i = 0; i < tempMin.rows(); i++)
        tempMin[i] = gridPos[i] * mMapVoxelSize;

    SimpleVector3 tempMax = tempMin + SimpleVector3(mMapVoxelSize, mMapVoxelSize, mMapVoxelSize);

    mapToWorldCoordinates(tempMin, min);
    mapToWorldCoordinates(tempMax, max);
}

void WorldHelper::voxelGridBox(const std::vector<GridVector3> &gridPositions, GridVector3 &min, GridVector3 &max)
{
    GridVector3 gridSize = mVoxelGridHelperPtr->getVoxelGridSize();
    min = GridVector3(gridSize[0] - 1, gridSize[1] - 1, gridSize[2] - 1);
    max = GridVector3(0,0,0);

    for (unsigned int i = 0; i < gridPositions.size(); i++)
    {
        GridVector3 gridPos = gridPositions[i];
        for (unsigned int j = 0; j < 3; j++)
        {
            if (gridPos[j] < min[j])
                min[j] = gridPos[j];
            if (gridPos[j] > max[j])
                max[j] = gridPos[j];
        }
    }
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

bool WorldHelper::lineIntersectsVoxel(const SimpleVector3 &lineStartPos, const SimpleVector3 &lineEndPos, const GridVector3 &gridPos)
{
    SimpleVector3 min, max;
    voxelToWorldBox(gridPos, min, max);
    return lineIntersectsVoxel(lineStartPos, lineEndPos, min, max);
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

bool WorldHelper::equalVoxels(const GridVector3 &a, const GridVector3 &b)
{
    return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}

}
