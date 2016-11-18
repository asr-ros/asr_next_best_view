#include "next_best_view/helper/WorldHelper.hpp"

namespace next_best_view {

WorldHelper::WorldHelper(next_best_view::MapHelperPtr mapHelperPtr, std::string filePath, double voxelSize, double worldHeight, bool visualizeRaytracing) :
    mMapHelperPtr(mapHelperPtr), mWorldVoxelSize(voxelSize), mVisualizeRaytracing(visualizeRaytracing)
{
    mVisHelperPtr = VisualizationHelperPtr(new VisualizationHelper(mMapHelperPtr));
    mDebugHelperPtr = DebugHelper::getInstance();

    mMapHelperPtr->worldToMapSize(voxelSize, mMapVoxelSize);

    initializeVoxelGridHelper(worldHeight);
    loadVoxelGrid(filePath);
}

void WorldHelper::filterOccludedObjects(SimpleVector3 cameraPosition, ObjectPointCloudPtr objectPointCloud, IndicesPtr indices,
                                                                                                                IndicesPtr &filteredIndices)
{
    // summarize objects in voxels
    ObjectVoxelToIndices voxelToIndices;

    BOOST_FOREACH(int index, *indices)
    {
        ObjectPoint &point = objectPointCloud->at(index);
        std::vector<GridVector3> voxels;
        worldToVoxelGridCoordinates(point.getPosition(), voxels);

        BOOST_FOREACH(GridVector3 voxel, voxels)
        {
            VoxelTuple voxelTuple = std::make_tuple(voxel[0], voxel[1], voxel[2]);
            if (!voxelToIndices[voxelTuple])
                voxelToIndices[voxelTuple] = IndicesPtr(new Indices());
            voxelToIndices[voxelTuple]->push_back(index);
        }
    }

    // check occlusion for each voxel containing objects
    filteredIndices = IndicesPtr(new Indices());
    std::vector<GridVector3> cameraVoxels;
    worldToVoxelGridCoordinates(cameraPosition, cameraVoxels);

    BOOST_FOREACH(ObjectVoxelAndIndices objectVoxelAndIndices, voxelToIndices)
    {
        VoxelTuple objectVoxel = objectVoxelAndIndices.first;
        GridVector3 targetVoxel(std::get<0>(objectVoxel), std::get<1>(objectVoxel), std::get<2>(objectVoxel));

        SimpleVector3 targetPosition = voxelToWorldPosition(targetVoxel);

        BOOST_FOREACH(GridVector3 cameraVoxel, cameraVoxels)
        {
            VoxelTuple sourceVoxel = std::make_tuple(cameraVoxel[0], cameraVoxel[1], cameraVoxel[2]);
            VoxelTuplesPtr &occludedVoxels = cameraVoxelToOccludedVoxels[sourceVoxel];
            VoxelTuplesPtr &unoccludedVoxels = cameraVoxelToUnoccludedVoxels[sourceVoxel];

            if (occludedVoxels &&
                    std::find(occludedVoxels->begin(), occludedVoxels->end(), objectVoxel) != occludedVoxels->end())
                continue;
            if (unoccludedVoxels &&
                    std::find(unoccludedVoxels->begin(), unoccludedVoxels->end(), objectVoxel) != unoccludedVoxels->end())
            {
                IndicesPtr objectIndices = objectVoxelAndIndices.second;
                insert(objectIndices, filteredIndices);
                continue;
            }


            SimpleVector3 sourcePosition = voxelToWorldPosition(cameraVoxel);

            if (!isOccluded(sourcePosition, targetPosition))
            {
                IndicesPtr objectIndices = objectVoxelAndIndices.second;
                insert(objectIndices, filteredIndices);
                if(!unoccludedVoxels)
                    unoccludedVoxels = VoxelTuplesPtr(new VoxelTuples());
                unoccludedVoxels->push_back(objectVoxel);
            }
            else
            {
                if(!occludedVoxels)
                    occludedVoxels = VoxelTuplesPtr(new VoxelTuples());
                occludedVoxels->push_back(objectVoxel);
            }
        }

    }
}

bool WorldHelper::isOccluded(SimpleVector3 cameraPosition, SimpleVector3 targetPosition)
{
    std::vector<GridVector3> cameraGridPositions;

    worldToVoxelGridCoordinates(cameraPosition, cameraGridPositions);

    std::vector<GridVector3> traversedVoxels;

    bool occluded = isOccluded(cameraPosition, targetPosition, cameraGridPositions[0], traversedVoxels);

    if (mVisualizeRaytracing)
        mVisHelperPtr->triggerRaytracingVisualization(cameraPosition, targetPosition, traversedVoxels, occluded, mWorldVoxelSize);

    return occluded;
}

bool WorldHelper::isOccluded(SimpleVector3 cameraPos, SimpleVector3 targetPos,
                             GridVector3 currVoxelPos, std::vector<GridVector3>& traversedVoxels)
{
    traversedVoxels.push_back(currVoxelPos);

    if (mVoxelGridHelperPtr->isMarked(currVoxelPos))
        return true;

    // go through all voxels surrounding this voxel
    for (int i = -1; i < 2; i++)
        for (int j = -1; j < 2; j++)
            for (int k = -1; k < 2; k++)
            {
                if (i == 0 && j == 0 && k == 0)
                    continue;

                GridVector3 voxel(i, j, k);

                voxel += currVoxelPos;

                // check if voxel was already traversed
                bool traversed = false;
                for (unsigned int i = 0; i < traversedVoxels.size(); i++)
                {
                    if (equalVoxels(voxel, traversedVoxels[i]))
                    {
                        traversed = true;
                        break;
                    }
                }

                if (traversed)
                    continue;

                bool checkVoxel = false;

                if (lineIntersectsVoxel(cameraPos, targetPos, voxel))
                    checkVoxel = true;

                if (checkVoxel)
                {
                    bool occluded = isOccluded(cameraPos, targetPos, voxel, traversedVoxels);

                    if (occluded)
                        return true;
                }

            }

    return false;
}

void WorldHelper::worldToVoxelGridCoordinates(const SimpleVector3 &worldPos, std::vector<GridVector3> &result)
{
    // world to map
    SimpleVector3 mapPosition;
    mMapHelperPtr->worldToMapCoordinates(worldPos, mapPosition);

    // map to grid

    // voxel that is always occupied by worldPos
    GridVector3 gridPos;
    for (int i = 0; i < 3; i++)
    {
        gridPos[i] = (int) (mapPosition[i] / mMapVoxelSize);
    }

    result.push_back(gridPos);

    // voxels that could also be occupied by worldPos

    // change 1 coordinate
    for (int i = 0; i < 3; i++)
    {
        if (fmod(mapPosition[i], mMapVoxelSize) < EPSILON)
        {
            GridVector3 additionalGridPos = gridPos;
            additionalGridPos[i] -= 1;
            result.push_back(additionalGridPos);
        }
    }

    unsigned int size = result.size();
    if (size == 1)
        return;

    // change 2 coordinates
    for (unsigned int i = 1; i < size - 1; i++)
        for (unsigned int j = i + 1; j < size; j++)
        {
            GridVector3 gridPos1 = result[i];
            GridVector3 gridPos2 = result[j];
            GridVector3 additionalGridPos = gridPos1;
            for (unsigned int k = 0; k < 3; k++)
            {
                if (gridPos1[k] < gridPos2[k])
                    additionalGridPos[k] -= 1;
            }
            result.push_back(additionalGridPos);
        }

    assert((size == 3 && result.size() == 4) || (size == 4 && result.size() == 7));

    // change all 3 coordinates
    if (size == 4)
    {
        GridVector3 additionalGridPos = gridPos;

        for (unsigned int i = 0; i < 3; i++)
        {
            additionalGridPos[i] -= 1;
        }

        result.push_back(additionalGridPos);
    }
}

void WorldHelper::worldToVoxelGridCoordinates(const std::vector<SimpleVector3> &worldPositions, std::vector<GridVector3> &results)
{
    // get grid positions of vertices
    results.clear();

    for (unsigned int i = 0; i < worldPositions.size(); i++)
    {
        SimpleVector3 worldPos = worldPositions[i];
        std::vector<GridVector3> gridPositions;
        worldToVoxelGridCoordinates(worldPos, gridPositions);
        results.insert(results.end(), gridPositions.begin(), gridPositions.end());
    }
}

// TODO remove
void WorldHelper::mapToWorldCoordinates(const SimpleVector3 &mapPos, SimpleVector3 &result)
{
    mMapHelperPtr->mapToWorldCoordinates(mapPos, result);
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

    mVisHelperPtr->triggerWorldMeshesVisualization(meshResources, positions, orientations, scales);

    auto begin = std::chrono::high_resolution_clock::now();

    std::vector<std::vector<SimpleVector3>> faces;

    for (unsigned int i = 0; i < meshResources.size(); i++)
    {
        loadMeshFile(meshResources[i], positions[i], orientations[i], scales[i], faces);
    }

    auto finish = std::chrono::high_resolution_clock::now();
    ROS_INFO_STREAM("Loading of mesh files into voxel grid took " << std::chrono::duration<float>(finish-begin).count() << " seconds.");

    mVisHelperPtr->triggerWorldTrianglesVisualization(faces);

    mVisHelperPtr->triggerVoxelGridVisualization(mVoxelGridHelperPtr, mWorldVoxelSize);
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

    for (unsigned int i = 0; i < currentScene->mNumMeshes; i++)
    {
        aiMesh* mesh = currentScene->mMeshes[i];

        mDebugHelperPtr->write(std::stringstream() << "Loaded mesh with " << mesh->mNumVertices << " vertices.",
                                                                                            DebugHelper::WORLD);

        SimpleQuaternion normalizedOrientation(0,0,0,0);
        if (!(orientation.norm() < EPSILON))
            normalizedOrientation = orientation.normalized();

        // transform mesh and put new vertices in vertices vector
        std::vector<SimpleVector3> vertices;
        for (unsigned int j = 0; j < mesh->mNumVertices; j++)
        {
            aiVector3D aiVertex = mesh->mVertices[j];
            SimpleVector3 vertex = TypeHelper::getSimpleVector3(aiVertex);
            vertex = vertex.cwiseProduct(scale);
            if (!orientation.norm() < EPSILON)
                vertex = normalizedOrientation.toRotationMatrix() * vertex;
            vertex += position;
            vertices.push_back(vertex);
        }

        // load mesh into voxel grid
        mDebugHelperPtr->write("Adding mesh to voxel grid.", DebugHelper::WORLD);
        for (unsigned int j = 0; j < mesh->mNumFaces; j++)
        {
            aiFace face = mesh->mFaces[j];

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
    }


    mDebugHelperPtr->write(std::stringstream() << "Loaded OBJ file " << filePath, DebugHelper::WORLD);
}

void WorldHelper::addPoint(const SimpleVector3 &point)
{
    std::vector<GridVector3> gridPositions;
    worldToVoxelGridCoordinates(point, gridPositions);
    for (unsigned int i = 0; i < gridPositions.size(); i++)
        mVoxelGridHelperPtr->markVoxel(gridPositions[i]);
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
                 * at least 1 vertex in voxel => intersection
                 * ----------------------------------------
                 */
                bool marked = false;

                for (unsigned int i = 0; i < verticesGridPositions.size(); i++)
                {
                    GridVector3 vertexGridPos = verticesGridPositions[i];
                    if (equalVoxels(voxel, vertexGridPos))
                    {
                        mVoxelGridHelperPtr->markVoxel(voxel);
                        marked = true;
                        break;
                    }
                }

                if (marked)
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
                 * ------------------------------------------
                 * at least 1 vertex in voxel => intersection
                 * ------------------------------------------
                 */
                bool marked = false;

                for (unsigned int i = 0; i < verticesGridPositions.size(); i++)
                {
                    GridVector3 vertexGridPos = verticesGridPositions[i];
                    if (equalVoxels(voxel, vertexGridPos))
                    {
                        mVoxelGridHelperPtr->markVoxel(voxel);
                        marked = true;
                        break;
                    }
                }

                if (marked)
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

                for (unsigned int l = 0; l < 2; l++)
                {
                    for (unsigned int m = l+1; m < 3; m++)
                    {
                        SimpleVector3 vertex1 = vertices[l];
                        SimpleVector3 vertex2 = vertices[m];

                        if (lineIntersectsVoxel(vertex1, vertex2, voxel))
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

                unsigned int count = 0;

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

            }

}

SimpleVector3 WorldHelper::voxelToWorldPosition(const GridVector3 &gridPos)
{
    // voxel to map position
    SimpleVector3 mapPos;
    for (int i = 0; i < mapPos.rows(); i++)
        mapPos[i] = (gridPos[i] + 0.5) * mMapVoxelSize;

    SimpleVector3 result;
    mapToWorldCoordinates(mapPos, result);
    return result;
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
    double t;
    return lineIntersectsVoxel(lineStartPos, lineEndPos, gridPos, 0, t);
}

bool WorldHelper::lineIntersectsVoxel(const SimpleVector3 &lineStartPos, const SimpleVector3 &lineEndPos, const GridVector3 &gridPos,
                                                                                                                    double tLowerBound)
{
    double t;
    // choose tLowerBound = -2 * EPSILON so that only values t < -EPSILON are excluded
    return lineIntersectsVoxel(lineStartPos, lineEndPos, gridPos, tLowerBound, t);
}

bool WorldHelper::lineIntersectsVoxel(const SimpleVector3 &lineStartPos, const SimpleVector3 &lineEndPos, const GridVector3 &gridPos,
                                                                                                        double tLowerBound, double &tMin)
{
    SimpleVector3 voxelMin, voxelMax;
    voxelToWorldBox(gridPos, voxelMin, voxelMax);

    double t1, t2;

    bool intersectsMin = lineIntersectsVoxelHelper(lineStartPos, lineEndPos, voxelMin, voxelMin, voxelMax, tLowerBound, t1);
    bool intersectsMax = lineIntersectsVoxelHelper(lineStartPos, lineEndPos, voxelMax, voxelMin, voxelMax, tLowerBound, t2);

    tMin = std::min(t1, t2);

    return intersectsMin || intersectsMax;
}

bool WorldHelper::lineIntersectsVoxelHelper(const SimpleVector3& lineStartPos, const SimpleVector3& lineEndPos, const SimpleVector3& voxelCoord,
                                                    const SimpleVector3& voxelMin, const SimpleVector3& voxelMax, double tLowerBound, double &tMin)
{
    SimpleVector3 direction = lineEndPos - lineStartPos;

    bool intersection = false;
    tMin = 1 + EPSILON;
    for(unsigned int i = 0; i < 3; i++)
    {
        // coordinate number i does not change
        if (abs(direction[i]) < EPSILON)
            continue;

        // set current t
        double t = (voxelCoord[i] - lineStartPos[i]) / direction[i];

        // check if t is candidate for tMin
        if (t < tLowerBound - EPSILON || t > tMin)
            continue;

        // t was set => check if resulting coordinate is in voxel bounds
        SimpleVector3 coordinate = lineStartPos + t * direction;

        bool inBounds = true;
        for (unsigned int j = 0; j < 3; j++)
        {
            if (j == i)
                continue;

            if (coordinate[j] < voxelMin[j] - EPSILON || coordinate[j] > voxelMax[j] + EPSILON)
                inBounds = false;
        }

        if (inBounds)
        {
            tMin = t;
            intersection = true;
        }
    }

    return intersection;
}

bool WorldHelper::lineIntersectsTriangle(const SimpleVector3& lineStartPos, const SimpleVector3& lineEndPos, const std::vector<SimpleVector3> triangleVertices)
{
    // get normal form of triangle plane (p dot normal + d = 0)
    SimpleVector3 direction1 = triangleVertices[1] - triangleVertices[0];
    SimpleVector3 direction2 = triangleVertices[2] - triangleVertices[0];

    SimpleVector3 normal = direction1.cross(direction2);
    double d = -triangleVertices[0].dot(normal);

    // get intersection point between line and plane
    SimpleVector3 intersection;

    SimpleVector3 lineDirection = lineEndPos - lineStartPos;

    double a = lineDirection.dot(normal);
    double b = lineStartPos.dot(normal) + d;

    if (abs(a) < EPSILON)
    {
        // line parallel to triangle
        if (abs(lineStartPos.dot(normal) + d) < EPSILON)
        {
            // intersection in one plain
            std::vector<double> tList;

            for (int i = 0; i < 2; i++)
                for (int j = 1; j < 3; j++)
                {
                    SimpleVector3 triangleEdge = triangleVertices[j] - triangleVertices[i];

                    // check linear independence between triangleEdge and lineDirection
                    double coeff = 0;
                    for (int k = 0; k < 3; k++)
                    {
                        if (abs(triangleEdge[k]) < EPSILON || abs(lineDirection[k]) < EPSILON)
                        {
                            if (abs(triangleEdge[k]) < EPSILON && abs(lineDirection[k]) < EPSILON)
                                continue;
                            coeff = 0;
                            break;
                        }

                        if (abs(coeff) < EPSILON)
                        {
                            // coeff not set -> set it
                            coeff = triangleEdge[k] / lineDirection[k];
                        }
                        else if (abs(coeff - triangleEdge[k] / lineDirection[k]) > EPSILON)
                        {
                            // coeff differs -> linearly independant
                            coeff = 0;
                            break;
                        }
                    }

                    if (abs(coeff) < EPSILON)
                    {
                        Eigen::Matrix<double, 3, 2> A;
                        Eigen::Matrix<double, 3, 1> b;

                        A << lineDirection[0], -triangleEdge[0],
                                lineDirection[1], -triangleEdge[1],
                                lineDirection[2], -triangleEdge[2];

                        b << triangleVertices[i][0] - lineStartPos[0],
                                triangleVertices[i][1] - lineStartPos[1],
                                triangleVertices[i][2] - lineStartPos[2];

                        Eigen::Matrix<double, 2, 1> solution = A.colPivHouseholderQr().solve(b);

                        double t1 = solution[0];
                        double t2 = solution[1];

                        if (t2 > -EPSILON && t2 < 1 + EPSILON)
                        {
                            if (t1 > -EPSILON && t1 < 1 + EPSILON)
                            {
                                return true;
                            }
                            else
                            {
                                tList.push_back(t1);
                            }
                        }
                    }

                }

            if (tList.size() == 0)
                return false;

            double tMax = -DBL_MAX;
            double tMin = DBL_MAX;

            for (unsigned int i = 0; i < tList.size(); i++)
            {
                if (tList[i] > tMax)
                    tMax = tList[i];
                if (tList[i] < tMin)
                    tMin = tList[i];
            }

            if (tMin > 1 + EPSILON || tMax < -EPSILON)
                return false;

            return true;
        }
        else
        {
            // line is not in triangle plain
            return false;
        }
    }

    // line not parallel to triangle
    double t = -b / a;

    if (t < -EPSILON || t > 1 + EPSILON)
        return false;

    intersection = lineStartPos + t * lineDirection;

    // check if intersection point is inside triangle
    for (unsigned int i = 0; i < 3; i++)
    {
        SimpleVector3 v1 = triangleVertices[i] - lineStartPos;
        SimpleVector3 v2 = triangleVertices[(i+1) % 3] - lineStartPos;
        SimpleVector3 v3 = triangleVertices[(i+2) % 3] - lineStartPos;

        // normal of plane
        SimpleVector3 n = v1.cross(v2);

        // check if vector order is right
        if (n.dot(v3) < 0)
            n = v2.cross(v1);

        // check if intersection point is on the right side of the plane
        SimpleVector3 vIntersection = intersection - lineStartPos;

        if (n.dot(vIntersection) < 0)
            return false;
    }

    return true;
}

bool WorldHelper::equalVoxels(const GridVector3 &a, const GridVector3 &b)
{
    return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}

void WorldHelper::insert(IndicesPtr source, IndicesPtr &target)
{
    BOOST_FOREACH(int index, *source)
    {
        if (std::find(target->begin(), target->end(), index) == target->end())
            target->push_back(index);
    }
}

}
