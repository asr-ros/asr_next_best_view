#pragma once

#include "typedef.hpp"
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/range/algorithm_ext/iota.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "next_best_view/camera_model_filter/CameraModelFilter.hpp"
#include "next_best_view/helper/MarkerHelper.hpp"
#include "next_best_view/helper/DebugHelper.hpp"
#include "next_best_view/helper/TypeHelper.hpp"
#include "next_best_view/helper/VoxelGridHelper.hpp"
#include "next_best_view/space_sampler/SpaceSampler.hpp"
#include "next_best_view/rating/RatingModule.hpp"
#include "next_best_view/rating/impl/DefaultScoreContainer.hpp"
#include "next_best_view/crop_box/CropBoxWrapper.hpp"
#include "pbd_msgs/PbdAttributedPointCloud.h"
#include "pbd_msgs/PbdAttributedPoint.h"
#include "next_best_view/helper/MapHelper.hpp"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include "tf/transform_datatypes.h"
#include <std_msgs/ColorRGBA.h>
#include <voxel_grid/voxel_grid.h>

namespace next_best_view
{

class VisualizationHelper
{

private:

    ros::Publisher mIterationMarkerArrayPublisher;
    ros::Publisher mFrustumMarkerArrayPublisher;
    ros::Publisher mObjectMeshMarkerPublisher;
    ros::Publisher mPointObjectNormalPublisher;
    ros::Publisher mFrustumObjectMeshMarkerPublisher;
    ros::Publisher mFrustumObjectNormalPublisher;
    ros::Publisher mCropBoxMarkerPublisher;
    ros::Publisher mWorldMeshesPublisher;
    ros::Publisher mWorldTriangleListPublisher;
    ros::Publisher mVoxelGridPublisher;
    ros::Publisher mRaytracingPublisher;

    ros::NodeHandle mNodeHandle;

    visualization_msgs::MarkerArray::Ptr mIterationMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mNewFrustumMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mOldFrustumMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mObjectMeshMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mObjectNormalsMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mFrustumObjectMeshMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mFrustumObjectNormalsMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mCropBoxMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mVoxelGridMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mRaytracingMarkerArrayPtr;

    MapHelperPtr mMapHelperPtr;
    DebugHelperPtr mDebugHelperPtr;

    int m_i;
    float m_j;
    int mIterationStep;
    bool mBoolClearBetweenIterations;

public:

    VisualizationHelper(MapHelperPtr mapHelperPtr):mMapHelperPtr(mapHelperPtr),m_i(0),m_j(0),mIterationStep(0) {
        mDebugHelperPtr = DebugHelper::getInstance();

        std::string iterationVisualization;
        std::string frustumVisualization;
        std::string objectsVisualization;
        std::string objectNormalsVisualization;
        std::string frustumObjectsVisualization;
        std::string frustumObjectNormalsVisualization;
        std::string cropBoxVisualization;
        std::string pointCloudVisualization;
        std::string worldMeshesVisualization;
        std::string worldTriangleListVisualization;
        std::string voxelGridVisualization;
        std::string raytracingVisualization;

        mNodeHandle.getParam("/nbv/iterationVisualization", iterationVisualization);
        mNodeHandle.getParam("/nbv/frustumVisualization", frustumVisualization);
        mNodeHandle.getParam("/nbv/objectsVisualization", objectsVisualization);
        mNodeHandle.getParam("/nbv/objectNormalsVisualization", objectNormalsVisualization);
        mNodeHandle.getParam("/nbv/frustumObjectsVisualization", frustumObjectsVisualization);
        mNodeHandle.getParam("/nbv/frustumObjectNormalsVisualization", frustumObjectNormalsVisualization);
        mNodeHandle.getParam("/nbv/cropBoxVisualization", cropBoxVisualization);
        mNodeHandle.getParam("/nbv/pointCloudVisualization", pointCloudVisualization);
        mNodeHandle.getParam("/nbv/worldMeshesVisualization", worldMeshesVisualization);
        mNodeHandle.getParam("/nbv/worldTriangleListVisualization", worldTriangleListVisualization);
        mNodeHandle.getParam("/nbv/voxelGridVisualization", voxelGridVisualization);
        mNodeHandle.getParam("/nbv/raytracingVisualization", raytracingVisualization);

        // initialize publishers
        mIterationMarkerArrayPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>(iterationVisualization, 1000);
        mFrustumMarkerArrayPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>(frustumVisualization, 1000);
        mObjectMeshMarkerPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>(objectsVisualization, 100, false);
        mPointObjectNormalPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>(objectNormalsVisualization, 100, false);
        mFrustumObjectMeshMarkerPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>(frustumObjectsVisualization, 100, false);
        mFrustumObjectNormalPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>(frustumObjectNormalsVisualization, 100, false);
        mCropBoxMarkerPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>(cropBoxVisualization, 100, false);
        mWorldMeshesPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>(worldMeshesVisualization, 100, false);
        mWorldTriangleListPublisher = mNodeHandle.advertise<visualization_msgs::Marker>(worldTriangleListVisualization, 100, false);
        mVoxelGridPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>(voxelGridVisualization, 100, false);
        mRaytracingPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>(raytracingVisualization, 100, false);

        if (!mIterationMarkerArrayPublisher) {
            ROS_ERROR("mIterationMarkerArrayPublisher is invalid.");
            throw "Publisher invalid";
        }
        if (!mFrustumMarkerArrayPublisher) {
            ROS_ERROR("mFrustumMarkerArrayPublisher is invalid.");
            throw "Publisher invalid";
        }
        if (!mObjectMeshMarkerPublisher) {
            ROS_ERROR("mObjectMeshMarkerPublisher is invalid.");
            throw "Publisher invalid";
        }
        if (!mPointObjectNormalPublisher) {
            ROS_ERROR("mPointObjectNormalPublisher is invalid.");
            throw "Publisher invalid";;
        }
        if (!mFrustumObjectMeshMarkerPublisher) {
            ROS_ERROR("mFrustumObjectMeshMarkerPublisher is invalid.");
            throw "Publisher invalid";;
        }

        // initalize data
        mNodeHandle.getParam("/nbv/boolClearBetweenIterations", mBoolClearBetweenIterations);

        // initialize marker arrays
        visualization_msgs::MarkerArray* iterationMarkerArray = new visualization_msgs::MarkerArray();
        visualization_msgs::MarkerArray* newFrustumMarkerArray = new visualization_msgs::MarkerArray();
        visualization_msgs::MarkerArray* oldFrustumMarkerArray = new visualization_msgs::MarkerArray();
        visualization_msgs::MarkerArray* objectMeshMarkerArray = new visualization_msgs::MarkerArray();
        visualization_msgs::MarkerArray* objectNormalsMarkerArray = new visualization_msgs::MarkerArray();
        visualization_msgs::MarkerArray* frustumObjectMeshMarkerArray = new visualization_msgs::MarkerArray();
        visualization_msgs::MarkerArray* frustumObjectNormalsMarkerArray = new visualization_msgs::MarkerArray();
        visualization_msgs::MarkerArray* mCropBoxMarkerArray = new visualization_msgs::MarkerArray();
        visualization_msgs::MarkerArray* voxelGridMarkerArray = new visualization_msgs::MarkerArray();
        visualization_msgs::MarkerArray* raytracingMarkerArray = new visualization_msgs::MarkerArray();

        mIterationMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>(*iterationMarkerArray);
        mNewFrustumMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>(*newFrustumMarkerArray);
        mOldFrustumMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>(*oldFrustumMarkerArray);
        mObjectMeshMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>(*objectMeshMarkerArray);
        mObjectNormalsMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>(*objectNormalsMarkerArray);
        mFrustumObjectMeshMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>(*frustumObjectMeshMarkerArray);
        mFrustumObjectNormalsMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>(*frustumObjectNormalsMarkerArray);
        mCropBoxMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>(*mCropBoxMarkerArray);
        mVoxelGridMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>(*voxelGridMarkerArray);
        mRaytracingMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>(*raytracingMarkerArray);
    }

    bool getBoolClearBetweenIterations() {
        return mBoolClearBetweenIterations;
    }

    void triggerIterationVisualization(int iterationStep, const SimpleQuaternionCollectionPtr &sampledOrientationsPtr,
                                            ViewportPoint currentBestViewport,
                                            IndicesPtr feasibleIndices,SamplePointCloudPtr pointcloud,
                                            SpaceSamplerPtr spaceSamplerPtr) {

        mDebugHelperPtr->writeNoticeably("STARTING ITERATION VISUALIZATION", DebugHelper::VISUALIZATION);

        if(!sampledOrientationsPtr){
            ROS_ERROR("triggerIterationVisualizations call with pointer sampledOrientationsPtr being null.");
            mDebugHelperPtr->writeNoticeably("ENDING ITERATION VISUALIZATION", DebugHelper::VISUALIZATION);
            return;
        }
        if(!feasibleIndices){
            ROS_ERROR("triggerIterationVisualizations call with pointer feasibleIndices being null.");
            mDebugHelperPtr->writeNoticeably("ENDING ITERATION VISUALIZATION", DebugHelper::VISUALIZATION);
            return;
        }
        if(!pointcloud){
            ROS_ERROR("triggerIterationVisualizations call with pointer pointcloud being null.");
            mDebugHelperPtr->writeNoticeably("ENDING ITERATION VISUALIZATION", DebugHelper::VISUALIZATION);
            return;
        }
        if(!spaceSamplerPtr){
            ROS_ERROR("triggerIterationVisualizations call with pointer spaceSamplerPtr being null.");
            mDebugHelperPtr->writeNoticeably("ENDING ITERATION VISUALIZATION", DebugHelper::VISUALIZATION);
            return;
        }
        if(!mIterationMarkerArrayPtr){
            ROS_ERROR("triggerIterationVisualizations call with pointer mIterationMarkerArrayPtr being null.");
            mDebugHelperPtr->writeNoticeably("ENDING ITERATION VISUALIZATION", DebugHelper::VISUALIZATION);
            return;
        }

        if (iterationStep == 0 && mBoolClearBetweenIterations == true) {
            // clear iteration visualization
            mDebugHelperPtr->write("Deleting last iteration visualization", DebugHelper::VISUALIZATION);
            this->deleteMarkerArray(mIterationMarkerArrayPtr, mIterationMarkerArrayPublisher);
            m_i = 0;
        }

        std::string s = boost::lexical_cast<std::string>(iterationStep);
        this->mIterationStep=iterationStep;
        m_j = iterationStep*0.25;
        m_j = std::min(1.0f, m_j); //Prevent overflow

        triggerSpaceSampling(feasibleIndices, pointcloud,s);
        triggerGrid(spaceSamplerPtr, s);
        triggerCameraVis(s, sampledOrientationsPtr, currentBestViewport);

        mDebugHelperPtr->write("Publishing markers", DebugHelper::VISUALIZATION);
        mIterationMarkerArrayPublisher.publish(mIterationMarkerArrayPtr);

        mDebugHelperPtr->writeNoticeably("ENDING ITERATION VISUALIZATION", DebugHelper::VISUALIZATION);
    }

    /*!
     * \brief visualizes the frustum of the last camera that was given and the frustum of the new given camera.
     * \param newCamera the new camera
     */
    void triggerFrustumsVisualization(CameraModelFilterPtr newCamera) {
        this->triggerOldFrustumVisualization();
        this->triggerNewFrustumVisualization(newCamera);
    }

    void triggerNewFrustumVisualization(CameraModelFilterPtr newCamera) {

        mDebugHelperPtr->writeNoticeably("STARTING NEW FRUSTUM VISUALIZATION", DebugHelper::VISUALIZATION);

        if(!newCamera){
            ROS_ERROR("triggerNewFrustumVisualization call with pointer newCamera being null.");
            mDebugHelperPtr->writeNoticeably("ENDING NEW FRUSTUM VISUALIZATION", DebugHelper::VISUALIZATION);
            return;
        }
        if(!mNewFrustumMarkerArrayPtr){
            ROS_ERROR("triggerNewFrustumVisualization call with pointer mNewFrustumMarkerArrayPtr being null.");
            mDebugHelperPtr->writeNoticeably("ENDING NEW FRUSTUM VISUALIZATION", DebugHelper::VISUALIZATION);
            return;
        }

        mDebugHelperPtr->write("Deleting last frustum visualization", DebugHelper::VISUALIZATION);
        this->deleteMarkerArray(mNewFrustumMarkerArrayPtr, mFrustumMarkerArrayPublisher);

        uint32_t sequence = 0;
        mNewFrustumMarkerArrayPtr = newCamera->getVisualizationMarkerArray(sequence, 0.0);

        mDebugHelperPtr->write(std::stringstream() << "Frustum Pivot Point : "
                                        << newCamera->getPivotPointPosition()[0]
                                        << " , " <<  newCamera->getPivotPointPosition()[1]
                                        << " , " << newCamera->getPivotPointPosition()[2],
                                    DebugHelper::VISUALIZATION);

        std::string ns = "new_nbv_frustum";

        for (unsigned int i = 0; i < mNewFrustumMarkerArrayPtr->markers.size(); i++)
        {
            mNewFrustumMarkerArrayPtr->markers.at(i).color.r = 0;
            mNewFrustumMarkerArrayPtr->markers.at(i).color.g = 1;
            mNewFrustumMarkerArrayPtr->markers.at(i).color.b = 1;
            mNewFrustumMarkerArrayPtr->markers.at(i).ns = ns;
        }

        mDebugHelperPtr->write("Publishing markers", DebugHelper::VISUALIZATION);
        mFrustumMarkerArrayPublisher.publish(*mNewFrustumMarkerArrayPtr);

        mDebugHelperPtr->writeNoticeably("ENDING NEW FRUSTUM VISUALIZATION", DebugHelper::VISUALIZATION);
    }

    void triggerOldFrustumVisualization(CameraModelFilterPtr camera = NULL) {
        mDebugHelperPtr->writeNoticeably("STARTING OLD FRUSTUM VISUALIZATION", DebugHelper::VISUALIZATION);

        if(!mOldFrustumMarkerArrayPtr){
            ROS_ERROR("triggerOldFrustumVisualization call with pointer mOldFrustumMarkerArrayPtr being null.");
            mDebugHelperPtr->writeNoticeably("ENDING OLD FRUSTUM VISUALIZATION", DebugHelper::VISUALIZATION);
            return;
        }

        mDebugHelperPtr->write("Deleting last frustum visualization", DebugHelper::VISUALIZATION);
        this->deleteMarkerArray(mOldFrustumMarkerArrayPtr, mFrustumMarkerArrayPublisher);

        std::string ns = "old_nbv_frustum";

        if (camera) {
            uint32_t sequence = 0;
            mOldFrustumMarkerArrayPtr = camera->getVisualizationMarkerArray(sequence, 0.0);
            for (unsigned int i = 0; i < mOldFrustumMarkerArrayPtr->markers.size(); i++)
            {
                mOldFrustumMarkerArrayPtr->markers.at(i).color.r = 1;
                mOldFrustumMarkerArrayPtr->markers.at(i).color.g = 0;
                mOldFrustumMarkerArrayPtr->markers.at(i).color.b = 1;
                mOldFrustumMarkerArrayPtr->markers.at(i).lifetime = ros::Duration(4.0);
                mOldFrustumMarkerArrayPtr->markers.at(i).ns = ns;
            }
        }
        else {

            if(!mNewFrustumMarkerArrayPtr){
                ROS_ERROR("triggerOldFrustumVisualization call with pointer mNewFrustumMarkerArrayPtr being null.");
                mDebugHelperPtr->writeNoticeably("ENDING OLD FRUSTUM VISUALIZATION", DebugHelper::VISUALIZATION);
                return;
            }

            // use old data in mNewFrustumMarkerArrayPtr if no camera is given
            if (mNewFrustumMarkerArrayPtr->markers.size() != 0)
            {
                mDebugHelperPtr->write("Copying old frustum marker array", DebugHelper::VISUALIZATION);
                std::copy(mNewFrustumMarkerArrayPtr->markers.begin(), mNewFrustumMarkerArrayPtr->markers.end(),
                          back_inserter(mOldFrustumMarkerArrayPtr->markers));
                mDebugHelperPtr->write("Old frustum marker array copied.", DebugHelper::VISUALIZATION);

                for (unsigned int i = 0; i < mOldFrustumMarkerArrayPtr->markers.size(); i++)
                {
                    mOldFrustumMarkerArrayPtr->markers.at(i).lifetime = ros::Duration(4.0);
                    mOldFrustumMarkerArrayPtr->markers.at(i).color.r = 1;
                    mOldFrustumMarkerArrayPtr->markers.at(i).color.g = 0;
                    mOldFrustumMarkerArrayPtr->markers.at(i).color.b = 1;
                    mOldFrustumMarkerArrayPtr->markers.at(i).ns = ns;
                }
            }
        }

        if (mOldFrustumMarkerArrayPtr->markers.size() != 0) {
            mDebugHelperPtr->write("Publishing markers", DebugHelper::VISUALIZATION);
            mFrustumMarkerArrayPublisher.publish(mOldFrustumMarkerArrayPtr);
        }
        mDebugHelperPtr->writeNoticeably("ENDING OLD FRUSTUM VISUALIZATION", DebugHelper::VISUALIZATION);
    }

    void clearFrustumVisualization()
    {

        if(!mNewFrustumMarkerArrayPtr){
            ROS_ERROR("clearFrustumVisualization call with pointer mNewFrustumMarkerArrayPtr being null.");
            return;
        }

        if (mNewFrustumMarkerArrayPtr->markers.size() == 0) {
            return;
        }

        mDebugHelperPtr->write("Deleting last frustum visualization", DebugHelper::VISUALIZATION);

        this->deleteMarkerArray(mNewFrustumMarkerArrayPtr, mFrustumMarkerArrayPublisher);
        this->deleteMarkerArray(mOldFrustumMarkerArrayPtr, mFrustumMarkerArrayPublisher);
    }

    void triggerObjectPointCloudVisualization(ObjectPointCloud& objectPointCloud, std::map<std::string, std::string>& typeToMeshResource) {
        mDebugHelperPtr->writeNoticeably("STARTING OBJECT POINT CLOUD VISUALIZATION", DebugHelper::VISUALIZATION);

        if(!mObjectMeshMarkerArrayPtr){
            ROS_ERROR("triggerObjectPointCloudVisualization call with pointer mObjectMeshMarkerArrayPtr being null.");
            mDebugHelperPtr->writeNoticeably("ENDING OBJECT POINT CLOUD VISUALIZATION", DebugHelper::VISUALIZATION);
            return;
        }
        if(!mObjectNormalsMarkerArrayPtr){
            ROS_ERROR("triggerObjectPointCloudVisualization call with pointer mObjectNormalsMarkerArrayPtr being null.");
            mDebugHelperPtr->writeNoticeably("ENDING OBJECT POINT CLOUD VISUALIZATION", DebugHelper::VISUALIZATION);
            return;
        }

        visualizePointCloud(objectPointCloud, typeToMeshResource,
                                mObjectMeshMarkerArrayPtr, mObjectMeshMarkerPublisher,
                                mObjectNormalsMarkerArrayPtr, mPointObjectNormalPublisher);

        mDebugHelperPtr->writeNoticeably("ENDING OBJECT POINT CLOUD VISUALIZATION", DebugHelper::VISUALIZATION);
    }

    void triggerFrustumObjectPointCloudVisualization(ObjectPointCloud& frustumObjectPointCloud, std::map<std::string, std::string>& typeToMeshResource) {
        mDebugHelperPtr->writeNoticeably("STARTING FRUSTUM OBJECT POINT CLOUD VISUALIZATION", DebugHelper::VISUALIZATION);

        if(!mFrustumObjectMeshMarkerArrayPtr){
            ROS_ERROR("triggerFrustumObjectPointCloudVisualization call with pointer mFrustumObjectMeshMarkerArrayPtr being null.");
            mDebugHelperPtr->writeNoticeably("ENDING FRUSTUM OBJECT POINT CLOUD VISUALIZATION", DebugHelper::VISUALIZATION);
            return;
        }

        std_msgs::ColorRGBA::Ptr colorFrustumMeshMarkerPtr(new std_msgs::ColorRGBA(this->createColorRGBA(0, 0, 1, 0.8)));

        visualizePointCloud(frustumObjectPointCloud, typeToMeshResource,
                                mFrustumObjectMeshMarkerArrayPtr, mFrustumObjectMeshMarkerPublisher,
                                mFrustumObjectNormalsMarkerArrayPtr, mFrustumObjectNormalPublisher,
                                colorFrustumMeshMarkerPtr);

        mDebugHelperPtr->writeNoticeably("ENDING FRUSTUM OBJECT POINT CLOUD VISUALIZATION", DebugHelper::VISUALIZATION);
    }

    void triggerCropBoxVisualization(const boost::shared_ptr<std::vector<CropBoxWrapperPtr>> cropBoxWrapperPtrList)
    {
        if(!mCropBoxMarkerArrayPtr)
        {
            ROS_ERROR_STREAM("triggerCropBoxVisualization::mCropBoxMarkerArrayPtr is empty.");
            return;
        }

        std::vector<double> CropBoxMarkerRGBA;
        mNodeHandle.getParam("/nbv/CropBoxMarker_RGBA", CropBoxMarkerRGBA);
        SimpleVector4 color = TypeHelper::getSimpleVector4(CropBoxMarkerRGBA);

        int id = 0;
        for(std::vector<CropBoxWrapperPtr>::const_iterator it = cropBoxWrapperPtrList->begin(); it != cropBoxWrapperPtrList->end(); ++it)
        {
            CropBoxWrapperPtr cropBoxWrapperPtr = *it;
            CropBoxPtr cropBoxPtr = cropBoxWrapperPtr->getCropBox();
            Eigen::Vector4f ptMin,ptMax;
            ptMin = cropBoxPtr->getMin();
            ptMax = cropBoxPtr->getMax();
            Eigen::Vector3f rotation, translation;
            rotation = cropBoxPtr->getRotation();
            translation = cropBoxPtr->getTranslation();

            Eigen::Matrix3f rotationMatrix;
            rotationMatrix = Eigen::AngleAxisf(rotation[0], Eigen::Vector3f::UnitX())
                    * Eigen::AngleAxisf(rotation[1], Eigen::Vector3f::UnitY())
                    * Eigen::AngleAxisf(rotation[2], Eigen::Vector3f::UnitZ());


            SimpleVector3 position_cb_frame;
            position_cb_frame[0] = (ptMax[0] + ptMin[0])/2;
            position_cb_frame[1] = (ptMax[1] + ptMin[1])/2;
            position_cb_frame[2] = (ptMax[2] + ptMin[2])/2;

            SimpleVector3 position_map_frame;
            position_map_frame = rotationMatrix * position_cb_frame + translation;

            SimpleQuaternion orientation(rotationMatrix);

            SimpleVector3 scale;
            scale[0] = std::abs(ptMax[0] - ptMin[0]);
            scale[1] = std::abs(ptMax[1] - ptMin[1]);
            scale[2] = std::abs(ptMax[2] - ptMin[2]);

            std::stringstream ns;
            ns << "cropbox_ns" << id;

            mCropBoxMarkerArrayPtr->markers.push_back(MarkerHelper::getCubeMarker(id,
                                                                                  position_map_frame, orientation, scale,  color, ns.str()));
            id++;
        }
        mCropBoxMarkerPublisher.publish(*mCropBoxMarkerArrayPtr);
    }

    void triggerWorldMeshesVisualization(std::vector<std::string> meshResources, std::vector<SimpleVector3> positions, std::vector<SimpleQuaternion> orientations,
                                                                                                                                    std::vector<SimpleVector3> scales)
    {
        visualization_msgs::MarkerArray worldMeshesMarker;

        for (unsigned int i = 0; i < meshResources.size(); i++)
        {
            visualization_msgs::Marker worldMeshMarker = MarkerHelper::getMeshMarker(i, meshResources[i], positions[i], orientations[i], scales[i], "WorldMesh");
            worldMeshesMarker.markers.push_back(worldMeshMarker);
        }

        mWorldMeshesPublisher.publish(worldMeshesMarker);
    }

    void triggerWorldTrianglesVisualization(std::vector<std::vector<SimpleVector3>> faces)
    {
        mDebugHelperPtr->writeNoticeably("STARTING WORLD TRIANGLES VISUALIZATION", DebugHelper::VISUALIZATION);
        std::vector<SimpleVector3> vertices;
        std::vector<SimpleVector4> colors;

        for (unsigned int i = 0; i < faces.size(); i++)
            for (unsigned int j = 0; j < 3; j++)
            {
                SimpleVector3 vertex = faces[i][j];
                vertices.push_back(vertex);
                colors.push_back(SimpleVector4(0,1,0,1.0));
            }

        mDebugHelperPtr->write(std::stringstream() << "Vertices size: " << vertices.size(), DebugHelper::VISUALIZATION);

        visualization_msgs::Marker facesMarker = MarkerHelper::getTriangleListMarker(0, vertices, colors, "WorldTriangles");
        mWorldTriangleListPublisher.publish(facesMarker);

        mDebugHelperPtr->writeNoticeably("ENDING WORLD TRIANGLES VISUALIZATION", DebugHelper::VISUALIZATION);
    }

    void triggerVoxelGridVisualization(VoxelGridHelperPtr voxelGridHelperPtr, double worldVoxelSize)
    {
        mDebugHelperPtr->writeNoticeably("STARTING VOXEL GRID VISUALIZATION", DebugHelper::VISUALIZATION);

        GridVector3 voxelGridSize = voxelGridHelperPtr->getVoxelGridSize();

        mDebugHelperPtr->write(std::stringstream() << "Voxel grid size: " << voxelGridSize[0] << "x" << voxelGridSize[1] << "x" << voxelGridSize[2],
                                                                                                                            DebugHelper::VISUALIZATION);

        deleteMarkerArray(mVoxelGridMarkerArrayPtr, mVoxelGridPublisher);

        // get paramters
        std::vector<double> VoxelGridMarkedMarkerRGBA;
        mNodeHandle.getParam("/nbv/VoxelGridMarkedMarker_RGBA", VoxelGridMarkedMarkerRGBA);
        SimpleVector4 colorMarked = TypeHelper::getSimpleVector4(VoxelGridMarkedMarkerRGBA);

        std::vector<double> VoxelGridNotMarkedMarkerRGBA;
        mNodeHandle.getParam("/nbv/VoxelGridNotMarkedMarker_RGBA", VoxelGridNotMarkedMarkerRGBA);
        SimpleVector4 colorNotMarked = TypeHelper::getSimpleVector4(VoxelGridNotMarkedMarkerRGBA);

        bool showVoxelGridNotMarkedMarker;
        mNodeHandle.getParam("/nbv/showVoxelGridNotMarkedMarker", showVoxelGridNotMarkedMarker);

        int id = 0;
        int count = 0;

        for (int i = 0; i < voxelGridSize[0]; i++)
            for (int j = 0; j < voxelGridSize[1]; j++)
                for (int k = 0; k < voxelGridSize[2]; k++)
                {
                    GridVector3 voxelPos(i, j, k);

                    SimpleVector4 color = colorNotMarked;

                    if (voxelGridHelperPtr->isMarked(voxelPos))
                        color = colorMarked;
                    else if (!showVoxelGridNotMarkedMarker)
                    {
                        id++;
                        continue;
                    }

                    std::string ns = "VoxelGrid";
                    visualization_msgs::Marker voxelMarker = getVoxelMarker(voxelPos, worldVoxelSize, color, id, ns);
                    mVoxelGridMarkerArrayPtr->markers.push_back(voxelMarker);
                    id++;
                    count++;
                }

        mDebugHelperPtr->write(std::stringstream() << "Publishing " << count << " voxels.", DebugHelper::VISUALIZATION);

        mVoxelGridPublisher.publish(mVoxelGridMarkerArrayPtr);

        mDebugHelperPtr->writeNoticeably("ENDING VOXEL GRID VISUALIZATION", DebugHelper::VISUALIZATION);
    }


    void triggerRaytracingVisualization(SimpleVector3 rayStartPos, SimpleVector3 rayEndPos, std::vector<GridVector3> traversedVoxels,
                                                                                                    bool occluded, double worldVoxelSize)
    {
        mDebugHelperPtr->writeNoticeably("STARTING RAYTRACING VISUALIZATION", DebugHelper::VISUALIZATION);

        deleteMarkerArray(mRaytracingMarkerArrayPtr, mRaytracingPublisher);

        // get paramters
        std::vector<double> RaytracingRayMarkerRGBA;
        mNodeHandle.getParam("/nbv/RaytracingRayMarker_RGBA", RaytracingRayMarkerRGBA);
        SimpleVector4 rayColor = TypeHelper::getSimpleVector4(RaytracingRayMarkerRGBA);

        std::vector<double> RaytracingTraversedNotMarkedVoxelMarkerRGBA;
        mNodeHandle.getParam("/nbv/RaytracingTraversedNotMarkedVoxelMarker_RGBA", RaytracingTraversedNotMarkedVoxelMarkerRGBA);
        SimpleVector4 voxelNotMarkedColor = TypeHelper::getSimpleVector4(RaytracingTraversedNotMarkedVoxelMarkerRGBA);

        std::vector<double> RaytracingTraversedMarkedVoxelMarkerRGBA;
        mNodeHandle.getParam("/nbv/RaytracingTraversedMarkedVoxelMarker_RGBA", RaytracingTraversedMarkedVoxelMarkerRGBA);
        SimpleVector4 voxelMarkedColor = TypeHelper::getSimpleVector4(RaytracingTraversedMarkedVoxelMarkerRGBA);

        // visualize ray
        visualization_msgs::Marker rayMarker = MarkerHelper::getArrowMarker(0, rayStartPos, rayEndPos, SimpleVector3(1,1,1), rayColor, "Ray");

        mRaytracingMarkerArrayPtr->markers.push_back(rayMarker);

        // visualize traversed voxels
        for (int i = 0; i < traversedVoxels.size(); i++)
        {
            SimpleVector4 color = voxelNotMarkedColor;
            if (occluded && i == traversedVoxels.size()-1)
                color = voxelMarkedColor;

            std::string ns = "VoxelGrid";
            visualization_msgs::Marker voxelMarker = getVoxelMarker(traversedVoxels[i], worldVoxelSize, color, i, ns);
            mRaytracingMarkerArrayPtr->markers.push_back(voxelMarker);
        }

        mDebugHelperPtr->write(std::stringstream() << "Publishing " << traversedVoxels.size() << " voxels.", DebugHelper::VISUALIZATION);

        mRaytracingPublisher.publish(mRaytracingMarkerArrayPtr);

        mDebugHelperPtr->writeNoticeably("ENDING RAYTRACING VISUALIZATION", DebugHelper::VISUALIZATION);
    }

    /* only working because the shape-based recognizer sets the observedId with the object color */
    static std_msgs::ColorRGBA getMeshColor(std::string observedId)
    {
        std_msgs::ColorRGBA retColor = VisualizationHelper::createColorRGBA(0.0, 0.0, 0.0, 0.0);

        if ( ( observedId.length() == 12 ) && ( observedId.find_first_not_of("0123456789") == std::string::npos ) )
        {
            float rgba[4];
            bool isColor = true;
            try
            {
                for (int i = 0; i <= 3; i++)
                {
                    std::string temp;

                    temp = observedId.substr( (i * 3), 3 );
                    rgba[i] = std::stof(temp) / 100.0;
                }
            }
            catch (std::invalid_argument& ia)
            {
                DebugHelper::getInstance()->write(ia.what(), DebugHelper::VISUALIZATION);
                isColor = false;
            }

            if(isColor)
            {
                retColor = VisualizationHelper::createColorRGBA(rgba[0], rgba[1], rgba[2], rgba[3]);
            }
        }

        return retColor;
    }

private:

    void triggerCameraVis(std::string s, const SimpleQuaternionCollectionPtr &sampledOrientationsPtr,
                          ViewportPoint currentBestViewport) {

        if(!sampledOrientationsPtr){
            ROS_ERROR("triggerCameraVis call with pointer sampledOrientationsPtr being null.");
            return;
        }
        if(!mIterationMarkerArrayPtr){
            ROS_ERROR("triggerCameraVis call with pointer mIterationMarkerArrayPtr being null.");
            return;
        }

        // get parameters
        std::vector<double> ViewPortMarkerScales;
        std::vector<double> ViewPortMarkerRGBA;
        std::vector<double> ViewPortDirectionsRGBA;
        std::vector<double> ViewPortDirectionsScales;
        std::vector<double> ColumnPositionMarkerRGBA;
        double ViewPortMarkerHeightFactor;
        double ViewPortMarkerShrinkFactor;
        double ColumnPositionMarkerWidth;

        mNodeHandle.getParam("/nbv/ViewPortMarker_Scales", ViewPortMarkerScales);
        mNodeHandle.getParam("/nbv/ViewPortMarker_HeightFactor", ViewPortMarkerHeightFactor);
        mNodeHandle.getParam("/nbv/ViewPortMarker_ShrinkFactor", ViewPortMarkerShrinkFactor);
        mNodeHandle.getParam("/nbv/ViewPortMarker_RGBA", ViewPortMarkerRGBA);
        mNodeHandle.getParam("/nbv/ViewPortDirections_RGBA", ViewPortDirectionsRGBA);
        mNodeHandle.getParam("/nbv/ViewPortDirections_Scales", ViewPortDirectionsScales);
        mNodeHandle.getParam("/nbv/ColumnPositionMarker_Width", ColumnPositionMarkerWidth);
        mNodeHandle.getParam("/nbv/ColumnPositionMarker_RGBA", ColumnPositionMarkerRGBA);

        SimpleVector3 position = currentBestViewport.getPosition();

        SimpleVector3 scaleViewPortDirection = TypeHelper::getSimpleVector3(ViewPortDirectionsScales);
        SimpleVector4 colorViewPortDirection = TypeHelper::getSimpleVector4(ViewPortDirectionsRGBA);
        colorViewPortDirection[0] -= m_j;
        colorViewPortDirection[1] += m_j;

        SimpleVector3 scaleViewPort = TypeHelper::getSimpleVector3(ViewPortMarkerScales);
        SimpleVector4 colorViewPort = TypeHelper::getSimpleVector4(ViewPortMarkerRGBA);
        colorViewPort[0] -= m_j;
        colorViewPort[1] += m_j;

        BOOST_FOREACH(SimpleQuaternion q, *sampledOrientationsPtr)
        {
            // get the data for the markers
            SimpleVector3 visualAxis = MathHelper::getVisualAxis(q);
            visualAxis[0] = visualAxis[0]/ViewPortMarkerShrinkFactor + position[0];
            visualAxis[1] = visualAxis[1]/ViewPortMarkerShrinkFactor + position[1];
            visualAxis[2] = visualAxis[2]/ViewPortMarkerShrinkFactor + position[2]
                    + mIterationStep * ViewPortMarkerHeightFactor;

            // get viewport direction marker
            m_i++;
            std::string ns = "ViewPortDirections" + s;

            visualization_msgs::Marker ViewPortDirectionsMarker = MarkerHelper::getArrowMarker(m_i, visualAxis, q, scaleViewPortDirection, colorViewPortDirection, ns);
            mIterationMarkerArrayPtr->markers.push_back(ViewPortDirectionsMarker);

            // get viewport marker
            m_i++;
            ns = "visualAxis" + s;

            visualization_msgs::Marker ViewPortMarker = MarkerHelper::getCubeMarker(m_i, visualAxis, q, scaleViewPort, colorViewPort, ns);
            mIterationMarkerArrayPtr->markers.push_back(ViewPortMarker);
        }

        // get unit sphere marker
        SimpleVector3 spherePosition(position);
        spherePosition[2] += mIterationStep * ViewPortMarkerHeightFactor;

        SimpleVector3 scale(2.0/ViewPortMarkerShrinkFactor, 2.0/ViewPortMarkerShrinkFactor, 2.0/ViewPortMarkerShrinkFactor);

        SimpleVector4 color = TypeHelper::getSimpleVector4(ViewPortMarkerRGBA);
        color[0] -= m_j;
        color[1] += m_j;
        color[3] /= 2.0;

        std::string ns = "visualAxisSphere" + s;

        m_i++;

        visualization_msgs::Marker ViewPortSphereMarker = MarkerHelper::getSphereMarker(m_i, spherePosition, scale, color, ns);
        mIterationMarkerArrayPtr->markers.push_back(ViewPortSphereMarker);

        // get column position marker
        SimpleVector3 point1(position);
        SimpleVector3 point2(position);
        point1[2] = 0;
        point2[2] += mIterationStep * ViewPortMarkerHeightFactor;
        std::vector<SimpleVector3> points;
        points.push_back(point1);
        points.push_back(point2);

        color = TypeHelper::getSimpleVector4(ColumnPositionMarkerRGBA);

        ns = "LineVizu" + s;

        m_i++;

        visualization_msgs::Marker ColumnPositionMarker = MarkerHelper::getLineListMarker(m_i, points, ColumnPositionMarkerWidth,
                                                                                          color, ns);
        mIterationMarkerArrayPtr->markers.push_back(ColumnPositionMarker);

        // get nbv camera direction
        scale = SimpleVector3(1, ColumnPositionMarkerWidth, ColumnPositionMarkerWidth);

        color = TypeHelper::getSimpleVector4(ViewPortMarkerRGBA);
        color[0] -= m_j;
        color[1] += m_j;

        ns = "ArrowVizu" +s ;

        m_i++;

        visualization_msgs::Marker NextBestViewCameraDirectionMarker = MarkerHelper::getArrowMarker(m_i, position,
                                                                                                    currentBestViewport.getSimpleQuaternion(),
                                                                                                    scale, color, ns);
        mIterationMarkerArrayPtr->markers.push_back(NextBestViewCameraDirectionMarker);
    }

    void triggerSpaceSampling(IndicesPtr feasibleIndices,SamplePointCloudPtr pointcloud, std::string s){

        if(!feasibleIndices){
            ROS_ERROR("triggerSpaceSampling call with pointer feasibleIndices being null.");
            return;
        }
        if(!pointcloud){
            ROS_ERROR("triggerSpaceSampling call with pointer pointcloud being null.");
            return;
        }
        if(!mIterationMarkerArrayPtr){
            ROS_ERROR("triggerSpaceSampling call with pointer mIterationMarkerArrayPtr being null.");
            return;
        }

        // get parameters
        double SpaceSamplingMarkerScale;
        std::vector<double> SpaceSamplingMarkerRGBA;
        mNodeHandle.getParam("/nbv/SpaceSamplingMarker_Scale", SpaceSamplingMarkerScale);
        mNodeHandle.getParam("/nbv/SpaceSamplingMarker_RGBA", SpaceSamplingMarkerRGBA);

        SimpleVector3 scale(SpaceSamplingMarkerScale, SpaceSamplingMarkerScale, SpaceSamplingMarkerScale);

        SimpleVector4 color = TypeHelper::getSimpleVector4(SpaceSamplingMarkerRGBA);
        color[0] -= m_j;
        color[1] += m_j;

        SamplePointCloud pcl = SamplePointCloud(*pointcloud, *feasibleIndices);

        for(SamplePointCloud::iterator it = pcl.points.begin(); it < pcl.points.end(); it++)
        {
            // get space sampling marker
            gm::Point point = it->getPoint();
            SimpleVector3 position = TypeHelper::getSimpleVector3(point);
            position[2] = 0.1;

            std::string ns = "SamplePoints_NS" + s;

            m_i++;

            visualization_msgs::Marker SpaceSamplingMarker = MarkerHelper::getCylinderMarker(m_i, position, 1, scale, color, ns);
            mIterationMarkerArrayPtr->markers.push_back(SpaceSamplingMarker);
        }

    }

    void triggerGrid(SpaceSamplerPtr spaceSamplerPtr, std::string s){

        if(!spaceSamplerPtr){
            ROS_ERROR("triggerGrid call with pointer spaceSamplerPtr being null.");
            return;
        }
        if(!mIterationMarkerArrayPtr){
            ROS_ERROR("triggerGrid call with pointer mIterationMarkerArrayPtr being null.");
            return;
        }

        // get parameters
        double GridMarkerScaleZ;
        std::vector<double> GridMarkerRGBA;
        mNodeHandle.getParam("/nbv/GridMarker_ScaleZ", GridMarkerScaleZ);
        mNodeHandle.getParam("/nbv/GridMarker_RGBA", GridMarkerRGBA);

        double xwidth = abs(spaceSamplerPtr->getXtop() - spaceSamplerPtr->getXbot());
        double ywidth = abs(spaceSamplerPtr->getYtop() - spaceSamplerPtr->getYbot());
        double xmid = (spaceSamplerPtr->getXtop() + spaceSamplerPtr->getXbot())/2.0;
        double ymid =  (spaceSamplerPtr->getYtop() + spaceSamplerPtr->getYbot())/2.0;

        SimpleVector3 position;
        position[0] = xmid;
        position[1] = ymid;
        position[2] = 0;

        SimpleQuaternion orientation(1,0,0,0);

        SimpleVector3 scale(xwidth, ywidth, GridMarkerScaleZ);
        SimpleVector4 color = TypeHelper::getSimpleVector4(GridMarkerRGBA);

        std::string ns = "Radius" + s;

        m_i++;

        visualization_msgs::Marker GridMarker = MarkerHelper::getCubeMarker(m_i, position, orientation, scale, color, ns);
        mIterationMarkerArrayPtr->markers.push_back(GridMarker);
    }

    visualization_msgs::Marker getVoxelMarker(GridVector3 voxelPos, double worldVoxelSize, SimpleVector4 color, int id, std::string ns) {
        double mapVoxelSize;
        mMapHelperPtr->worldToMapSize(worldVoxelSize, mapVoxelSize);

        SimpleVector3 mapPosition(((double) voxelPos[0] + 0.5) * mapVoxelSize, ((double) voxelPos[1] + 0.5) * mapVoxelSize, ((double) voxelPos[2] + 0.5) * mapVoxelSize);
        SimpleVector3 worldPosition;
        mMapHelperPtr->mapToWorldCoordinates(mapPosition, worldPosition);

        SimpleQuaternion orientation(1,0,0,0);
        SimpleVector3 scale(worldVoxelSize,worldVoxelSize,worldVoxelSize);
        return MarkerHelper::getCubeMarker(id, worldPosition, orientation, scale, color, ns);
    }

    static void visualizePointCloud(ObjectPointCloud& objectPointCloud, std::map<std::string, std::string>& typeToMeshResource,
                                        visualization_msgs::MarkerArray::Ptr objectMarkerArrayPtr, ros::Publisher& objectPublisher,
                                        visualization_msgs::MarkerArray::Ptr objectNormalsMarkerArrayPtr, ros::Publisher& objectNormalsPublisher,
                                        std_msgs::ColorRGBA::Ptr objectColorPtr = NULL) {
        DebugHelperPtr debugHelperPtr = DebugHelper::getInstance();

        debugHelperPtr->write("Deleting old object point cloud visualization", DebugHelper::VISUALIZATION);
        deleteMarkerArray(objectMarkerArrayPtr, objectPublisher);

        unsigned int index = 0;
        std::string ns = "ObjectMeshes";

        for(ObjectPointCloud::iterator it = objectPointCloud.begin(); it < objectPointCloud.end(); it++)
        {
            geometry_msgs::Pose pose = it->getPose();
            std_msgs::ColorRGBA color;

            if (!objectColorPtr) {
                color = it->color;
            }
            else {
                color = *objectColorPtr;
            }

            visualization_msgs::Marker objectMarker = getObjectMarker(pose, it->type, color, typeToMeshResource, index, ns);

            objectMarkerArrayPtr->markers.push_back(objectMarker);

            index++;
        }

        debugHelperPtr->write(std::stringstream() << "Publishing " << objectPointCloud.size() <<" object points",
                    DebugHelper::VISUALIZATION);
        objectPublisher.publish(*objectMarkerArrayPtr);

        debugHelperPtr->write("Deleting old object normals visualization", DebugHelper::VISUALIZATION);
        deleteMarkerArray(objectNormalsMarkerArrayPtr, objectNormalsPublisher);

        index = 0;
        SimpleVector4 color = SimpleVector4(1.0, 1.0, 0.0, 1.0);
        SimpleVector3 scale = SimpleVector3(0.005, 0.01, 0.005);
        ns = "ObjectNormals";

        for(unsigned int i = 0; i < objectPointCloud.points.size(); i++)
        {
            ObjectPoint point = objectPointCloud.points[i];

            for(Indices::iterator it = point.active_normal_vectors->begin(); it < point.active_normal_vectors->end(); ++it)
            {
                SimpleVector3 start = point.getPosition();
                SimpleVector3 end = 0.07 * point.normal_vectors->at(*it);
                end[0] += start[0];
                end[1] += start[1];
                end[2] += start[2];


                visualization_msgs::Marker objectNormalMarker = MarkerHelper::getArrowMarker(index, start, end,
                                                                                             scale, color, ns);
                objectNormalsMarkerArrayPtr->markers.push_back(objectNormalMarker);

                index++;
            }
        }

        debugHelperPtr->write("Publishing object normals", DebugHelper::VISUALIZATION);
        objectNormalsPublisher.publish(*objectNormalsMarkerArrayPtr);
    }
    
    static void deleteMarkerArray(visualization_msgs::MarkerArray::Ptr &array, ros::Publisher &publisher)
    {
        if(!array || !publisher)
            return;

        if (array->markers.size() == 0) {
            return;
        }

        for (unsigned int i = 0; i < array->markers.size(); i++)
        {
            array->markers.at(i).action = visualization_msgs::Marker::DELETE;
        }

        publisher.publish(array);
        array->markers.clear();
    }

    static visualization_msgs::Marker getObjectMarker(geometry_msgs::Pose pose, std::string type, std_msgs::ColorRGBA color,
                                                      std::map<std::string, std::string> typeToMeshResource, int id, std::string ns) {
        visualization_msgs::Marker objectMarker;

        SimpleVector3 position = TypeHelper::getSimpleVector3(pose);
        SimpleQuaternion orientation = TypeHelper::getSimpleQuaternion(pose);

        if (typeToMeshResource.count(type) == 1) {
            std::string meshResource = typeToMeshResource[type];

            // Cut off .iv, append .dae
            boost::filesystem::path meshResourcePath = boost::filesystem::path(meshResource).replace_extension(".dae");
            meshResource = meshResourcePath.string();

            // the model size unit is mm
            SimpleVector3 scale(0.0005, 0.0005, 0.0005);

            objectMarker = MarkerHelper::getMeshMarker(id, meshResource, position, orientation, scale, ns);
            objectMarker.color = color;
        }
        else {
            SimpleVector3 scale(0.01, 0.01, 0.01);
            SimpleVector4 sphereColor = TypeHelper::getSimpleVector4(color);
            objectMarker = MarkerHelper::getSphereMarker(id, position, scale, sphereColor, ns);
        }

        return objectMarker;
    }

    static std_msgs::ColorRGBA createColorRGBA(float red, float green, float blue, float alpha)
    {
        std_msgs::ColorRGBA color;

        color.r = red;
        color.g = green;
        color.b = blue;
        color.a = alpha;

        return color;
    }

};

typedef boost::shared_ptr<VisualizationHelper> VisualizationHelperPtr;

}
