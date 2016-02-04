#pragma once

#include "typedef.hpp"
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/range/algorithm_ext/iota.hpp>
#include <boost/lexical_cast.hpp>

#include "next_best_view/NextBestViewCalculator.hpp"
#include "next_best_view/camera_model_filter/CameraModelFilter.hpp"
#include "next_best_view/helper/MarkerHelper.hpp"
#include "next_best_view/helper/TypeHelper.hpp"
#include "next_best_view/space_sampler/SpaceSampler.hpp"
#include "next_best_view/rating/RatingModule.hpp"
#include "next_best_view/rating/impl/DefaultScoreContainer.hpp"
#include "pbd_msgs/PbdAttributedPointCloud.h"
#include "pbd_msgs/PbdAttributedPoint.h"
#include "next_best_view/helper/MapHelper.hpp"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include "tf/transform_datatypes.h"
#include <std_msgs/ColorRGBA.h>

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
    ros::Publisher mCropBoxMarkerPublisher;

    ros::NodeHandle mNodeHandle;

    visualization_msgs::MarkerArray::Ptr mIterationMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mNewFrustumMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mOldFrustumMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mObjectMeshMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mObjectNormalsMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mFrustumObjectMeshMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mCropBoxMarkerArrayPtr;

    int m_i;
    float m_j;
    int mIterationStep;
    bool mBoolClearBetweenIterations;

public:

    VisualizationHelper():m_i(0),m_j(0),mIterationStep(0) {
        // initialize publishers
        mIterationMarkerArrayPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>( "/nbv/iteration_visualization", 1000);
        mFrustumMarkerArrayPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>("/nbv/frustum_marker_array", 1000);
        mObjectMeshMarkerPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>("/nbv/object_meshes", 100, false);
        mPointObjectNormalPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>("/nbv/object_normals", 100, false);
        mFrustumObjectMeshMarkerPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>("/nbv/frustum_object_meshes", 100, false);
        mCropBoxMarkerPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>("/nbv/crop_box", 100, false);

        // initalize data
        mNodeHandle.getParam("/nbv/boolClearBetweenIterations", mBoolClearBetweenIterations);
        ROS_DEBUG_STREAM("boolClearBetweenIterations: " << mBoolClearBetweenIterations);

        // initialize marker arrays
        visualization_msgs::MarkerArray* iterationMarkerArray = new visualization_msgs::MarkerArray();
        visualization_msgs::MarkerArray* newFrustumMarkerArray = new visualization_msgs::MarkerArray();
        visualization_msgs::MarkerArray* oldFrustumMarkerArray = new visualization_msgs::MarkerArray();
        visualization_msgs::MarkerArray* objectMeshMarkerArray = new visualization_msgs::MarkerArray();
        visualization_msgs::MarkerArray* frustumObjectMeshMarkerArray = new visualization_msgs::MarkerArray();
        visualization_msgs::MarkerArray* objectNormalsMarkerArray = new visualization_msgs::MarkerArray();
        visualization_msgs::MarkerArray* mCropBoxMarkerArray = new visualization_msgs::MarkerArray();


        mIterationMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>(*iterationMarkerArray);
        mNewFrustumMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>(*newFrustumMarkerArray);
        mOldFrustumMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>(*oldFrustumMarkerArray);
        mObjectMeshMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>(*objectMeshMarkerArray);
        mObjectNormalsMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>(*objectNormalsMarkerArray);
        mFrustumObjectMeshMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>(*frustumObjectMeshMarkerArray);
        mCropBoxMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>(*mCropBoxMarkerArray);
    }

    void triggerIterationVisualizations(int iterationStep, SimpleVector3 position, const SimpleQuaternionCollectionPtr
                               &sampledOrientationsPtr, ViewportPoint currentBestViewport,
                               IndicesPtr feasibleIndices,SamplePointCloudPtr pointcloud,
                               SpaceSamplerPtr spaceSamplerPtr) {

        ROS_DEBUG_STREAM("iteration visualization");

        if (iterationStep == 0 && mBoolClearBetweenIterations == true) {
            // clear iteration visualization
            this->deleteMarkerArray(mIterationMarkerArrayPtr, mIterationMarkerArrayPublisher);
            m_i = 0;
        }

        std::string s = boost::lexical_cast<std::string>(iterationStep);
        this->mIterationStep=iterationStep;
        m_j = iterationStep*0.25;
        m_j = std::min(1.0f, m_j); //Prevent overflow

        triggerSpaceSampling(feasibleIndices, pointcloud,s);
        triggerGrid(spaceSamplerPtr, s);
        triggerCameraVis(s, position, sampledOrientationsPtr, currentBestViewport);

        ROS_DEBUG_STREAM("publish markers");

        mIterationMarkerArrayPublisher.publish(mIterationMarkerArrayPtr);
    }

    /*!
     * \brief visualizes the frustum of the last camera that was given and the frustum of the new given camera.
     * \param newCamera the new camera
     * \param numberSearchedObjects the number of searched objects
     */
    void triggerFrustumsVisualization(CameraModelFilterPtr newCamera, int numberSearchedObjects = -1) {
        this->triggerOldFrustumVisualization();
        this->triggerNewFrustumVisualization(newCamera, numberSearchedObjects);
    }

    void triggerNewFrustumVisualization(CameraModelFilterPtr newCamera, int numberSearchedObjects = -1) {
        ROS_DEBUG("Publish new frustum");

        this->deleteMarkerArray(mNewFrustumMarkerArrayPtr, mFrustumMarkerArrayPublisher);

        uint32_t sequence = 0;
        mNewFrustumMarkerArrayPtr = newCamera->getVisualizationMarkerArray(sequence, 0.0);

        ROS_DEBUG_STREAM("Frustum Pivot Point : " << newCamera->getPivotPointPosition()[0] <<
                         " , " <<  newCamera->getPivotPointPosition()[1]
                         << " , " << newCamera->getPivotPointPosition()[2]);

        std::string ns = "new_nbv_frustum";

        for (unsigned int i = 0; i < mNewFrustumMarkerArrayPtr->markers.size(); i++)
        {
                mNewFrustumMarkerArrayPtr->markers.at(i).color.r = 0;
                mNewFrustumMarkerArrayPtr->markers.at(i).color.g = 1;
                mNewFrustumMarkerArrayPtr->markers.at(i).color.b = 1;
                mNewFrustumMarkerArrayPtr->markers.at(i).ns = ns;
        }

        if (numberSearchedObjects != -1) {
            ROS_DEBUG("Adding text");
            ROS_INFO_STREAM("numberSearchedObjects: " <<  numberSearchedObjects);

            geometry_msgs::Pose pose;
            pose.position = TypeHelper::getPointMSG(newCamera->getPivotPointPosition());
            pose.orientation = TypeHelper::getQuaternionMSG(newCamera->getOrientation());

            ns = "new_nbv_frustum_text";
            std::string result = "searched objects: " + boost::lexical_cast<std::string>(numberSearchedObjects);
            visualization_msgs::Marker textMarker = MarkerHelper::getTextMarker(mNewFrustumMarkerArrayPtr->markers.size(), result, pose, ns);
            mNewFrustumMarkerArrayPtr->markers.push_back(textMarker);
        }

        ROS_DEBUG_STREAM("Marker array size:" << mNewFrustumMarkerArrayPtr->markers.size());
        mFrustumMarkerArrayPublisher.publish(*mNewFrustumMarkerArrayPtr);
    }

    void triggerOldFrustumVisualization(CameraModelFilterPtr camera = NULL) {
        ROS_DEBUG("Publish old frustum");

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
            // use old data in mNewFrustumMarkerArrayPtr if no camera is given
            if (mNewFrustumMarkerArrayPtr->markers.size() != 0)
            {
                ROS_DEBUG("Copying old frustum marker array...");
                std::copy(mNewFrustumMarkerArrayPtr->markers.begin(), mNewFrustumMarkerArrayPtr->markers.end(),
                            back_inserter(mOldFrustumMarkerArrayPtr->markers));
                ROS_DEBUG("Old frustum marker array copied.");

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
            mFrustumMarkerArrayPublisher.publish(mOldFrustumMarkerArrayPtr);
        }
    }

    void clearFrustumVisualization()
    {
        ROS_DEBUG("Deleting last frustum visualization");
        if (mNewFrustumMarkerArrayPtr->markers.size() == 0) {
            return;
        }

        this->deleteMarkerArray(mNewFrustumMarkerArrayPtr, mFrustumMarkerArrayPublisher);
        this->deleteMarkerArray(mOldFrustumMarkerArrayPtr, mFrustumMarkerArrayPublisher);
    }

    void triggerObjectPointCloudVisualization(ObjectPointCloud objectPointCloud, std::map<std::string, std::string> typeToMeshResource) {
        ROS_DEBUG("Publishing Point Cloud");

        ROS_DEBUG("Deleting old object point cloud visualization...");
        this->deleteMarkerArray(mObjectMeshMarkerArrayPtr, mObjectMeshMarkerPublisher);

        unsigned int index = 0;
        std::string ns = "mMeshMarkerArray";

        for(ObjectPointCloud::iterator it = objectPointCloud.begin(); it < objectPointCloud.end(); it++)
        {
            geometry_msgs::Pose pose = it->getPose();

            visualization_msgs::Marker objectMarker = this->getObjectMarker(pose, it->type, it->color, typeToMeshResource, index, ns);

            mObjectMeshMarkerArrayPtr->markers.push_back(objectMarker);

            index++;
        }

        mObjectMeshMarkerPublisher.publish(*mObjectMeshMarkerArrayPtr);

        ROS_DEBUG("Deleting old object normals visualization...");
        this->deleteMarkerArray(mObjectNormalsMarkerArrayPtr, mPointObjectNormalPublisher);

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
                mObjectNormalsMarkerArrayPtr->markers.push_back(objectNormalMarker);

                index++;
            }
        }

        mPointObjectNormalPublisher.publish(*mObjectNormalsMarkerArrayPtr);
    }

    void triggerFrustumObjectPointCloudVisualization(ObjectPointCloud frustumObjectPointCloud, std::map<std::string, std::string> typeToMeshResource) {
        ROS_DEBUG("Publishing Frustum Marker Array");
        std_msgs::ColorRGBA colorFrustumMeshMarker = this->createColorRGBA(0, 0, 1, 0.8);

        ROS_DEBUG("Deleting old frustum object point cloud visualization...");
        this->deleteMarkerArray(mFrustumObjectMeshMarkerArrayPtr, mFrustumObjectMeshMarkerPublisher);

        unsigned int index = 0;
        std::string ns = "mFrustumObjectMesh";

        for(ObjectPointCloud::iterator it = frustumObjectPointCloud.begin(); it < frustumObjectPointCloud.end(); it++)
        {
            geometry_msgs::Pose pose = it->getPose();

            visualization_msgs::Marker objectMarker = this->getObjectMarker(pose, it->type, colorFrustumMeshMarker, typeToMeshResource, index, ns);

            mFrustumObjectMeshMarkerArrayPtr->markers.push_back(objectMarker);

            index++;
        }
        mFrustumObjectMeshMarkerPublisher.publish(*mFrustumObjectMeshMarkerArrayPtr);
    }

    void triggerCropBoxVisualization(const std::vector<CropBoxPtr> cropBoxListPtr)
    {
        int id = 0;
        for(std::vector<CropBoxPtr>::const_iterator it = cropBoxListPtr.begin(); it != cropBoxListPtr.end(); ++it)
        {
            Eigen::Vector4f ptMin,ptMax;
            ptMin = (*it)->getMin();
            ptMax = (*it)->getMax();
            Eigen::Vector3f rotation, translation;
            rotation = (*it)->getRotation();
            translation = (*it)->getTranslation();

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

            SimpleVector4 color = SimpleVector4(0.5,0,0,0.5);

            SimpleQuaternion orientation(rotationMatrix);

            SimpleVector3 scale;
            scale[0] = std::abs(ptMax[0] - ptMin[0]);
            scale[1] = std::abs(ptMax[1] - ptMin[1]);
            scale[2] = std::abs(ptMax[2] - ptMin[2]);

            mCropBoxMarkerArrayPtr->markers.push_back(MarkerHelper::getCubeMarker(id, position_map_frame, orientation, scale,  color, "cropbox_ns"));
            id++;
        }
        mCropBoxMarkerPublisher.publish(*mCropBoxMarkerArrayPtr);
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
                ROS_DEBUG_STREAM(ia.what());
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

    void triggerCameraVis(std::string s,SimpleVector3 position,
                          const SimpleQuaternionCollectionPtr &sampledOrientationsPtr,
                          ViewportPoint currentBestViewport) {
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
    
    static void deleteMarkerArray(visualization_msgs::MarkerArray::Ptr &array, ros::Publisher &publisher)
    {
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
}
