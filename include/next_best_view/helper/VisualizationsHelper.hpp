#pragma once

#include "typedef.hpp"
#include <vector>

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

    ros::NodeHandle mNodeHandle;

    visualization_msgs::MarkerArray mIterationMarkerArray;
    visualization_msgs::MarkerArray mIterationMarkerArrayDeleteList;

    // TODO rename
    visualization_msgs::MarkerArray::Ptr mMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mMeshMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mObjectNormalsMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mFrustumMeshMarkerArrayPtr;

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

        // initalize data
        mNodeHandle.getParam("/nbv/boolClearBetweenIterations", mBoolClearBetweenIterations);
        ROS_DEBUG_STREAM("boolClearBetweenIterations: " << mBoolClearBetweenIterations);
        mIterationMarkerArray = visualization_msgs::MarkerArray();

        //Initialize marker arrays
        visualization_msgs::MarkerArray *mMeshMarkerArray = new visualization_msgs::MarkerArray();
        visualization_msgs::MarkerArray *mFrustumMeshMarkerArray = new visualization_msgs::MarkerArray();
        visualization_msgs::MarkerArray *mObjectNormalsMarkerArray = new visualization_msgs::MarkerArray();

        mMeshMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>(*mMeshMarkerArray);
        mObjectNormalsMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>(*mObjectNormalsMarkerArray);
        mFrustumMeshMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>(*mFrustumMeshMarkerArray);
    }

    void triggerIterationVisualizations(int iterationStep, SimpleVector3 position, const SimpleQuaternionCollectionPtr
                               &sampledOrientationsPtr, ViewportPoint currentBestViewport,
                               IndicesPtr feasibleIndices,SamplePointCloudPtr pointcloud,
                               SpaceSamplerPtr spaceSamplerPtr) {

        ROS_DEBUG_STREAM("iteration visualization");

        if(iterationStep==0 && mBoolClearBetweenIterations == true) {
            clearIterationVisualzation();
        }

        std::string s = boost::lexical_cast<std::string>(iterationStep);
        this->mIterationStep=iterationStep;
        m_j = iterationStep*0.25;
        m_j = std::min(1.0f, m_j); //Prevent overflow

        triggerSpaceSampling(feasibleIndices, pointcloud,s);
        triggerGrid(spaceSamplerPtr, s);
        triggerCameraVis(s, position, sampledOrientationsPtr, currentBestViewport);

        ROS_DEBUG_STREAM("publish markers");

        mIterationMarkerArrayPublisher.publish(mIterationMarkerArray);
    }

    void triggerFrustumVisualization(CameraModelFilterPtr camera, unsigned int numberSearchedObjects) {
        ROS_DEBUG("Publish actual frustum");

        this->clearFrustumVisualization();

        uint32_t sequence = 0;
        mMarkerArrayPtr = camera->getVisualizationMarkerArray(sequence, 0.0);

        for (unsigned int i = 0; i < mMarkerArrayPtr->markers.size(); i++)
        {
                mMarkerArrayPtr->markers.at(i).color.r = 0;
                mMarkerArrayPtr->markers.at(i).color.g = 1;
                mMarkerArrayPtr->markers.at(i).color.b = 1;
                mMarkerArrayPtr->markers.at(i).ns = "actual_nbv_frustum";
        }

        geometry_msgs::Pose pose;
        pose.position = TypeHelper::getPointMSG(camera->getPivotPointPosition());
        pose.orientation = TypeHelper::getQuaternionMSG(camera->getOrientation());

        std::string result = "searched objects: " + boost::lexical_cast<std::string>(numberSearchedObjects);
        // TODO change id?
        visualization_msgs::Marker textMarker = MarkerHelper::getTextMarker(mMarkerArrayPtr->markers.size(), result, pose, "searched_obj");
        mMarkerArrayPtr->markers.push_back(textMarker);

        mFrustumMarkerArrayPublisher.publish(*mMarkerArrayPtr);
    }

    void triggerOldFrustumVisualization(CameraModelFilterPtr camera) {
        ROS_DEBUG("Publish previous frustum");

        uint32_t sequence = 0;
        visualization_msgs::MarkerArray::Ptr markerArrayPtr;
        markerArrayPtr = camera->getVisualizationMarkerArray(sequence, 0.0);
        for (unsigned int i = 0; i < markerArrayPtr->markers.size(); i++)
        {
            markerArrayPtr->markers.at(i).color.r = 1;
            markerArrayPtr->markers.at(i).color.g = 0;
            markerArrayPtr->markers.at(i).color.b = 1;
            markerArrayPtr->markers.at(i).lifetime = ros::Duration(4.0);
            markerArrayPtr->markers.at(i).ns = "previous_nbv_frustum";
        }

        mFrustumMarkerArrayPublisher.publish(*markerArrayPtr);
    }

    // TODO private?
    // TODO delete?
    void clearFrustumVisualization()
    {
        ROS_DEBUG("Deleted last frustum in rviz");
        if (!mMarkerArrayPtr) {
            return;
        }

        for (unsigned int i = 0; i < mMarkerArrayPtr->markers.size(); i++)
        {
            mMarkerArrayPtr->markers.at(i).action = visualization_msgs::Marker::DELETE;
        }
        mFrustumMarkerArrayPublisher.publish(*mMarkerArrayPtr);
        // TODO mMarkerArrayPtr.reset(); ??
    }

    void triggerObjectPointCloudVisualization(ObjectPointCloud objectPointCloud, std::map<std::string, std::string> typeToMeshResource) {
        ROS_DEBUG("Publishing Point Cloud");

        ROS_DEBUG("Deleting old object point cloud visualization...");
        this->deleteMarkerArray(mMeshMarkerArrayPtr, mObjectMeshMarkerPublisher);

        unsigned int index = 0;
        std::string ns = "mMeshMarkerArray";

        for(ObjectPointCloud::iterator it = objectPointCloud.begin(); it < objectPointCloud.end(); it++)
        {
            std_msgs::ColorRGBA colorMeshMarker = it->color;
            std::string meshResource = typeToMeshResource[it->type];
            geometry_msgs::Pose pose = it->getPose();

            visualization_msgs::Marker objectMarker = MarkerHelper::getObjectMarker(index++, meshResource, pose, colorMeshMarker, ns);
            mMeshMarkerArrayPtr->markers.push_back(objectMarker);
        }

        mObjectMeshMarkerPublisher.publish(*mMeshMarkerArrayPtr);

        ROS_DEBUG("Deleting old object normals visualization...");
        this->deleteMarkerArray(mObjectNormalsMarkerArrayPtr, mPointObjectNormalPublisher);

        index = 0;
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
                                                                                                SimpleVector3(0.005, 0.01, 0.005),
                                                                                                SimpleVector4(1.0, 1.0, 0.0, 1.0), ns);
                mObjectNormalsMarkerArrayPtr->markers.push_back(objectNormalMarker);

                index++;
            }
        }

        mPointObjectNormalPublisher.publish(*mObjectNormalsMarkerArrayPtr);
    }

    void triggerFrustumObjectPointCloudVisualization(ObjectPointCloud frustumObjectPointCloud, std::map<std::string, std::string> typeToMeshResource) {
        ROS_DEBUG("Publishing Frustum Marker Array");
        std_msgs::ColorRGBA colorFrustumMeshMarker = VisualizationHelper::createColorRGBA(0, 0, 1, 0.8);

        ROS_DEBUG("Deleting old frustum object point cloud visualization...");
        this->deleteMarkerArray(mFrustumMeshMarkerArrayPtr, mFrustumObjectMeshMarkerPublisher);

        unsigned int index = 0;
        std::string ns = "mFrustumObjectMesh";

        for(ObjectPointCloud::iterator it = frustumObjectPointCloud.begin(); it < frustumObjectPointCloud.end(); it++)
        {
            std::string meshResource = typeToMeshResource[it->type];
            geometry_msgs::Pose pose = it->getPose();

            visualization_msgs::Marker objectMarker = MarkerHelper::getObjectMarker(index++, meshResource, pose, colorFrustumMeshMarker, ns);
            mFrustumMeshMarkerArrayPtr->markers.push_back(objectMarker);
        }
        mFrustumObjectMeshMarkerPublisher.publish(*mFrustumMeshMarkerArrayPtr);
    }

    // TODO rename
    void triggerFrustumMarkerArrayVisualization(CameraModelFilterPtr camera, int numberSearchedObjects = -1, geometry_msgs::Pose pose = geometry_msgs::Pose()) {
        visualization_msgs::MarkerArray::Ptr TempMarkerArray(new viz::MarkerArray);

        if (mMarkerArrayPtr)
        {
            this->clearFrustumVisualization();
            ROS_DEBUG("Publishing old frustum");
            for (unsigned int i = 0; i < mMarkerArrayPtr->markers.size(); i++)
            {
                mMarkerArrayPtr->markers.at(i).lifetime = ros::Duration(4.0);
                mMarkerArrayPtr->markers.at(i).action = visualization_msgs::Marker::ADD;
                // TODO change id?
                mMarkerArrayPtr->markers.at(i).id +=  mMarkerArrayPtr->markers.size();
                mMarkerArrayPtr->markers.at(i).color.r = 1;
                mMarkerArrayPtr->markers.at(i).color.g = 0;
                mMarkerArrayPtr->markers.at(i).color.b = 1;
                // TODO change ns?
                mMarkerArrayPtr->markers.at(i).ns = "old_nbv_frustum";
                TempMarkerArray->markers.push_back(mMarkerArrayPtr->markers.at(i));
            }

        }

        ROS_DEBUG_STREAM("Frustum Pivot Point : " << camera->getPivotPointPosition()[0] <<
                         " , " <<  camera->getPivotPointPosition()[1]
                         << " , " << camera->getPivotPointPosition()[2]);

        uint32_t sequence = 0;
        mMarkerArrayPtr = camera->getVisualizationMarkerArray(sequence, 0.0);

        ROS_DEBUG("Publishing new frustum");
        for (unsigned int i = 0; i < mMarkerArrayPtr->markers.size(); i++)
        {
            mMarkerArrayPtr->markers.at(i).color.r = 0;
            mMarkerArrayPtr->markers.at(i).color.g = 1;
            mMarkerArrayPtr->markers.at(i).color.b = 1;
            // TODO change ns?
            mMarkerArrayPtr->markers.at(i).ns = "new_nbv_frustum";
        }
        if (numberSearchedObjects != -1)
        {
            ROS_DEBUG("Adding text");
            ROS_INFO_STREAM("numberSearchedObjects: " <<  numberSearchedObjects);
            std::string result = "searched objects: " + boost::lexical_cast<std::string>(numberSearchedObjects);
            // TODO change id, ns ?
            std::string ns = "new_nbv_frustum_text";

            visualization_msgs::Marker textMarker = MarkerHelper::getTextMarker(mMarkerArrayPtr->markers.size(), result, pose, ns);
            mMarkerArrayPtr->markers.push_back(textMarker);
        }
        if (TempMarkerArray) {
            mFrustumMarkerArrayPublisher.publish(*TempMarkerArray);
        }
        ROS_DEBUG_STREAM("Marker array size:" << mMarkerArrayPtr->markers.size());
        mFrustumMarkerArrayPublisher.publish(*mMarkerArrayPtr);
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

        BOOST_FOREACH(SimpleQuaternion q, *sampledOrientationsPtr)
        {
            // get the data for the markers
            SimpleVector3 visualAxis = MathHelper::getVisualAxis(q);
            visualAxis[0] = visualAxis[0]/ViewPortMarkerShrinkFactor + position[0];
            visualAxis[1] = visualAxis[1]/ViewPortMarkerShrinkFactor + position[1];
            visualAxis[2] = visualAxis[2]/ViewPortMarkerShrinkFactor + position[2]
                                + mIterationStep * ViewPortMarkerHeightFactor;

            // get viewport direction marker
            std::vector<double> color = std::vector<double>(ViewPortDirectionsRGBA);
            color[0] -= m_j;
            color[1] += m_j;

            std::string ns = "ViewPortDirections" + s;

            m_i++;

            visualization_msgs::Marker ViewPortDirectionsMarker = MarkerHelper::getArrowMarker(m_i, visualAxis, q, ViewPortDirectionsScales, color, ns);
            visualization_msgs::Marker ViewPortDirectionsDeleteAction = MarkerHelper::getDeleteMarker(m_i, ns);

            mIterationMarkerArray.markers.push_back(ViewPortDirectionsMarker);
            mIterationMarkerArrayDeleteList.markers.push_back(ViewPortDirectionsDeleteAction);

            // get viewport marker
            color = std::vector<double>(ViewPortMarkerRGBA);
            color[0] -= m_j;
            color[1] += m_j;

            ns = "visualAxis" + s;

            m_i++;

            visualization_msgs::Marker ViewPortMarker = MarkerHelper::getCubeMarker(m_i, visualAxis, q, ViewPortMarkerScales, color, ns);
            visualization_msgs::Marker ViewPortMarkerDeleteAction = MarkerHelper::getDeleteMarker(m_i, ns);

            mIterationMarkerArray.markers.push_back(ViewPortMarker);
            mIterationMarkerArrayDeleteList.markers.push_back(ViewPortMarkerDeleteAction);
        }

        // get unit sphere marker
        SimpleVector3 spherePosition = SimpleVector3(position);
        spherePosition[2] += mIterationStep * ViewPortMarkerHeightFactor;

        std::vector<double> scale = std::vector<double>(3, 2.0/ViewPortMarkerShrinkFactor);

        std::vector<double> color = std::vector<double>(ViewPortMarkerRGBA);
        color[0] -= m_j;
        color[1] += m_j;
        color[3] /= 2.0;

        std::string ns = "visualAxisSphere" + s;

        m_i++;

        visualization_msgs::Marker ViewPortSphereMarker = MarkerHelper::getSphereMarker(m_i, spherePosition, scale, color, ns);
        visualization_msgs::Marker ViewPortSphereDeleteAction = MarkerHelper::getDeleteMarker(m_i, ns);

        mIterationMarkerArrayDeleteList.markers.push_back(ViewPortSphereDeleteAction);
        mIterationMarkerArray.markers.push_back(ViewPortSphereMarker);

        // get column position marker
        SimpleVector3 point1 = SimpleVector3(position);
        SimpleVector3 point2 = SimpleVector3(position);
        point1[2] = 0;
        point2[2] += mIterationStep * ViewPortMarkerHeightFactor;
        std::vector<SimpleVector3> points;
        points.push_back(point1);
        points.push_back(point2);

        ns = "LineVizu" + s;

        m_i++;

        visualization_msgs::Marker ColumnPositionMarker = MarkerHelper::getLineListMarker(m_i, points, ColumnPositionMarkerWidth,
                                                                                            ColumnPositionMarkerRGBA, ns);
        visualization_msgs::Marker ColumnPositionDeleteAction = MarkerHelper::getDeleteMarker(m_i, ns);

        mIterationMarkerArrayDeleteList.markers.push_back(ColumnPositionDeleteAction);
        mIterationMarkerArray.markers.push_back(ColumnPositionMarker);

        // get nbv camera direction
        scale.clear();
        scale.push_back(1);
        scale.push_back(ColumnPositionMarkerWidth);
        scale.push_back(ColumnPositionMarkerWidth);

        color = std::vector<double>(ViewPortMarkerRGBA);
        color[0] -= m_j;
        color[1] += m_j;

        ns = "ArrowVizu" +s ;

        m_i++;

        visualization_msgs::Marker NextBestViewCameraDirectionMarker = MarkerHelper::getArrowMarker(m_i, position,
                                                                                                        currentBestViewport.getSimpleQuaternion(),
                                                                                                        scale, color, ns);
        visualization_msgs::Marker NextBestViewCameraDirectionDeleteAction = MarkerHelper::getDeleteMarker(m_i, ns);

        mIterationMarkerArrayDeleteList.markers.push_back(NextBestViewCameraDirectionDeleteAction);
        mIterationMarkerArray.markers.push_back(NextBestViewCameraDirectionMarker);
    }

    void triggerSpaceSampling(IndicesPtr feasibleIndices,SamplePointCloudPtr pointcloud, std::string s){

        // get parameters
        double SpaceSamplingMarkerScale;
        std::vector<double> SpaceSamplingMarkerRGBA;
        mNodeHandle.getParam("/nbv/SpaceSamplingMarker_Scale", SpaceSamplingMarkerScale);
        mNodeHandle.getParam("/nbv/SpaceSamplingMarker_RGBA", SpaceSamplingMarkerRGBA);

        std::vector<double> scale(3, SpaceSamplingMarkerScale);
        std::vector<double> color = std::vector<double>(SpaceSamplingMarkerRGBA);
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
            visualization_msgs::Marker SpaceSamplingMarkerDeleteAction = MarkerHelper::getDeleteMarker(m_i, ns);

            mIterationMarkerArray.markers.push_back(SpaceSamplingMarker);
            mIterationMarkerArrayDeleteList.markers.push_back(SpaceSamplingMarkerDeleteAction);
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

        std::vector<double> scale;
        scale.push_back(xwidth);
        scale.push_back(ywidth);
        scale.push_back(GridMarkerScaleZ);

        std::string ns = "Radius" + s;

        m_i++;

        visualization_msgs::Marker GridMarker = MarkerHelper::getCubeMarker(m_i, position, orientation, scale, GridMarkerRGBA, ns);
        visualization_msgs::Marker GridMarkerDeleteAction = MarkerHelper::getDeleteMarker(m_i, ns);

        mIterationMarkerArray.markers.push_back(GridMarker);
        mIterationMarkerArrayDeleteList.markers.push_back(GridMarkerDeleteAction);
    }
    
    void clearIterationVisualzation(){
        mIterationMarkerArray.markers.clear();
        mIterationMarkerArrayPublisher.publish(mIterationMarkerArrayDeleteList);
        mIterationMarkerArrayDeleteList.markers.clear();
        m_i=0;
    }

    void deleteMarkerArray(visualization_msgs::MarkerArray::Ptr &array,ros::Publisher &publisher )
    {
        for (unsigned int i = 0; i < array->markers.size(); i++)
        {
            array->markers.at(i).action = visualization_msgs::Marker::DELETE;
        }
        publisher.publish(*array);
        array->markers.clear();
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
