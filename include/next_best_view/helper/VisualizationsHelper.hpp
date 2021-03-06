/**

Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Heller Florian, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Stroh Daniel, Trautmann Jeremias, Walter Milena, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

4. The use is explicitly not permitted to any application which deliberately try to kill or do harm to any living creature.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

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
#include "next_best_view/space_sampler/SpaceSampler.hpp"
#include "next_best_view/rating/RatingModule.hpp"
#include "next_best_view/rating/impl/DefaultScoreContainer.hpp"
#include "next_best_view/crop_box/CropBoxWrapper.hpp"
#include "asr_msgs/AsrAttributedPointCloud.h"
#include "asr_msgs/AsrAttributedPoint.h"
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
    ros::Publisher mFrustumObjectMeshMarkerPublisher;
    ros::Publisher mPointObjectNormalPublisher;
    ros::Publisher mCropBoxMarkerPublisher;
    ros::Publisher mSamplingPublisher;

    ros::NodeHandle mNodeHandle;

    visualization_msgs::MarkerArray::Ptr mIterationMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mNewFrustumMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mOldFrustumMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mObjectMeshMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mObjectNormalsMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mFrustumObjectMeshMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mFrustumObjectNormalsMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mCropBoxMarkerArrayPtr;
    visualization_msgs::MarkerArray::Ptr mSamplingMarkerArrayPtr;

    MapHelperPtr mMapHelperPtr;
    DebugHelperPtr mDebugHelperPtr;

    int m_i;
    float m_j;
    int mIterationStep;
    bool mBoolClearBetweenIterations;
    int mSampleCounter;

    // mutex to lock ray tracing visualization.
    boost::mutex mutex;

public:

    VisualizationHelper(MapHelperPtr mapHelperPtr):mMapHelperPtr(mapHelperPtr),m_i(0),m_j(0),mIterationStep(0) {
        mDebugHelperPtr = DebugHelper::getInstance();

        // TODO: create something like a visualization package class that contains publisher, parameter and markers
        std::string iterationVisualization;
        std::string frustumVisualization;
        std::string objectsVisualization;
        std::string objectNormalsVisualization;
        std::string frustumObjectsVisualization;
        std::string frustumObjectNormalsVisualization;
        std::string cropBoxVisualization;
        std::string samplingVisualization;

        mNodeHandle.getParam("/nbv/iterationVisualization", iterationVisualization);
        mNodeHandle.getParam("/nbv/frustumVisualization", frustumVisualization);
        mNodeHandle.getParam("/nbv/objectsVisualization", objectsVisualization);
        mNodeHandle.getParam("/nbv/objectNormalsVisualization", objectNormalsVisualization);
        mNodeHandle.getParam("/nbv/frustumObjectsVisualization", frustumObjectsVisualization);
        mNodeHandle.getParam("/nbv/frustumObjectNormalsVisualization", frustumObjectNormalsVisualization);
        mNodeHandle.getParam("/nbv/cropBoxVisualization", cropBoxVisualization);
        mNodeHandle.getParam("/nbv/samplingVisualization", samplingVisualization);

        // initialize publishers
        mIterationMarkerArrayPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>(iterationVisualization, 1000);
        mFrustumMarkerArrayPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>(frustumVisualization, 1000);
        mObjectMeshMarkerPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>(objectsVisualization, 100, false);
        mFrustumObjectMeshMarkerPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>(frustumObjectsVisualization, 100, false);
        mPointObjectNormalPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>(objectNormalsVisualization, 100, false);
        mCropBoxMarkerPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>(cropBoxVisualization, 100, false);
        mSamplingPublisher = mNodeHandle.advertise<visualization_msgs::MarkerArray>(samplingVisualization, 10000, false);

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
        if (!mFrustumObjectMeshMarkerPublisher) {
            ROS_ERROR("mFrustumObjectMeshMarkerPublisher is invalid.");
            throw "Publisher invalid";
        }
        if (!mPointObjectNormalPublisher) {
            ROS_ERROR("mPointObjectNormalPublisher is invalid.");
            throw "Publisher invalid";
        }

        // initalize data
        mNodeHandle.getParam("/nbv/boolClearBetweenIterations", mBoolClearBetweenIterations);

        // initialize marker arrays
        mIterationMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>();
        mNewFrustumMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>();
        mOldFrustumMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>();
        mObjectMeshMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>();
        mObjectNormalsMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>();
        mFrustumObjectMeshMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>();
        mFrustumObjectNormalsMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>();
        mCropBoxMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>();
        mSamplingMarkerArrayPtr = boost::make_shared<visualization_msgs::MarkerArray>();

        mSampleCounter = 0;
    }

    bool getBoolClearBetweenIterations() {
        return mBoolClearBetweenIterations;
    }

    void triggerIterationVisualization(int iterationStep, const SimpleQuaternionCollectionPtr &sampledOrientationsPtr,
                                            ViewportPoint currentBestViewport,
                                            SamplePointCloudPtr pointcloud,
                                            SpaceSamplerPtr spaceSamplerPtr) {

        mDebugHelperPtr->writeNoticeably("STARTING ITERATION VISUALIZATION", DebugHelper::VISUALIZATION);

        if(!sampledOrientationsPtr){
            ROS_ERROR("triggerIterationVisualizations call with pointer sampledOrientationsPtr being null.");
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

        triggerSpaceSampling(pointcloud,s);
        triggerGrid(spaceSamplerPtr, s);
        triggerCameraVis(s, sampledOrientationsPtr, currentBestViewport);

        mDebugHelperPtr->write("Publishing markers", DebugHelper::VISUALIZATION);
        mIterationMarkerArrayPublisher.publish(mIterationMarkerArrayPtr);

        mDebugHelperPtr->writeNoticeably("ENDING ITERATION VISUALIZATION", DebugHelper::VISUALIZATION);
    }



    void triggerSamplingVisualization(ViewportPointCloudPtr samples, Color markerColor, std::string ns) {

        mDebugHelperPtr->writeNoticeably("STARTING SAMPLING VISUALIZATION", DebugHelper::VISUALIZATION);
        if (!samples) {
            ROS_ERROR("triggerSamplingVisualization call with pointer samples being null");
            mDebugHelperPtr->writeNoticeably("ENDING SAMPLING VISUALIZATION", DebugHelper::VISUALIZATION);
            return;
        }
        if (!mSamplingMarkerArrayPtr) {
            ROS_ERROR("triggerSamplingVisualization call with pointer mSamplingMarkerArrayPtr being null");
            mDebugHelperPtr->writeNoticeably("ENDING SAMPLING VISUALIZATION", DebugHelper::VISUALIZATION);
            return;
        }

        // filter multiple viewports that are on the same position
        SimpleVector3Collection filteredPositions;

        for (ViewportPoint &viewportPoint : *samples) {
            SimpleVector3 viewportPointPosition = viewportPoint.getPosition();
            viewportPointPosition[2] = 0.1;
            bool isInFilteredPositions = false;
            for (SimpleVector3 &filteredPosition : filteredPositions) {
                if (MathHelper::vector3Equal(filteredPosition, viewportPointPosition)) {
                    isInFilteredPositions = true;
                    break;
                }
            }
            if (!isInFilteredPositions) {
                filteredPositions.push_back(viewportPointPosition);
            }
        }

        // create markers
        ROS_INFO_STREAM("size: " << filteredPositions.size());
        double SpaceSamplingMarkerScale;
        mNodeHandle.getParam("/nbv/SpaceSamplingMarker_Scale", SpaceSamplingMarkerScale);
        SimpleVector3 scale(SpaceSamplingMarkerScale, SpaceSamplingMarkerScale, SpaceSamplingMarkerScale);
        for (SimpleVector3 &samplePosition : filteredPositions) {
            visualization_msgs::Marker samplingMarker = MarkerHelper::getCylinderMarker(mSampleCounter, samplePosition, 1, scale, markerColor, ns);
            mSamplingMarkerArrayPtr->markers.push_back(samplingMarker);
            mSampleCounter++;
        }

        mSamplingPublisher.publish(mSamplingMarkerArrayPtr);

        mDebugHelperPtr->writeNoticeably("ENDING SAMPLING VISUALIZATION", DebugHelper::VISUALIZATION);
    }

    void resetSamplingVisualization() {
        deleteMarkerArray(mSamplingMarkerArrayPtr, mSamplingPublisher);
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

    /**
     * @brief triggerObjectPointCloudVisualization shows only objects without hypothesis
     * @param objectPointCloud
     * @param typeToMeshResource
     */
    void triggerObjectPointCloudVisualization(ObjectPointCloud& objectPointCloud, std::map<std::string, std::string>& typeToMeshResource) {
        mDebugHelperPtr->writeNoticeably("STARTING OBJECT POINT CLOUD VISUALIZATION", DebugHelper::VISUALIZATION);

        if(!mObjectMeshMarkerArrayPtr){
            ROS_ERROR("triggerObjectPointCloudVisualization call with pointer mObjectMeshMarkerArrayPtr being null.");
            mDebugHelperPtr->writeNoticeably("ENDING OBJECT POINT CLOUD VISUALIZATION", DebugHelper::VISUALIZATION);
            return;
        }

        visualizePointCloudObjects(objectPointCloud, typeToMeshResource,
                                   mObjectMeshMarkerArrayPtr, mObjectMeshMarkerPublisher);

        mDebugHelperPtr->writeNoticeably("ENDING OBJECT POINT CLOUD VISUALIZATION", DebugHelper::VISUALIZATION);
    }

    /**
     * @brief triggerFrustumObjectPointCloudVisualization shows only objects without hypothesis.
     * @param frustumObjectPointCloud
     * @param typeToMeshResource
     */
    void triggerFrustumObjectPointCloudVisualization(ObjectPointCloud& frustumObjectPointCloud, std::map<std::string, std::string>& typeToMeshResource) {
        mDebugHelperPtr->writeNoticeably("STARTING FRUSTUM OBJECT POINT CLOUD VISUALIZATION", DebugHelper::VISUALIZATION);

        if(!mFrustumObjectMeshMarkerArrayPtr){
            ROS_ERROR("triggerFrustumObjectPointCloudVisualization call with pointer mFrustumObjectMeshMarkerArrayPtr being null.");
            mDebugHelperPtr->writeNoticeably("ENDING FRUSTUM OBJECT POINT CLOUD VISUALIZATION", DebugHelper::VISUALIZATION);
            return;
        }

        std_msgs::ColorRGBA::Ptr colorFrustumMeshMarkerPtr(new std_msgs::ColorRGBA(this->createColorRGBA(0, 0, 1, 0.8)));

        visualizePointCloudObjects(frustumObjectPointCloud, typeToMeshResource,
                                   mFrustumObjectMeshMarkerArrayPtr, mFrustumObjectMeshMarkerPublisher,
                                   colorFrustumMeshMarkerPtr);

        mDebugHelperPtr->writeNoticeably("ENDING FRUSTUM OBJECT POINT CLOUD VISUALIZATION", DebugHelper::VISUALIZATION);
    }

    void clearFrustumObjectPointCloudVisualization() {
        deleteMarkerArray(mFrustumObjectMeshMarkerArrayPtr, mFrustumObjectMeshMarkerPublisher);
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

    void triggerObjectNormalsVisualization(ObjectPointCloud& objectPointCloud) {
        mDebugHelperPtr->writeNoticeably("STARTING OBJECT POINT CLOUD HYPOTHESIS VISUALIZATION", DebugHelper::VISUALIZATION);

        if(!mObjectNormalsMarkerArrayPtr){
            ROS_ERROR("triggerObjectPointCloudHypothesisVisualization call with pointer mObjectNormalsMarkerArrayPtr being null.");
            mDebugHelperPtr->writeNoticeably("ENDING OBJECT POINT CLOUD HYPOTHESIS VISUALIZATION", DebugHelper::VISUALIZATION);
            return;
        }

        visualizePointCloudNormals(objectPointCloud,
                                   mObjectNormalsMarkerArrayPtr, mPointObjectNormalPublisher);

        mDebugHelperPtr->writeNoticeably("ENDING OBJECT POINT CLOUD HYPOTHESIS VISUALIZATION", DebugHelper::VISUALIZATION);
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

    void triggerSpaceSampling(SamplePointCloudPtr pointcloud, std::string s){

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


        for(SamplePointCloud::iterator it = pointcloud->points.begin(); it < pointcloud->points.end(); it++)
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

        double xwidth = std::abs(spaceSamplerPtr->getXtop() - spaceSamplerPtr->getXbot());
        double ywidth = std::abs(spaceSamplerPtr->getYtop() - spaceSamplerPtr->getYbot());
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

    /**
     * @brief visualizePointCloudObjects visualizes objects of objectPointCloud, using typeToMeshResource to get meshes.
     * @param objectPointCloud contains objects
     * @param typeToMeshResource contains mesh informations per object
     * @param objectMarkerArrayPtr where to place new markers/current markers are located.
     * @param objectPublisher where to publish the markers
     * @param objectColorPtr color of objects, can be null/not set if object color should be used.
     */
    static void visualizePointCloudObjects(ObjectPointCloud& objectPointCloud, std::map<std::string, std::string>& typeToMeshResource,
                                           visualization_msgs::MarkerArray::Ptr objectMarkerArrayPtr, ros::Publisher& objectPublisher,
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
    }

    /**
     * @brief visualizePointCloudNormals visualizes all hypothesis of each object.
     * @param objectPointCloud contains objects
     * @param objectNormalsMarkerArrayPtr where to place new markers/current markers are located.
     * @param objectNormalsPublisher where to publish the markers
     */
    static void visualizePointCloudNormals(ObjectPointCloud& objectPointCloud,
                                           visualization_msgs::MarkerArray::Ptr objectNormalsMarkerArrayPtr, ros::Publisher& objectNormalsPublisher) {
        DebugHelperPtr debugHelperPtr = DebugHelper::getInstance();

        debugHelperPtr->write("Deleting old object normals visualization", DebugHelper::VISUALIZATION);
        deleteMarkerArray(objectNormalsMarkerArrayPtr, objectNormalsPublisher);

        unsigned int index = 0;
        SimpleVector4 color = SimpleVector4(1.0, 1.0, 0.0, 1.0);
        SimpleVector3 scale = SimpleVector3(0.005, 0.01, 0.005);
        std::string ns = "ObjectNormals";

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
