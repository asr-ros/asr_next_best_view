#pragma once

#include "typedef.hpp"
#include <vector>

#include <boost/foreach.hpp>
#include <boost/range/algorithm_ext/iota.hpp>
#include <boost/lexical_cast.hpp>

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

    ros::Publisher vis_pub;
    ros::NodeHandle node_handle;
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::MarkerArray markerArrayDeleteList;
    int i;
    float j;
    int iterationStep;
    bool boolClearBetweenIterations;

public:

    VisualizationHelper():i(0),j(0),iterationStep(0) {
        vis_pub = node_handle.advertise<visualization_msgs::MarkerArray>( "/nbv/iteration_visualization", 1000);
        node_handle.getParam("/nbv/boolClearBetweenIterations", boolClearBetweenIterations);
        ROS_DEBUG_STREAM("boolClearBetweenIterations: " << boolClearBetweenIterations);
        markerArray = visualization_msgs::MarkerArray();
    }

    void triggerIterationVisualizations(int iterationStep, SimpleVector3 position, const SimpleQuaternionCollectionPtr
                               &sampledOrientationsPtr, ViewportPoint currentBestViewport,
                               IndicesPtr feasibleIndices,SamplePointCloudPtr pointcloud,
                               SpaceSamplerPtr spaceSamplerPtr) {

        ROS_DEBUG_STREAM("iteration visualization");

        if(iterationStep==0 && boolClearBetweenIterations == true) {
            clearVisualzation();
        }

        std::string s = boost::lexical_cast<std::string>(iterationStep);
        this->iterationStep=iterationStep;
        j = iterationStep*0.25;
        j = std::min(1.0f, j); //Prevent overflow

        triggerSpaceSampling(feasibleIndices, pointcloud,s);
        triggerGrid(spaceSamplerPtr, s);
        triggerCameraVis(s, position, sampledOrientationsPtr, currentBestViewport);

        ROS_DEBUG_STREAM("publish markers");

        vis_pub.publish(markerArray);
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

        node_handle.getParam("/nbv/ViewPortMarker_Scales", ViewPortMarkerScales);
        node_handle.getParam("/nbv/ViewPortMarker_HeightFactor", ViewPortMarkerHeightFactor);
        node_handle.getParam("/nbv/ViewPortMarker_ShrinkFactor", ViewPortMarkerShrinkFactor);
        node_handle.getParam("/nbv/ViewPortMarker_RGBA", ViewPortMarkerRGBA);
        node_handle.getParam("/nbv/ViewPortDirections_RGBA", ViewPortDirectionsRGBA);
        node_handle.getParam("/nbv/ViewPortDirections_Scales", ViewPortDirectionsScales);
        node_handle.getParam("/nbv/ColumnPositionMarker_Width", ColumnPositionMarkerWidth);
        node_handle.getParam("/nbv/ColumnPositionMarker_RGBA", ColumnPositionMarkerRGBA);

        BOOST_FOREACH(SimpleQuaternion q, *sampledOrientationsPtr)
        {
            // get the data for the markers
            SimpleVector3 visualAxis = MathHelper::getVisualAxis(q);
            visualAxis[0] = visualAxis[0]/ViewPortMarkerShrinkFactor + position[0];
            visualAxis[1] = visualAxis[1]/ViewPortMarkerShrinkFactor + position[1];
            visualAxis[2] = visualAxis[2]/ViewPortMarkerShrinkFactor + position[2]
                                + iterationStep * ViewPortMarkerHeightFactor;

            // get viewport direction marker
            std::vector<double> color = std::vector<double>(ViewPortDirectionsRGBA);
            color[0] -= j;
            color[1] += j;

            std::string ns = "ViewPortDirections" + s;

            i++;

            visualization_msgs::Marker ViewPortDirectionsMarker = MarkerHelper::getArrowMarker(i, visualAxis, q, ViewPortDirectionsScales, color, ns);
            visualization_msgs::Marker ViewPortDirectionsDeleteAction = MarkerHelper::getDeleteMarker(i, ns);

            markerArray.markers.push_back(ViewPortDirectionsMarker);
            markerArrayDeleteList.markers.push_back(ViewPortDirectionsDeleteAction);

            // get viewport marker
            color = std::vector<double>(ViewPortMarkerRGBA);
            color[0] -= j;
            color[1] += j;

            ns = "visualAxis" + s;

            i++;

            visualization_msgs::Marker ViewPortMarker = MarkerHelper::getCubeMarker(i, visualAxis, q, ViewPortMarkerScales, color, ns);
            visualization_msgs::Marker ViewPortMarkerDeleteAction = MarkerHelper::getDeleteMarker(i, ns);

            markerArray.markers.push_back(ViewPortMarker);
            markerArrayDeleteList.markers.push_back(ViewPortMarkerDeleteAction);
        }

        // get unit sphere marker
        SimpleVector3 spherePosition = SimpleVector3(position);
        spherePosition[2] += iterationStep * ViewPortMarkerHeightFactor;

        std::vector<double> scale = std::vector<double>(3, 2.0/ViewPortMarkerShrinkFactor);

        std::vector<double> color = std::vector<double>(ViewPortMarkerRGBA);
        color[0] -= j;
        color[1] += j;
        color[3] /= 2.0;

        std::string ns = "visualAxisSphere" + s;

        i++;

        visualization_msgs::Marker ViewPortSphereMarker = MarkerHelper::getSphereMarker(i, spherePosition, scale, color, ns);
        visualization_msgs::Marker ViewPortSphereDeleteAction = MarkerHelper::getDeleteMarker(i, ns);

        markerArrayDeleteList.markers.push_back(ViewPortSphereDeleteAction);
        markerArray.markers.push_back(ViewPortSphereMarker);

        // get column position marker
        SimpleVector3 point1 = SimpleVector3(position);
        SimpleVector3 point2 = SimpleVector3(position);
        point1[2] = 0;
        point2[2] += iterationStep * ViewPortMarkerHeightFactor;
        std::vector<SimpleVector3> points;
        points.push_back(point1);
        points.push_back(point2);

        ns = "LineVizu" + s;

        i++;

        visualization_msgs::Marker ColumnPositionMarker = MarkerHelper::getLineListMarker(i, points, ColumnPositionMarkerWidth,
                                                                                            ColumnPositionMarkerRGBA, ns);
        visualization_msgs::Marker ColumnPositionDeleteAction = MarkerHelper::getDeleteMarker(i, ns);

        markerArrayDeleteList.markers.push_back(ColumnPositionDeleteAction);
        markerArray.markers.push_back(ColumnPositionMarker);

        // get nbv camera direction
        scale.clear();
        scale.push_back(1);
        scale.push_back(ColumnPositionMarkerWidth);
        scale.push_back(ColumnPositionMarkerWidth);

        color = std::vector<double>(ViewPortMarkerRGBA);
        color[0] -= j;
        color[1] += j;

        ns = "ArrowVizu" +s ;

        i++;

        visualization_msgs::Marker NextBestViewCameraDirectionMarker = MarkerHelper::getArrowMarker(i, position,
                                                                                                        currentBestViewport.getSimpleQuaternion(),
                                                                                                        scale, color, ns);
        visualization_msgs::Marker NextBestViewCameraDirectionDeleteAction = MarkerHelper::getDeleteMarker(i, ns);

        markerArrayDeleteList.markers.push_back(NextBestViewCameraDirectionDeleteAction);
        markerArray.markers.push_back(NextBestViewCameraDirectionMarker);
    }

    void triggerSpaceSampling(IndicesPtr feasibleIndices,SamplePointCloudPtr pointcloud, std::string s){

        // get parameters
        double SpaceSamplingMarkerScale;
        std::vector<double> SpaceSamplingMarkerRGBA;
        node_handle.getParam("/nbv/SpaceSamplingMarker_Scale", SpaceSamplingMarkerScale);
        node_handle.getParam("/nbv/SpaceSamplingMarker_RGBA", SpaceSamplingMarkerRGBA);

        std::vector<double> scale(3, SpaceSamplingMarkerScale);
        std::vector<double> color = std::vector<double>(SpaceSamplingMarkerRGBA);
        color[0] -= j;
        color[1] += j;

        SamplePointCloud pcl = SamplePointCloud(*pointcloud, *feasibleIndices);

        for(SamplePointCloud::iterator it = pcl.points.begin(); it < pcl.points.end(); it++)
        {
            // get space sampling marker
            gm::Point point = it->getPoint();
            SimpleVector3 position = TypeHelper::getSimpleVector3(point);
            position[2] = 0.1;

            std::string ns = "SamplePoints_NS" + s;

            i++;

            visualization_msgs::Marker SpaceSamplingMarker = MarkerHelper::getCylinderMarker(i, position, 1, scale, color, ns);
            visualization_msgs::Marker SpaceSamplingMarkerDeleteAction = MarkerHelper::getDeleteMarker(i, ns);

            markerArray.markers.push_back(SpaceSamplingMarker);
            markerArrayDeleteList.markers.push_back(SpaceSamplingMarkerDeleteAction);
        }

    }

    void triggerGrid(SpaceSamplerPtr spaceSamplerPtr, std::string s){

        // get parameters
        double GridMarkerScaleZ;
        std::vector<double> GridMarkerRGBA;
        node_handle.getParam("/nbv/GridMarker_ScaleZ", GridMarkerScaleZ);
        node_handle.getParam("/nbv/GridMarker_RGBA", GridMarkerRGBA);

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

        i++;

        visualization_msgs::Marker GridMarker = MarkerHelper::getCubeMarker(i, position, orientation, scale, GridMarkerRGBA, ns);
        visualization_msgs::Marker GridMarkerDeleteAction = MarkerHelper::getDeleteMarker(i, ns);

        markerArray.markers.push_back(GridMarker);
        markerArrayDeleteList.markers.push_back(GridMarkerDeleteAction);
    }
    
    void clearVisualzation(){
        markerArray.markers.clear();
        vis_pub.publish(markerArrayDeleteList);
        markerArrayDeleteList.markers.clear();
        i=0;
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
