#pragma once

#include "typedef.hpp"
#include <vector>

#include <boost/foreach.hpp>
#include <boost/range/algorithm_ext/iota.hpp>
#include <boost/lexical_cast.hpp>

#include "next_best_view/hypothesis_updater/HypothesisUpdater.hpp"
#include "next_best_view/robot_model/RobotModel.hpp"
#include "next_best_view/camera_model_filter/CameraModelFilter.hpp"
#include "next_best_view/unit_sphere_sampler/UnitSphereSampler.hpp"
#include "next_best_view/space_sampler/SpaceSampler.hpp"
#include "next_best_view/rating/RatingModule.hpp"
#include "next_best_view/rating/impl/DefaultScoreContainer.hpp"
#include "next_best_view/AttributedPointCloud.h"
#include "next_best_view/AttributedPoint.h"
#include "next_best_view/helper/MapHelper.hpp"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

namespace next_best_view
{

class VisualizationHelper
{

private:

    void triggerCameraVis(std::string s,SimpleVector3 position,
                          const SimpleQuaternionCollectionPtr &sampledOrientationsPtr,
                          ViewportPoint currentBestViewport){

        double ViewPortMarkerScale;
        std::vector<double> ViewPortMarkerRGBA;
        double ViewPortMarkerHeightFactor;
        double ViewPortMarkerShrinkFactor;
        node_handle.getParam("/nbv/ViewPortMarker_Scale", ViewPortMarkerScale);
        node_handle.getParam("/nbv/ViewPortMarker_HeightFactor", ViewPortMarkerHeightFactor);
        node_handle.getParam("/nbv/ViewPortMarker_ShrinkFactor", ViewPortMarkerShrinkFactor);
        node_handle.getParam("/nbv/ViewPortMarker_RGBA", ViewPortMarkerRGBA);



        BOOST_FOREACH(SimpleQuaternion q, *sampledOrientationsPtr)
        {
            SimpleVector3 visualAxis = MathHelper::getVisualAxis(q);
            visualization_msgs::Marker ViewPortMarker = visualization_msgs::Marker();
            ViewPortMarker.header.stamp = ros::Time();
            ViewPortMarker.header.frame_id = "/map";
            ViewPortMarker.type = ViewPortMarker.SPHERE;
            ViewPortMarker.action = ViewPortMarker.ADD;
            ViewPortMarker.id = ++i;
            ViewPortMarker.lifetime = ros::Duration();
            ViewPortMarker.ns = "visualAxis" +s ;
            ViewPortMarker.scale.x = ViewPortMarkerScale;
            ViewPortMarker.scale.y = ViewPortMarkerScale;
            ViewPortMarker.scale.z = ViewPortMarkerScale;
            ViewPortMarker.color.a = ViewPortMarkerRGBA[3];
            ViewPortMarker.color.r = ViewPortMarkerRGBA[0]-j;
            ViewPortMarker.color.g = ViewPortMarkerRGBA[1]+j;
            ViewPortMarker.color.b = ViewPortMarkerRGBA[2];
            ViewPortMarker.pose.position.x = visualAxis[0]/ViewPortMarkerShrinkFactor +position[0];
            ViewPortMarker.pose.position.y = visualAxis[1]/ViewPortMarkerShrinkFactor +position[1];
            ViewPortMarker.pose.position.z = visualAxis[2]/ViewPortMarkerShrinkFactor +position[2]+iterationStep * ViewPortMarkerHeightFactor;
            ViewPortMarker.pose.orientation.w = 1;

            markerArray.markers.push_back(ViewPortMarker);
        }

        visualization_msgs::Marker ViewPortSphereMarker = visualization_msgs::Marker();

        ViewPortSphereMarker.header.stamp = ros::Time();
        ViewPortSphereMarker.header.frame_id = "/map";
        ViewPortSphereMarker.type = ViewPortSphereMarker.SPHERE;
        ViewPortSphereMarker.action = ViewPortSphereMarker.ADD;
        ViewPortSphereMarker.id = ++i;
        ViewPortSphereMarker.lifetime = ros::Duration();
        ViewPortSphereMarker.ns = "visualAxisSphere" +s ;
        ViewPortSphereMarker.scale.x = 2.0/ViewPortMarkerShrinkFactor;
        ViewPortSphereMarker.scale.y = 2.0/ViewPortMarkerShrinkFactor;
        ViewPortSphereMarker.scale.z = 2.0/ViewPortMarkerShrinkFactor;
        ViewPortSphereMarker.color.a = ViewPortMarkerRGBA[3] / 2.0;
        ViewPortSphereMarker.color.r = ViewPortMarkerRGBA[0]-j;
        ViewPortSphereMarker.color.g = ViewPortMarkerRGBA[1]+j;
        ViewPortSphereMarker.color.b = ViewPortMarkerRGBA[2];
        ViewPortSphereMarker.pose.position.x = position[0];
        ViewPortSphereMarker.pose.position.y = position[1];
        ViewPortSphereMarker.pose.position.z = position[2]+iterationStep * ViewPortMarkerHeightFactor;

        markerArray.markers.push_back(ViewPortSphereMarker);


        double ColumnPositionMarkerWidth;
        node_handle.getParam("/nbv/ColumnPositionMarker_Width", ColumnPositionMarkerWidth);

        visualization_msgs::Marker ColumnPositionMarker = visualization_msgs::Marker();

        ColumnPositionMarker.header.stamp = ros::Time();
        ColumnPositionMarker.header.frame_id = "/map";
        ColumnPositionMarker.type = ColumnPositionMarker.LINE_LIST;
        ColumnPositionMarker.action = ColumnPositionMarker.ADD;
        ColumnPositionMarker.id = ++i;
        ColumnPositionMarker.lifetime = ros::Duration();
        ColumnPositionMarker.ns = "LineVizu" +s ;
        ColumnPositionMarker.scale.x = ColumnPositionMarkerWidth;
        ColumnPositionMarker.color.a = ViewPortMarkerRGBA[3];
        ColumnPositionMarker.color.r = ViewPortMarkerRGBA[0]-j;
        ColumnPositionMarker.color.g = ViewPortMarkerRGBA[1]+j;
        ColumnPositionMarker.color.b = ViewPortMarkerRGBA[2];
        geometry_msgs::Point point1 = geometry_msgs::Point();
        point1.x = position[0];
        point1.y = position[1];
        point1.z = 0;
        geometry_msgs::Point point2 = geometry_msgs::Point();
        point2.x = position[0];
        point2.y = position[1];
        point2.z = position[2]+iterationStep*ViewPortMarkerHeightFactor;
        ColumnPositionMarker.points.push_back(point1);
        ColumnPositionMarker.points.push_back(point2);

        markerArray.markers.push_back(ColumnPositionMarker);

        visualization_msgs::Marker NextBestViewCameraDirectionMarker = visualization_msgs::Marker();

        NextBestViewCameraDirectionMarker.header.stamp = ros::Time();
        NextBestViewCameraDirectionMarker.header.frame_id = "/map";
        NextBestViewCameraDirectionMarker.type = NextBestViewCameraDirectionMarker.ARROW;
        NextBestViewCameraDirectionMarker.action = NextBestViewCameraDirectionMarker.ADD;
        NextBestViewCameraDirectionMarker.id = ++i;
        NextBestViewCameraDirectionMarker.lifetime = ros::Duration();
        NextBestViewCameraDirectionMarker.ns = "ArrowVizu" +s ;
        NextBestViewCameraDirectionMarker.scale.x = 1;
        NextBestViewCameraDirectionMarker.scale.y = ColumnPositionMarkerWidth;
        NextBestViewCameraDirectionMarker.scale.z = ColumnPositionMarkerWidth;
        NextBestViewCameraDirectionMarker.color.a = ViewPortMarkerRGBA[3];
        NextBestViewCameraDirectionMarker.color.r = ViewPortMarkerRGBA[0]-j;
        NextBestViewCameraDirectionMarker.color.g = ViewPortMarkerRGBA[1]+j;
        NextBestViewCameraDirectionMarker.color.b = ViewPortMarkerRGBA[2];
        NextBestViewCameraDirectionMarker.pose.position.x = position[0];
        NextBestViewCameraDirectionMarker.pose.position.y = position[1];
        NextBestViewCameraDirectionMarker.pose.position.z = position[2];
        NextBestViewCameraDirectionMarker.pose.orientation = currentBestViewport.getQuaternion();

        markerArray.markers.push_back(NextBestViewCameraDirectionMarker);
    }

    void triggerSpaceSampling(IndicesPtr feasibleIndices,SamplePointCloudPtr pointcloud, std::string s){

        double SpaceSamplingMarkerScale;
        std::vector<double> SpaceSamplingMarkerRGBA;
        node_handle.getParam("/nbv/SpaceSamplingMarker_Scale", SpaceSamplingMarkerScale);
        ROS_INFO("/nbv/SpaceSamplingMarkerScale %0.2f", SpaceSamplingMarkerScale);
        node_handle.getParam("/nbv/SpaceSamplingMarker_RGBA", SpaceSamplingMarkerRGBA);

        SamplePointCloud pcl = SamplePointCloud(*pointcloud, *feasibleIndices);

        for(SamplePointCloud::iterator it = pcl.points.begin(); it < pcl.points.end(); it++)
        {
            gm::Point point = it->getPoint();
            visualization_msgs::Marker SpaceSamplingMarker = visualization_msgs::Marker();
            SpaceSamplingMarker.header.stamp = ros::Time();
            SpaceSamplingMarker.header.frame_id = "/map";
            SpaceSamplingMarker.type = SpaceSamplingMarker.CUBE;
            SpaceSamplingMarker.action = SpaceSamplingMarker.ADD;
            SpaceSamplingMarker.id = ++i;
            SpaceSamplingMarker.lifetime = ros::Duration();
            SpaceSamplingMarker.ns = "SamplePoints_NS"+s;
            SpaceSamplingMarker.scale.x = SpaceSamplingMarkerScale;
            SpaceSamplingMarker.scale.y = SpaceSamplingMarkerScale;
            SpaceSamplingMarker.scale.z = SpaceSamplingMarkerScale;
            SpaceSamplingMarker.color.a = SpaceSamplingMarkerRGBA[3];
            SpaceSamplingMarker.color.r = SpaceSamplingMarkerRGBA[0]-j;
            SpaceSamplingMarker.color.g = SpaceSamplingMarkerRGBA[1]+j;
            SpaceSamplingMarker.color.b = SpaceSamplingMarkerRGBA[2];
            SpaceSamplingMarker.pose.position.x = point.x;
            SpaceSamplingMarker.pose.position.y = point.y;
            SpaceSamplingMarker.pose.position.z = 0.1;
            SpaceSamplingMarker.pose.orientation.w = 1;

            markerArray.markers.push_back(SpaceSamplingMarker);
        }

    }

    void triggerGrid(SpaceSamplerPtr spaceSamplerPtr, std::string s){

        double GridMarkerScaleZ;
        std::vector<double> GridMarkerRGBA;
        node_handle.getParam("/nbv/GridMarker_ScaleZ", GridMarkerScaleZ);
        node_handle.getParam("/nbv/GridMarker_RGBA", GridMarkerRGBA);

        double xwidth = abs(spaceSamplerPtr->getXtop() - spaceSamplerPtr->getXbot());
        double ywidth = abs(spaceSamplerPtr->getYtop() - spaceSamplerPtr->getYbot());
        double xmid = (spaceSamplerPtr->getXtop() + spaceSamplerPtr->getXbot())/2.0;
        double ymid =  (spaceSamplerPtr->getYtop() + spaceSamplerPtr->getYbot())/2.0;

        visualization_msgs::Marker GridMarker = visualization_msgs::Marker();
        GridMarker.header.stamp = ros::Time();
        GridMarker.header.frame_id = "/map";
        GridMarker.type = GridMarker.CUBE;
        GridMarker.action = GridMarker.ADD;
        GridMarker.id = ++i;
        GridMarker.lifetime = ros::Duration();
        GridMarker.ns = "Radius" + s;
        GridMarker.scale.x = xwidth;
        GridMarker.scale.y = ywidth;
        GridMarker.scale.z = GridMarkerScaleZ;
        GridMarker.color.a = GridMarkerRGBA[3];
        GridMarker.color.r = GridMarkerRGBA[0];
        GridMarker.color.g = GridMarkerRGBA[1];
        GridMarker.color.b = GridMarkerRGBA[2];
        GridMarker.pose.position.x = xmid;
        GridMarker.pose.position.y = ymid;
        GridMarker.pose.position.z = 0;
        GridMarker.pose.orientation.w = 1;

        markerArray.markers.push_back(GridMarker);
    }

    ros::Publisher vis_pub;
    ros::NodeHandle node_handle;
    visualization_msgs::MarkerArray markerArray;
    int i;
    float j;
    int iterationStep;
public:

    VisualizationHelper():i(0),j(0),iterationStep(0){
        node_handle;
        vis_pub = node_handle.advertise<visualization_msgs::MarkerArray>( "visualization_markerarray", 100 );
        markerArray = visualization_msgs::MarkerArray();
    }

    void triggerVisualizations(int iterationStep, SimpleVector3 position, const SimpleQuaternionCollectionPtr
                               &sampledOrientationsPtr, ViewportPoint currentBestViewport,
                               IndicesPtr feasibleIndices,SamplePointCloudPtr pointcloud,
                               SpaceSamplerPtr spaceSamplerPtr){

        std::string s = boost::lexical_cast<std::string>(iterationStep);
        this->iterationStep=iterationStep;
        j = iterationStep*0.15;

        triggerSpaceSampling(feasibleIndices, pointcloud,s);
        triggerGrid(spaceSamplerPtr, s);
        triggerCameraVis(s, position, sampledOrientationsPtr, currentBestViewport);

        vis_pub.publish(markerArray);

    }

};




}
