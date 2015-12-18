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

    void triggerCameraVis(std::string s,SimpleVector3 position,
                          const SimpleQuaternionCollectionPtr &sampledOrientationsPtr,
                          ViewportPoint currentBestViewport){

        std::vector<double> ViewPortMarkerScales;
        std::vector<double> ViewPortMarkerRGBA;
        std::vector<double> ViewPortDirectionsRGBA;
        std::vector<double> ViewPortDirectionsScales;
        double ViewPortMarkerHeightFactor;
        double ViewPortMarkerShrinkFactor;
        node_handle.getParam("/nbv/ViewPortMarker_Scales", ViewPortMarkerScales);
        node_handle.getParam("/nbv/ViewPortMarker_HeightFactor", ViewPortMarkerHeightFactor);
        node_handle.getParam("/nbv/ViewPortMarker_ShrinkFactor", ViewPortMarkerShrinkFactor);
        node_handle.getParam("/nbv/ViewPortMarker_RGBA", ViewPortMarkerRGBA);
        node_handle.getParam("/nbv/ViewPortDirections_RGBA", ViewPortDirectionsRGBA);
        node_handle.getParam("/nbv/ViewPortDirections_Scales", ViewPortDirectionsScales);


        BOOST_FOREACH(SimpleQuaternion q, *sampledOrientationsPtr)
        {

            SimpleVector3 visualAxis = MathHelper::getVisualAxis(q);
            geometry_msgs::Point extPoint;
            extPoint.x = visualAxis[0]/ViewPortMarkerShrinkFactor + position[0];
            extPoint.y = visualAxis[1]/ViewPortMarkerShrinkFactor + position[1];
            extPoint.z = visualAxis[2]/ViewPortMarkerShrinkFactor + position[2]
                    + iterationStep * ViewPortMarkerHeightFactor;
            //tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
            geometry_msgs::Quaternion orientation;
            orientation.w = q.w();
            orientation.x = q.x();
            orientation.y = q.y();
            orientation.z = q.z();

            visualization_msgs::Marker ViewPortDirectionsMarker = visualization_msgs::Marker();

            ViewPortDirectionsMarker.header.stamp = ros::Time();
            ViewPortDirectionsMarker.header.frame_id = "/map";
            ViewPortDirectionsMarker.type = ViewPortDirectionsMarker.ARROW;
            ViewPortDirectionsMarker.action = ViewPortDirectionsMarker.ADD;
            ViewPortDirectionsMarker.id = ++i;
            ViewPortDirectionsMarker.lifetime = ros::Duration();
            ViewPortDirectionsMarker.ns = "ViewPortDirections" +s ;
            ViewPortDirectionsMarker.scale.x = ViewPortDirectionsScales[0];
            ViewPortDirectionsMarker.scale.y = ViewPortDirectionsScales[1];
            ViewPortDirectionsMarker.scale.z = ViewPortDirectionsScales[2];
            ViewPortDirectionsMarker.color.a = ViewPortDirectionsRGBA[3];
            ViewPortDirectionsMarker.color.r = ViewPortDirectionsRGBA[0]-j;
            ViewPortDirectionsMarker.color.g = ViewPortDirectionsRGBA[1]+j;
            ViewPortDirectionsMarker.color.b = ViewPortDirectionsRGBA[2];
            ViewPortDirectionsMarker.pose.position = extPoint;
            ViewPortDirectionsMarker.pose.orientation = orientation;

            visualization_msgs::Marker ViewPortDirectionsDeleteAction = ViewPortDirectionsMarker;
            ViewPortDirectionsDeleteAction.action = ViewPortDirectionsDeleteAction.DELETE;
            markerArrayDeleteList.markers.push_back(ViewPortDirectionsDeleteAction);
            markerArray.markers.push_back(ViewPortDirectionsMarker);


            visualization_msgs::Marker ViewPortMarker = visualization_msgs::Marker();
            ViewPortMarker.header.stamp = ros::Time();
            ViewPortMarker.header.frame_id = "/map";
            ViewPortMarker.type = ViewPortMarker.CUBE                                                    ;
            ViewPortMarker.action = ViewPortMarker.ADD;
            ViewPortMarker.id = ++i;
            ViewPortMarker.lifetime = ros::Duration();
            ViewPortMarker.ns = "visualAxis" +s ;
            ViewPortMarker.scale.x = ViewPortMarkerScales[0];
            ViewPortMarker.scale.y = ViewPortMarkerScales[1];
            ViewPortMarker.scale.z = ViewPortMarkerScales[2];
            ViewPortMarker.color.a = ViewPortMarkerRGBA[3];
            ViewPortMarker.color.r = ViewPortMarkerRGBA[0]-j;
            ViewPortMarker.color.g = ViewPortMarkerRGBA[1]+j;
            ViewPortMarker.color.b = ViewPortMarkerRGBA[2];
            ViewPortMarker.pose.position = extPoint;
            ViewPortMarker.pose.orientation = orientation;

            visualization_msgs::Marker ViewPortMarkerDeleteAction = ViewPortMarker;
            ViewPortMarkerDeleteAction.action = ViewPortMarkerDeleteAction.DELETE;
            markerArrayDeleteList.markers.push_back(ViewPortMarkerDeleteAction);
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

        visualization_msgs::Marker  ViewPortSphereDeleteAction = ViewPortSphereMarker;
        ViewPortSphereDeleteAction.action = ViewPortSphereDeleteAction.DELETE;
        markerArrayDeleteList.markers.push_back(ViewPortSphereDeleteAction);
        markerArray.markers.push_back(ViewPortSphereMarker);


        double ColumnPositionMarkerWidth;
        std::vector<double> ColumnPositionMarkerRGBA;
        node_handle.getParam("/nbv/ColumnPositionMarker_Width", ColumnPositionMarkerWidth);
        node_handle.getParam("/nbv/ColumnPositionMarker_RGBA", ColumnPositionMarkerRGBA);
	
        visualization_msgs::Marker ColumnPositionMarker = visualization_msgs::Marker();

        ColumnPositionMarker.header.stamp = ros::Time();
        ColumnPositionMarker.header.frame_id = "/map";
        ColumnPositionMarker.type = ColumnPositionMarker.LINE_LIST;
        ColumnPositionMarker.action = ColumnPositionMarker.ADD;
        ColumnPositionMarker.id = ++i;
        ColumnPositionMarker.lifetime = ros::Duration();
        ColumnPositionMarker.ns = "LineVizu" +s ;
        ColumnPositionMarker.scale.x = ColumnPositionMarkerWidth;
        ColumnPositionMarker.color.a = ColumnPositionMarkerRGBA[3];
        ColumnPositionMarker.color.r = ColumnPositionMarkerRGBA[0];
        ColumnPositionMarker.color.g = ColumnPositionMarkerRGBA[1];
        ColumnPositionMarker.color.b = ColumnPositionMarkerRGBA[2];
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

        visualization_msgs::Marker ColumnPositionDeleteAction = ColumnPositionMarker;
        ColumnPositionDeleteAction.action = ColumnPositionDeleteAction.DELETE;
        markerArrayDeleteList.markers.push_back(ColumnPositionDeleteAction);
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

        visualization_msgs::Marker NextBestViewCameraDirectionDeleteAction = NextBestViewCameraDirectionMarker;
        NextBestViewCameraDirectionDeleteAction.action = NextBestViewCameraDirectionDeleteAction.DELETE;
        markerArrayDeleteList.markers.push_back(NextBestViewCameraDirectionDeleteAction);
        markerArray.markers.push_back(NextBestViewCameraDirectionMarker);
    }

    void triggerSpaceSampling(IndicesPtr feasibleIndices,SamplePointCloudPtr pointcloud, std::string s){

        double SpaceSamplingMarkerScale;
        std::vector<double> SpaceSamplingMarkerRGBA;
        node_handle.getParam("/nbv/SpaceSamplingMarker_Scale", SpaceSamplingMarkerScale);
        node_handle.getParam("/nbv/SpaceSamplingMarker_RGBA", SpaceSamplingMarkerRGBA);

        SamplePointCloud pcl = SamplePointCloud(*pointcloud, *feasibleIndices);

        for(SamplePointCloud::iterator it = pcl.points.begin(); it < pcl.points.end(); it++)
        {
            gm::Point point = it->getPoint();
            visualization_msgs::Marker SpaceSamplingMarker = visualization_msgs::Marker();
            SpaceSamplingMarker.header.stamp = ros::Time();
            SpaceSamplingMarker.header.frame_id = "/map";
            SpaceSamplingMarker.type = SpaceSamplingMarker.CYLINDER;
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

            visualization_msgs::Marker SpaceSamplingMarkerDeleteAction = SpaceSamplingMarker;
            SpaceSamplingMarkerDeleteAction.action = SpaceSamplingMarkerDeleteAction.DELETE;
            markerArrayDeleteList.markers.push_back(SpaceSamplingMarkerDeleteAction);

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

        visualization_msgs::Marker GridMarkerDeleteAction = GridMarker;
        GridMarkerDeleteAction.action = GridMarkerDeleteAction.DELETE;
        markerArrayDeleteList.markers.push_back(GridMarkerDeleteAction);

        markerArray.markers.push_back(GridMarker);
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

    ros::Publisher vis_pub;
    ros::NodeHandle node_handle;
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::MarkerArray markerArrayDeleteList;
    int i;
    float j;
    int iterationStep;
    bool boolClearBetweenIterations;
public:

    VisualizationHelper():i(0),j(0),iterationStep(0){
        vis_pub = node_handle.advertise<visualization_msgs::MarkerArray>( "/nbv/iteration_visualization", 1000);
        node_handle.getParam("/nbv/boolClearBetweenIterations", boolClearBetweenIterations);
        ROS_DEBUG_STREAM("boolClearBetweenIterations: " << boolClearBetweenIterations);
        markerArray = visualization_msgs::MarkerArray();
    }

    void triggerVisualizations(int iterationStep, SimpleVector3 position, const SimpleQuaternionCollectionPtr
                               &sampledOrientationsPtr, ViewportPoint currentBestViewport,
                               IndicesPtr feasibleIndices,SamplePointCloudPtr pointcloud,
                               SpaceSamplerPtr spaceSamplerPtr){
      
    if(iterationStep==0 && boolClearBetweenIterations == true)  clearVisualzation();
        std::string s = boost::lexical_cast<std::string>(iterationStep);
        this->iterationStep=iterationStep;
        j = iterationStep*0.25;
	j = std::min(1.0f, j); //Prevent overflow

        triggerSpaceSampling(feasibleIndices, pointcloud,s);
        triggerGrid(spaceSamplerPtr, s);
        triggerCameraVis(s, position, sampledOrientationsPtr, currentBestViewport);

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

};
}
