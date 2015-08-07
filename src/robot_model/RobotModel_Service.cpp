#include "next_best_view/robot_model/impl/MILDRobotModel.hpp"
#include "next_best_view/robot_model/impl/MILDRobotState.hpp"
#include "next_best_view/robot_model/RobotState.hpp"
#include <ros/ros.h>
#include <typedef.hpp>
#include "next_best_view/GetMovementCosts.h"
#include "next_best_view/GetDistance.h"
#include "next_best_view/CalculateRobotState.h"
#include "next_best_view/CalculateCameraPose.h"
#include "next_best_view/IsPositionReachable.h"
#include "next_best_view/RobotStateMessage.h"
#include "Eigen/Dense"
#include <pcl/point_cloud.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>
#include "typedef.hpp"


using namespace next_best_view;
MILDRobotModelPtr robotModelPtr;

bool getMovementCosts(GetMovementCosts::Request  &req, GetMovementCosts::Response &res)
{
    float costs;
    MILDRobotState * currentState = new MILDRobotState(req.currentState.pan, req.currentState.tilt,req.currentState.rotation,req.currentState.x,req.currentState.y);
    MILDRobotState * targetState = new MILDRobotState(req.targetState.pan, req.targetState.tilt,req.targetState.rotation,req.targetState.x,req.targetState.y);
    MILDRobotStatePtr currentStatePtr(currentState);
    MILDRobotStatePtr targetStatePtr(targetState);

    costs = robotModelPtr->getMovementCosts(currentStatePtr, targetStatePtr);
    res.costs = costs;
    return true;
}

bool getDistance(GetDistance::Request &req, GetDistance::Response &res)
{
  res.distance = robotModelPtr->getDistance(req.sourcePosition, req.targetPosition);
  return true;
}

bool calculateRobotState(CalculateRobotState::Request  &req, CalculateRobotState::Response &res)
{
  MILDRobotState * sourceRobotState = new MILDRobotState(req.sourceRobotState.pan, req.sourceRobotState.tilt,req.sourceRobotState.rotation,req.sourceRobotState.x,req.sourceRobotState.y);
  RobotStatePtr sourceRobotStatePtr(sourceRobotState);

  Eigen::Vector3d position;
  Eigen::Quaternion<double> orientation;
        
  tf::quaternionMsgToEigen(req.orientation, orientation);
  tf::vectorMsgToEigen(req.position, position);

  SimpleVector3 positionF = position.cast<float>();
  SimpleQuaternion orientationF = orientation.cast<float>();

  MILDRobotStatePtr newRobotState = boost::static_pointer_cast<MILDRobotState>(robotModelPtr->calculateRobotState(sourceRobotStatePtr, positionF, orientationF));

  RobotStateMessage newRobotStateMessage;
  newRobotStateMessage.pan = newRobotState->pan;
  newRobotStateMessage.tilt = newRobotState->tilt;
  newRobotStateMessage.rotation = newRobotState->rotation;
  newRobotStateMessage.y = newRobotState->y;
  newRobotStateMessage.x = newRobotState->x;
  res.newRobotState = newRobotStateMessage;

  return true;
}

bool calculateCameraPose(CalculateCameraPose::Request &req, CalculateCameraPose::Response &res)
{
    MILDRobotState * sourceRobotState = new MILDRobotState(req.sourceRobotState.pan, req.sourceRobotState.tilt,req.sourceRobotState.rotation,req.sourceRobotState.x,req.sourceRobotState.y);
    RobotStatePtr sourceRobotStatePtr(sourceRobotState);
    res.cameraFrame = robotModelPtr->calculateCameraPose(sourceRobotStatePtr);
    return true;
}

bool isPositionReachable(IsPositionReachable::Request &req, IsPositionReachable::Response &res)
{
  res.isReachable = robotModelPtr->isPositionReachable(req.sourcePosition, req.targetPosition);
  return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "getMovementCosts");
    ros::NodeHandle n;
    ros::ServiceServer service_GetMovementCosts = n.advertiseService("GetMovementCosts", getMovementCosts);
    ros::ServiceServer service_GetDistance = n.advertiseService("GetDistance", getDistance);
    ros::ServiceServer service_CalculateRobotState = n.advertiseService("CalculateRobotState", calculateRobotState);
    ros::ServiceServer service_CalculateCameraPose = n.advertiseService("CalculateCameraPose", calculateCameraPose);
    ros::ServiceServer service_IsPositionReachable = n.advertiseService("IsPositionReachable", isPositionReachable);

    robotModelPtr = MILDRobotModelPtr(new MILDRobotModel());
    robotModelPtr->setTiltAngleLimits(-45, 45);
    robotModelPtr->setPanAngleLimits(-60, 60);

    ROS_INFO("RobotModel Service started.");
    ros::spin();

    return 0;
}
