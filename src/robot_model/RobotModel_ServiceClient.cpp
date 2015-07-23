#include "next_best_view/robot_model/impl/MILDRobotModel.hpp"
#include "next_best_view/robot_model/impl/MILDRobotState.hpp"
#include <ros/ros.h>
#include <ros/init.h>
#include "next_best_view/GetMovementCosts.h"
#include "Eigen/Dense"
#include <pcl/point_cloud.h>


using namespace next_best_view;
MILDRobotModelPtr robotModelPtr;

bool getMovementCosts(GetMovementCosts::Request  &req, GetMovementCosts::Response &res)
{
    float distance;

    MILDRobotState * currentState = new MILDRobotState(req.currentState.pan, req.currentState.tilt,req.currentState.rotation,req.currentState.x,req.currentState.y);
    MILDRobotState * targetState = new MILDRobotState(req.targetState.pan, req.currentState.tilt,req.currentState.rotation,req.currentState.x,req.currentState.y);
    MILDRobotStatePtr currentStatePtr(currentState);
    MILDRobotStatePtr targetStatePtr(targetState);

    distance = robotModelPtr->getMovementCosts(currentStatePtr, targetStatePtr);
    res.distance = distance;
    return true;
}

bool calculateRobotState(const RobotStatePtr &sourceRobotState, const SimpleVector3 &position, const SimpleQuaternion &orientation)
{
  
  return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "getMovementCosts");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("getMovementCosts", getMovementCosts);

    robotModelPtr = MILDRobotModelPtr(new MILDRobotModel());
    robotModelPtr->setTiltAngleLimits(-45, 45);
    robotModelPtr->setPanAngleLimits(-60, 60);

    ROS_INFO("RobotModel Service Client started.");
    ros::spin();

    return 0;
}

