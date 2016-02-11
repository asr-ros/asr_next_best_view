/*
 * next_best_view.cpp
 *
 *  Created on: Aug 31, 2014
 *      Author: ralfschleicher
 */

#include <assert.h>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <set>

#include "typedef.hpp"
#include "next_best_view/helper/MapHelper.hpp"
#include "next_best_view/NextBestView.hpp"
#include "next_best_view/helper/MathHelper.hpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "next_best_view");
	ros::start();

	next_best_view::NextBestView nbv;

	ros::spin();

	return 0;
}



