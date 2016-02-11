/*
 * SpaceSamplePoint.hpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#ifndef SPACESAMPLEPOINT_HPP_
#define SPACESAMPLEPOINT_HPP_

#define PCL_NO_PRECOMPILE
#include "typedef.hpp"
#include <geometry_msgs/Pose.h>
#include <string>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/search/search.h>
#include <pcl/search/organized.h>
#include <pcl/search/impl/organized.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/impl/conditional_euclidean_clustering.hpp>
#include <pcl/pcl_base.h>
#include <pcl/impl/pcl_base.hpp>

namespace next_best_view {
	namespace gm = geometry_msgs;

	/*!
	 * \brief SpaceSamplePoint
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 * \sa pcl::PointXYZ
	 */
  //Comment?
	struct SpaceSamplePoint {
	public:
		PCL_ADD_POINT4D;

	  //Radius search result -> Comment?
		ObjectPointCloudPtr child_point_cloud;
		IndicesPtr child_indices;
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		SpaceSamplePoint(const gm::Pose &pose = gm::Pose()) {
			x = pose.position.x;
			y = pose.position.y;
			z = pose.position.z;
		}

		SpaceSamplePoint(const SimpleVector3 &vector) {
			x = vector[0];
			y = vector[1];
			z = vector[2];
		}

		gm::Point getPoint() const {
			gm::Point pt;
			pt.x = x;
			pt.y = y;
			pt.z = z;
			return pt;
		}

		SimpleVector3 getSimpleVector3() const {
			return SimpleVector3(x, y, z);
		}
	} EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT (next_best_view::SpaceSamplePoint,
   (float, x, x)
   (float, y, y)
   (float, z, z)
)


#endif /* SPACESAMPLEPOINT_HPP_ */
