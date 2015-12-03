/*
 * ViewportPoint.hpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ralfschleicher
 */

#ifndef DEFAULTVIEWPORTPOINT_HPP_
#define DEFAULTVIEWPORTPOINT_HPP_

#define PCL_NO_PRECOMPILE
#include "typedef.hpp"
#include "next_best_view/rating/BaseScoreContainer.hpp"
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
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/impl/frustum_culling.hpp>

namespace next_best_view {
	namespace gm = geometry_msgs;

	/*!
	 * \brief DefaultViewportPoint
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 * \sa pcl::PointXYZ
	 */
  //Comment?
	struct DefaultViewportPoint {
	public:
		PCL_ADD_POINT4D;

		union {
			struct {
				float qw;
				float qx;
				float qy;
				float qz;
			};
		};

		ObjectPointCloudPtr child_point_cloud;
		IndicesPtr child_indices;
		ObjectNameSetPtr object_name_set;
		BaseScoreContainerPtr score;
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		DefaultViewportPoint(const gm::Pose &pose = gm::Pose()) : child_indices(new Indices()) {
			x = pose.position.x;
			y = pose.position.y;
			z = pose.position.z;

			qw = pose.orientation.w;
			qx = pose.orientation.x;
			qy = pose.orientation.y;
			qz = pose.orientation.z;
		}

		DefaultViewportPoint(const SimpleVector3 &vector, const SimpleQuaternion &orientation = SimpleQuaternion(1.0, 0.0, 0.0, 0.0)) : qw(orientation.w()), qx(orientation.x()), qy(orientation.y()), qz(orientation.z()) {
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

		gm::Quaternion getQuaternion() const {
			gm::Quaternion qt;
			qt.w = qw;
			qt.x = qx;
			qt.y = qy;
			qt.z = qz;
			return qt;
		}

		gm::Pose getPose() const {
			gm::Pose pose;
			pose.position = this->getPoint();
			pose.orientation = this->getQuaternion();
			return pose;
		}

        SimpleVector3 getPosition() const {
			return SimpleVector3(x, y, z);
		}

		SimpleQuaternion getSimpleQuaternion() const {
			return SimpleQuaternion(qw, qx, qy, qz);
		}

        void setOrientation(const SimpleQuaternion &orientation) {
			qw = orientation.w();
			qx = orientation.x();
			qy = orientation.y();
			qz = orientation.z();
		}
	} EIGEN_ALIGN16;
}
//Comment?
POINT_CLOUD_REGISTER_POINT_STRUCT (next_best_view::ViewportPoint,
   (float, x, x)
   (float, y, y)
   (float, z, z)
)


#endif /* DEFAULTVIEWPORTPOINT_HPP_ */
