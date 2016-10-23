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

#ifndef REALOBJECTPOINT_HPP_
#define REALOBJECTPOINT_HPP_

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
#include <std_msgs/ColorRGBA.h>

namespace next_best_view {
	namespace gm = geometry_msgs;

	/*!
	 * \brief RealObjectPoint
	 * \author Ralf Schleicher
	 * \date 2014
	 * \version 1.0
	 * \copyright GNU Public License
	 * \sa pcl::PointXYZ
	 */
  //Comment?
	struct RealObjectPoint {
		PCL_ADD_POINT4D;
		PCL_ADD_RGB;

		union {
			struct {
				float qw;
				float qx;
				float qy;
				float qz;
			};
		};

        std::string type;
        std::string identifier;
		SimpleVector3CollectionPtr normal_vectors;
		IndicesPtr active_normal_vectors;
        std_msgs::ColorRGBA color;
        double intermediate_object_weight;
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		RealObjectPoint(const gm::Pose &pose = gm::Pose()) : normal_vectors(new SimpleVector3Collection()), active_normal_vectors(new Indices()) {
			x = pose.position.x;
			y = pose.position.y;
			z = pose.position.z;

			qw = pose.orientation.w;
			qx = pose.orientation.x;
			qy = pose.orientation.y;
			qz = pose.orientation.z;
		}

		RealObjectPoint(const SimpleVector3 &vector) : qw(1.0), qx(0.0), qy(0.0), qz(0.0), normal_vectors(new SimpleVector3Collection()) {
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
	} EIGEN_ALIGN16;
    typedef boost::shared_ptr<RealObjectPoint> RealObjectPointPtr;

    std::ostream& operator<<(std::ostream &strm, const next_best_view::RealObjectPoint &p);
    std::ostream& operator<<(std::ostream &strm, const next_best_view::RealObjectPointPtr &p);
}

POINT_CLOUD_REGISTER_POINT_STRUCT (next_best_view::RealObjectPoint,
   (float, x, x)
   (float, y, y)
   (float, z, z)
)

#endif /* REALOBJECTPOINT_HPP_ */


