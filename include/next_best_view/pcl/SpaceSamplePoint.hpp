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
    /* Describes a point in the space in the point cloud.
     * Space Points stands for possible position of the robot to look at objects.
     */
	struct SpaceSamplePoint {
	public:
		PCL_ADD_POINT4D;

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
    typedef boost::shared_ptr<SpaceSamplePoint> SpaceSamplePointPtr;

    std::ostream& operator<<(std::ostream &strm, const next_best_view::SpaceSamplePoint &p);
    std::ostream& operator<<(std::ostream &strm, const next_best_view::SpaceSamplePointPtr &p);
}

POINT_CLOUD_REGISTER_POINT_STRUCT (next_best_view::SpaceSamplePoint,
   (float, x, x)
   (float, y, y)
   (float, z, z)
)


#endif /* SPACESAMPLEPOINT_HPP_ */
