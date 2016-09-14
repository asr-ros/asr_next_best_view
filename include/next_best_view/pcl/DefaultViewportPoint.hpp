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

#ifndef DEFAULTVIEWPORTPOINT_HPP_
#define DEFAULTVIEWPORTPOINT_HPP_

#define PCL_NO_PRECOMPILE
#include "typedef.hpp"
#include <boost/foreach.hpp>
#include "next_best_view/rating/impl/DefaultScoreContainer.hpp"
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
#include "next_best_view/helper/DebugHelper.hpp"

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

        DebugHelperPtr mDebugHelperPtr;

        // point cloud containing the object points in the viewport
		ObjectPointCloudPtr child_point_cloud;
        // the whole point cloud
        ObjectPointCloudPtr point_cloud;
        // indices of the object points in the viewport
        IndicesPtr child_indices;
        ObjectTypeSetPtr object_type_set;
        DefaultScoreContainerPtr score;
        // old idx in the saved datastructure to reorder
        unsigned int oldIdx;
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        DefaultViewportPoint(const gm::Pose &pose = gm::Pose()) : DefaultViewportPoint(SimpleVector3(pose.position.x, pose.position.y, pose.position.z)) {
            child_indices = IndicesPtr(new Indices());

			qw = pose.orientation.w;
			qx = pose.orientation.x;
			qy = pose.orientation.y;
			qz = pose.orientation.z;
		}

		DefaultViewportPoint(const SimpleVector3 &vector, const SimpleQuaternion &orientation = SimpleQuaternion(1.0, 0.0, 0.0, 0.0)) : qw(orientation.w()), qx(orientation.x()), qy(orientation.y()), qz(orientation.z()) {
			x = vector[0];
			y = vector[1];
			z = vector[2];

            mDebugHelperPtr = DebugHelper::getInstance();
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

        /*!
         * \brief filters all the objects in this viewport with one of the given types and puts them in a new viewport.
         * \param objectTypeSetPtr [in] the object type names that shall be put in the new viewport
         * \param viewportPoint [out] the new viewport only containing the objects with the given types
         * \return whether the result is valid
         */
        bool filterObjectTypes(const ObjectTypeSetPtr &objectTypeSetPtr, ViewportPoint &viewportPoint) {
            IndicesPtr objectTypeIndicesPtr(new Indices());
            BOOST_FOREACH(std::size_t index, *(this->child_indices)) {
                ObjectPoint &point = point_cloud->at(index);

                // check if object type is in objectTypeSet
                ObjectTypeSet::iterator iter = std::find(objectTypeSetPtr->begin(), objectTypeSetPtr->end(), point.type);
                if (iter != objectTypeSetPtr->end()) {
                    objectTypeIndicesPtr->push_back(index);
                }
            }

            if (objectTypeIndicesPtr->size() == 0) {
                return false;
            }

            viewportPoint = ViewportPoint(this->getPosition(), this->getSimpleQuaternion());
            viewportPoint.child_indices = objectTypeIndicesPtr;
            viewportPoint.child_point_cloud = this->child_point_cloud;
            viewportPoint.point_cloud = this->point_cloud;
            viewportPoint.object_type_set = objectTypeSetPtr;

            return true;
        }
	} EIGEN_ALIGN16;
    typedef boost::shared_ptr<DefaultViewportPoint> DefaultViewportPointPtr;

    std::ostream& operator<<(std::ostream &strm, const next_best_view::DefaultViewportPoint &p);
    std::ostream& operator<<(std::ostream &strm, const next_best_view::DefaultViewportPointPtr &p);
}
//Comment?
POINT_CLOUD_REGISTER_POINT_STRUCT (next_best_view::ViewportPoint,
   (float, x, x)
   (float, y, y)
   (float, z, z)
)


#endif /* DEFAULTVIEWPORTPOINT_HPP_ */
