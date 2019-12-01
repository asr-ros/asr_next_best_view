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

#ifndef TYPEDEF_HPP_
#define TYPEDEF_HPP_

#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/crop_box.h>
#include <set>
#include <vector>

/*!
* \brief this namespace contains all generally usable classes.
*/
namespace next_best_view {
	// the used precision in all calculations
	typedef float Precision;

	// matrix types
	typedef Eigen::Matrix<Precision, 2, 2> SimpleMatrix2;
	typedef Eigen::Matrix<Precision, 3, 3> SimpleMatrix3;
	typedef Eigen::Matrix<Precision, 4, 4> SimpleMatrix4;
	typedef Eigen::Matrix<Precision, Eigen::Dynamic, Eigen::Dynamic> SimpleMatrixX;

	// matrix list types
	typedef std::vector<SimpleMatrix3, Eigen::aligned_allocator<SimpleMatrix3> > SimpleMatrix3Collection;
	typedef boost::shared_ptr<SimpleMatrix3Collection> SimpleMatrix3CollectionPtr;

	typedef std::vector<SimpleMatrix4, Eigen::aligned_allocator<SimpleMatrix4> > SimpleMatrix4Collection;
	typedef boost::shared_ptr<SimpleMatrix4Collection> SimpleMatrix4CollectionPtr;

	// vector types
	typedef Eigen::Matrix<Precision, 2, 1> SimpleVector2;
	typedef Eigen::Matrix<Precision, 3, 1> SimpleVector3;
	typedef SimpleVector3 SimpleSphereCoordinates;
	typedef Eigen::Matrix<Precision, 4, 1> SimpleVector4;
	typedef Eigen::Matrix<Precision, Eigen::Dynamic, 1> SimpleVectorX;
    typedef Eigen::Matrix<int, 3, 1> GridVector3;

	// vector list types
	typedef std::vector<SimpleVector3, Eigen::aligned_allocator<SimpleVector3> > SimpleVector3Collection;
	typedef boost::shared_ptr<SimpleVector3Collection> SimpleVector3CollectionPtr;

	typedef std::vector<SimpleVector4, Eigen::aligned_allocator<SimpleVector4> > SimpleVector4Collection;
	typedef boost::shared_ptr<SimpleVector4Collection> SimpleVector4CollectionPtr;

	// quaternion type
	typedef Eigen::Quaternion<Precision> SimpleQuaternion;

	// quaternion list types
	typedef std::vector<SimpleQuaternion, Eigen::aligned_allocator<SimpleQuaternion> > SimpleQuaternionCollection;
	typedef boost::shared_ptr<SimpleQuaternionCollection> SimpleQuaternionCollectionPtr;

	// forward declaration for point types
	struct RealObjectPoint;
	struct SpaceSamplePoint;
	struct DefaultViewportPoint;

	// create aliases for actually used point types
	typedef RealObjectPoint ObjectPoint;
	typedef SpaceSamplePoint SamplePoint;
	typedef DefaultViewportPoint ViewportPoint;
    typedef pcl::PointXYZ WorldPoint;

	// defining different point cloud types
	typedef pcl::PointCloud<ObjectPoint> ObjectPointCloud;
	typedef ObjectPointCloud::Ptr ObjectPointCloudPtr;
	typedef ObjectPointCloud::ConstPtr ObjectPointCloudConstPtr;
    typedef pcl::PointCloud<WorldPoint> WorldPointCloud;
    typedef WorldPointCloud::Ptr WorldPointCloudPtr;
    typedef WorldPointCloud::ConstPtr WorldPointCloudConstPtr;

	typedef pcl::PointCloud<SamplePoint> SamplePointCloud;
	typedef SamplePointCloud::Ptr SamplePointCloudPtr;
	typedef SamplePointCloud::ConstPtr SamplePointCloudConstPtr;

	typedef pcl::PointCloud<ViewportPoint> ViewportPointCloud;
	typedef ViewportPointCloud::Ptr ViewportPointCloudPtr;
	typedef ViewportPointCloud::ConstPtr ViewportPointCloudConstPtr;

	// defining the frustum culling
	typedef pcl::FrustumCulling<ObjectPoint> FrustumCulling;
	typedef FrustumCulling::Ptr FrustumCullingPtr;

    // defining the cropbox filtering
    typedef pcl::CropBox<ObjectPoint> CropBox;
    typedef CropBox::Ptr CropBoxPtr;

	// defining kdtree types
	typedef pcl::KdTreeFLANN<ObjectPoint> KdTree;
	typedef KdTree::Ptr KdTreePtr;
	typedef KdTree::ConstPtr KdTreeConstPtr;

	// defining squared distances type used in kdtree
	typedef std::vector<float> SquaredDistances;

	// defining indices types
	typedef std::vector<int> Indices;
	typedef boost::shared_ptr<Indices> IndicesPtr;
	typedef boost::shared_ptr<const IndicesPtr> IndicesConstPtr;

    // the list of object types
    typedef std::vector<std::string> ObjectTypeList;
    typedef boost::shared_ptr<ObjectTypeList> ObjectTypeListPtr;
    typedef std::set<std::string> ObjectTypeSet;
    typedef boost::shared_ptr<ObjectTypeSet> ObjectTypeSetPtr;
    typedef std::set<ObjectTypeSetPtr> ObjectTypePowerSet;
    typedef boost::shared_ptr<ObjectTypePowerSet> ObjectTypePowerSetPtr;

    // visualization
    typedef SimpleVector4 Color;

}

#include "next_best_view/pcl/RealObjectPoint.hpp"
#include "next_best_view/pcl/SpaceSamplePoint.hpp"
#include "next_best_view/pcl/DefaultViewportPoint.hpp"

#endif /* TYPEDEF_HPP_ */
