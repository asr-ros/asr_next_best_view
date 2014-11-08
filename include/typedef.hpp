/*
 * typedef.hpp
 *
 *  Created on: Aug 9, 2014
 *      Author: ralfschleicher
 */

#ifndef TYPEDEF_HPP_
#define TYPEDEF_HPP_

#include <vector>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/frustum_culling.h>

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

	// forward declaration for real object point
	struct RealObjectPoint;
	struct SpaceSamplePoint;
	struct DefaultViewportPoint;

	// create aliases for actually used point types
	typedef RealObjectPoint ObjectPoint;
	typedef SpaceSamplePoint SamplePoint;
	typedef DefaultViewportPoint ViewportPoint;

	// defining different point cloud types
	typedef pcl::PointCloud<ObjectPoint> ObjectPointCloud;
	typedef ObjectPointCloud::Ptr ObjectPointCloudPtr;
	typedef ObjectPointCloud::ConstPtr ObjectPointCloudConstPtr;

	typedef pcl::PointCloud<SamplePoint> SamplePointCloud;
	typedef SamplePointCloud::Ptr SamplePointCloudPtr;
	typedef SamplePointCloud::ConstPtr SamplePointCloudConstPtr;

	typedef pcl::PointCloud<ViewportPoint> ViewportPointCloud;
	typedef ViewportPointCloud::Ptr ViewportPointCloudPtr;
	typedef ViewportPointCloud::ConstPtr ViewportPointCloudConstPtr;

	// defining the frustum culling
	typedef pcl::FrustumCulling<ObjectPoint> FrustumCulling;
	typedef FrustumCulling::Ptr FrustumCullingPtr;

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
}

#include "next_best_view/pcl/RealObjectPoint.hpp"
#include "next_best_view/pcl/SpaceSamplePoint.hpp"
#include "next_best_view/pcl/DefaultViewportPoint.hpp"

#endif /* TYPEDEF_HPP_ */
