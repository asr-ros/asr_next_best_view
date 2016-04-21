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
         * \brief prints the viewport point as debug output
         * \param rating of this viewport
         * \param level the debug level of the output
         */
        void print(float rating, DebugHelper::DebugLevel level) {
            // viewport position
            mDebugHelperPtr->write(std::stringstream() << "Viewport position: (x = " << x << ", y = " << y << ", z = " << z << ")",
                        level);
            // viewport orientation
            mDebugHelperPtr->write(std::stringstream() << "Viewport orienation: (qw = " << qw << ", qx = " << qx << ", qy = " << qy << ", qz = " << qz << ")",
                        level);
            // viewport object types
            std::string types = "";
            for (ObjectTypeSet::iterator objectIter = object_type_set->begin(); objectIter != object_type_set->end(); objectIter++) {
                if (types.size() > 0) {
                    types += ", ";
                }
                types += *objectIter;
            }
            mDebugHelperPtr->write(std::stringstream() << "Viewport object types: " << types, level);
            // viewport utility and costs
            mDebugHelperPtr->write(std::stringstream() << "Viewport weighted normalized utility: " << score->getWeightedNormalizedUtility()
                                    << " weighted inverse costs: " << score->getWeightedInverseCosts(),
                        level);

            BOOST_FOREACH(std::string objectType, *object_type_set) {
                mDebugHelperPtr->write(std::stringstream() << "Viewport unweighted normalized utility for " << objectType << ": "
                                    << score->getUnweightedNormalizedObjectUtility(objectType),
                        level);
            }

            mDebugHelperPtr->write(std::stringstream() << "Viewport utility normalization: " << score->getUtilityNormalization(),
                        level);
            mDebugHelperPtr->write(std::stringstream() << "Viewport unweighted inverse costs base translation: " << score->getUnweightedInverseMovementCostsBaseTranslation()
                                    << " unweighted inverse costs base rotation: " << score->getUnweightedInverseMovementCostsBaseRotation(),
                        level);
            mDebugHelperPtr->write(std::stringstream() << "Viewport unweighted inverse costs PTU movement: " << score->getUnweightedInverseMovementCostsPTU()
                                   << " unweighted inverse costs recognition: " << score->getUnweightedInverseRecognitionCosts(),
                        level);
            // viewport rating
            mDebugHelperPtr->write(std::stringstream() << "Viewport rating: " << rating,
                        level);
            mDebugHelperPtr->write("", level);
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
}
//Comment?
POINT_CLOUD_REGISTER_POINT_STRUCT (next_best_view::ViewportPoint,
   (float, x, x)
   (float, y, y)
   (float, z, z)
)


#endif /* DEFAULTVIEWPORTPOINT_HPP_ */
