/*
 * NextBestViewCalculator.hpp
 *
 *  Created on: Sep 25, 2014
 *      Author: ralfschleicher
 */

#ifndef NEXTBESTVIEWCALCULATOR_HPP_
#define NEXTBESTVIEWCALCULATOR_HPP_

#include "typedef.hpp"
#include <vector>

#include <boost/foreach.hpp>

#include "next_best_view/hypothesis_updater/HypothesisUpdater.hpp"
#include "next_best_view/robot_model/RobotModel.hpp"
#include "next_best_view/camera_model_filter/CameraModelFilter.hpp"
#include "next_best_view/unit_sphere_sampler/UnitSphereSampler.hpp"
#include "next_best_view/space_sampler/SpaceSampler.hpp"
#include "next_best_view/rating/RatingModule.hpp"
#include "next_best_view/rating/impl/DefaultScoreContainer.hpp"
#include "next_best_view/AttributedPointCloud.h"
#include "next_best_view/AttributedPoint.h"

namespace next_best_view {
	class NextBestViewCalculator {
	private:
		ObjectPointCloudPtr mPointCloudPtr;
		KdTreePtr mKdTreePtr;
		UnitSphereSamplerPtr mUnitSphereSamplerPtr;
		SpaceSamplerPtr mSpaceSamplerPtr;
		RobotModelPtr mRobotModelPtr;
		CameraModelFilterPtr mCameraModelFilterPtr;
		RatingModulePtr mRatingModulePtr;
		HypothesisUpdaterPtr mHypothesisUpdaterPtr;
		float mEpsilon;
	public:
		NextBestViewCalculator(const UnitSphereSamplerPtr & unitSphereSamplerPtr = UnitSphereSamplerPtr(),
				const SpaceSamplerPtr &spaceSamplerPtr = SpaceSamplerPtr(),
				const RobotModelPtr &robotModelPtr = RobotModelPtr(),
				const CameraModelFilterPtr &cameraModelFilterPtr = CameraModelFilterPtr(),
				const RatingModulePtr &ratingModulePtr = RatingModulePtr())
			: mUnitSphereSamplerPtr(unitSphereSamplerPtr),
			  mSpaceSamplerPtr(spaceSamplerPtr),
			  mRobotModelPtr(robotModelPtr),
			  mCameraModelFilterPtr(cameraModelFilterPtr),
			  mRatingModulePtr(),
			  mEpsilon(10E-3) {	}
	public:

		/**
		 * Calculates the next best view.
		 */
		bool calculateNextBestView(const ViewportPoint &currentCameraViewport, ViewportPoint &resultViewport) {
			ROS_INFO("Starting calculation of next best view");

			RobotStatePtr currentState = mRobotModelPtr->calculateRobotState(currentCameraViewport.getSimpleVector3(), currentCameraViewport.getSimpleQuaternion());
			mRobotModelPtr->setCurrentRobotState(currentState);

			// Get the orientations.
			ROS_INFO("Create and filter unit sphere");
			SimpleQuaternionCollectionPtr sampledOrientationsPtr = mUnitSphereSamplerPtr->getSampledUnitSphere();
			SimpleQuaternionCollectionPtr feasibleOrientationsCollectionPtr(new SimpleQuaternionCollection());
			BOOST_FOREACH(SimpleQuaternion q, *sampledOrientationsPtr) {
				if (mRobotModelPtr->isPoseReachable(SimpleVector3(0, 0, 0), q)) {
					feasibleOrientationsCollectionPtr->push_back(q);
				}
			}

			// create the next best view point cloud
			return this->doIteration(currentCameraViewport, feasibleOrientationsCollectionPtr, resultViewport);
		}

		void getFeasibleSamplePoints(const SamplePointCloudPtr &sampledSpacePointCloudPtr, IndicesPtr &resultIndicesPtr) {
			resultIndicesPtr = IndicesPtr(new Indices());
			// get max radius by far clipping plane of camera. this marks the limit for the visible object distance.
			double radius = mCameraModelFilterPtr->getFarClippingPlane();
			for(std::size_t index = 0; index < sampledSpacePointCloudPtr->size(); index++) {
				// get the point
				SamplePoint &spaceSamplePoint = sampledSpacePointCloudPtr->at(index);
				ObjectPoint comparablePoint(spaceSamplePoint.getSimpleVector3());

				// the resulting child indices will be written in here
				IndicesPtr childIndicesPtr(new Indices());
				// we don't need distances, but distances are written in here
				SquaredDistances dismissDistances;

				// this is a radius search - which reduces the complexity level for frustum culling.
				int k = mKdTreePtr->radiusSearch(comparablePoint, radius, *childIndicesPtr, dismissDistances);

				// if there is no result of neighboured points, no need to add this point.
				if (k == 0) {
					continue;
				}

				// set the indices
				spaceSamplePoint.child_indices = childIndicesPtr;
				spaceSamplePoint.child_point_cloud = mSpaceSamplerPtr->getInputCloud();

				// add the index to active indices.
				resultIndicesPtr->push_back(index);
			}
		}

	private:
		bool doIteration(const ViewportPoint &currentCameraViewport, const SimpleQuaternionCollectionPtr &sampledOrientationsPtr, ViewportPoint &resultViewport) {
			// reset the contractor
			float contractor = 1.0;

			ViewportPoint currentBestViewport = currentCameraViewport;
			while (ros::ok()) {
				ViewportPoint intermediateResultViewport;
				ROS_INFO("Prepare iteration step");
				if (!this->doIterationStep(currentCameraViewport, currentBestViewport, sampledOrientationsPtr, contractor, intermediateResultViewport)) {
					return false;
				}

				DefaultScoreContainerPtr drPtr = boost::static_pointer_cast<DefaultScoreContainer>(intermediateResultViewport.score);
				ROS_INFO("x: %f, y: %f, z: %f, ElementCount: %f, Normality: %f, Contractor: %f", intermediateResultViewport.x, intermediateResultViewport.y, intermediateResultViewport.z, drPtr->element_density, drPtr->normality, contractor);

				if (currentCameraViewport.getSimpleVector3() == intermediateResultViewport.getSimpleVector3() || (intermediateResultViewport.getSimpleVector3() - currentBestViewport.getSimpleVector3()).lpNorm<2>() <= this->getEpsilon()) {
					resultViewport = intermediateResultViewport;
					return true;
				}

				currentBestViewport = intermediateResultViewport;

				// smaller contractor.
				contractor /= 2.0;
			}

			return false;
		}

		bool doIterationStep(const ViewportPoint &currentCameraViewport, const ViewportPoint &currentBestViewport, const SimpleQuaternionCollectionPtr &sampledOrientationsPtr, float contractor, ViewportPoint &resultViewport) {
			// current camera position
			SimpleVector3 currentBestPosition = currentBestViewport.getSimpleVector3();

			// do a prefiltering for interesting space sample points
			SamplePointCloudPtr sampledSpacePointCloudPtr = mSpaceSamplerPtr->getSampledSpacePointCloud(currentBestPosition, contractor);

			IndicesPtr feasibleIndicesPtr;
			this->getFeasibleSamplePoints(sampledSpacePointCloudPtr, feasibleIndicesPtr);

			if (feasibleIndicesPtr->size() == 1 && this->getEpsilon() < contractor) {
				return doIterationStep(currentCameraViewport, currentBestViewport, sampledOrientationsPtr, contractor * .5, resultViewport);
			}

			ViewportPointCloudPtr nextBestViewports = ViewportPointCloudPtr(new ViewportPointCloud());
			// do the frustum culling by camera model filter
			BOOST_FOREACH(int activeIndex, *feasibleIndicesPtr) {
				SamplePoint &samplePoint = sampledSpacePointCloudPtr->at(activeIndex);


				BOOST_FOREACH(SimpleQuaternion orientation, *sampledOrientationsPtr) {
					ViewportPoint candidateViewportPoint;

					// set the input cloud
					this->doFrustumCulling(samplePoint.getSimpleVector3(), orientation, samplePoint.child_indices, candidateViewportPoint);

					if (candidateViewportPoint.child_indices->size() == 0) {
						continue;
					}

					// if the rating is not feasible - skip adding to point cloud
					if (!mRatingModulePtr->getScoreContainer(currentCameraViewport, candidateViewportPoint, candidateViewportPoint.score)) {
						continue;
					}

					RobotStatePtr targetRobotState = mRobotModelPtr->calculateRobotState(candidateViewportPoint.getSimpleVector3(), candidateViewportPoint.getSimpleQuaternion());
					candidateViewportPoint.score->costs = mRobotModelPtr->getMovementCosts(targetRobotState);

					// add the candidate viewport to the next best view point cloud.
					nextBestViewports->push_back(candidateViewportPoint);
				}
			}

			// if there aren't any viewports, the search failed.
			if (nextBestViewports->size() == 0) {
				return false;
			}

			ViewportPointCloud::iterator maxElement = std::max_element(nextBestViewports->begin(), nextBestViewports->end(), boost::bind(&NextBestViewCalculator::compareViewports, *this, _1, _2));
			resultViewport = *maxElement;

			return true;
		}
	public:
		bool doFrustumCulling(const SimpleVector3 &point, const SimpleQuaternion &orientation, const IndicesPtr &indices, ViewportPoint &viewportPoint) {
			mCameraModelFilterPtr->setIndices(indices);
			mCameraModelFilterPtr->setPivotPointPose(point, orientation);

			// do the frustum culling
			IndicesPtr frustumIndicesPtr(new Indices());
			mCameraModelFilterPtr->filter(frustumIndicesPtr);

			viewportPoint = ViewportPoint(point, orientation);
			viewportPoint.child_indices = frustumIndicesPtr;
			viewportPoint.child_point_cloud = mCameraModelFilterPtr->getInputCloud();

			return true;
		}

		void updateObjectPointCloud(const ViewportPoint &viewportPoint) {
			mHypothesisUpdaterPtr->update(viewportPoint);
		}
	private:
		/**
		 * Compares two viewports
		 * @param a - viewport a
		 * @param b - viewport b
		 * @return a < b
		 */
		bool compareViewports(ViewportPoint &a, ViewportPoint &b) {
			return mRatingModulePtr->compareScoreContainer(a.score, b.score);
		}
	public:

		/////
		///
		// GETTER AND SETTER
		///
		//////

		/**
		 * Sets the point cloud points from point cloud message
		 * @param message - message containing the point cloud
		 */
		void setPointCloudFromMessage(const AttributedPointCloud &msg) {
			// create a new point cloud
			ObjectPointCloudPtr pointCloudPtr = ObjectPointCloudPtr(new ObjectPointCloud());
			object_database::ObjectManager manager;

			// put each element into the point cloud
			BOOST_FOREACH(AttributedPoint element, msg.elements) {
				// Create a new point with pose and set object type
				ObjectPoint pointCloudPoint(element.pose);
				pointCloudPoint.r = 0;
				pointCloudPoint.g = 255;
				pointCloudPoint.b = 0;
				pointCloudPoint.object_type_name = element.object_type;

				// Get the rotation matrix to translate the normal vectors of the object.
				SimpleMatrix3 rotationMatrix = pointCloudPoint.getSimpleQuaternion().toRotationMatrix();

				// get object type information
				object_database::ObjectTypeResponsePtr responsePtr = manager.get(pointCloudPoint.object_type_name);

				// translating from std::vector<geometry_msgs::Point> to std::vector<SimpleVector3>
				int normalVectorCount = 0;
				BOOST_FOREACH(geometry_msgs::Point point, responsePtr->normal_vectors) {
					SimpleVector3 normal(point.x, point.y, point.z);
					normal = rotationMatrix * normal;
					pointCloudPoint.normal_vectors->push_back(normal);
					pointCloudPoint.active_normal_vectors->push_back(normalVectorCount++);
				}

				// add point to array
				pointCloudPtr->push_back(pointCloudPoint);
			}

			this->setPointCloudPtr(pointCloudPtr);
		}

		/**
		 * Sets the point cloud ptr.
		 */
		void setPointCloudPtr(const ObjectPointCloudPtr &pointCloudPtr) {
			mPointCloudPtr = pointCloudPtr;

			mKdTreePtr = KdTreePtr(new KdTree());
			mKdTreePtr->setInputCloud(mPointCloudPtr);
			mRatingModulePtr->setInputCloud(mPointCloudPtr);
			mCameraModelFilterPtr->setInputCloud(mPointCloudPtr);
			mSpaceSamplerPtr->setInputCloud(mPointCloudPtr);
		}

		/**
		 * @return the point cloud pointer
		 */
		ObjectPointCloudPtr getPointCloudPtr() {
			return mPointCloudPtr;
		}

		/**
		 * @return the point cloud pointer
		 */
		KdTreePtr getKdTreePtr() {
			return mKdTreePtr;
		}

		/**
		 * Sets the unit sphere sampler.
		 * @param unitSphereSamplerPtr - the pointer to the unit sphere sampler
		 */
		void setUnitSphereSampler(const UnitSphereSamplerPtr &unitSphereSamplerPtr) {
			mUnitSphereSamplerPtr = unitSphereSamplerPtr;
		}

		/**
		 * @return the unit sphere sampler.
		 */
		UnitSphereSamplerPtr getUnitSphereSampler() {
			return mUnitSphereSamplerPtr;
		}

		/**
		 * Sets the space sampler.
		 * @param spaceSamplerPtr - the pointer to the space sampler
		 */
		void setSpaceSampler(const SpaceSamplerPtr &spaceSamplerPtr) {
			mSpaceSamplerPtr = spaceSamplerPtr;
		}

		/**
		 * @return the pointer to the space sampler.
		 */
		SpaceSamplerPtr getSpaceSampler() {
			return mSpaceSamplerPtr;
		}

		/**
		 * sets the robot model
		 * @param robotModelPtr - the pointer to robot model.
		 */
		void setRobotModel(const RobotModelPtr &robotModelPtr) {
			mRobotModelPtr = robotModelPtr;
		}

		/**
		 *@return the robot ptr
		 */
		RobotModelPtr getRobotModel() {
			return mRobotModelPtr;
		}

		/**
		 * Sets the camera model filter.
		 * @param cameraModelFilterPtr - the pointer to the camera model filter.
		 */
		void setCameraModelFilter(const CameraModelFilterPtr &cameraModelFilterPtr) {
			mCameraModelFilterPtr = cameraModelFilterPtr;
		}

		/**
		 * @return the pointer to the camera model filter
		 */
		CameraModelFilterPtr getCameraModelFilter() {
			return mCameraModelFilterPtr;
		}

		/**
		 * Sets the rating module.
		 * @param ratingModulePtr - the pointer to the rating module
		 */
		void setRatingModule(const RatingModulePtr &ratingModulePtr) {
			mRatingModulePtr = ratingModulePtr;
		}

		/**
		 * @return the pointer to the rating module.
		 */
		RatingModulePtr getRatingModule() {
			return mRatingModulePtr;
		}

		void setHypothesisUpdater(const HypothesisUpdaterPtr &hypothesisUpdaterPtr) {
			mHypothesisUpdaterPtr = hypothesisUpdaterPtr;
		}

		HypothesisUpdaterPtr getHypothesisUpdater() {
			return mHypothesisUpdaterPtr;
		}

		/**
		 * @param epsilon - error threshold
		 */
		void setEpsilon(float epsilon) {
			mEpsilon = epsilon;
		}

		/**
		 * @return epsilon
		 */
		float getEpsilon() {
			return mEpsilon;
		}
	};
}


#endif /* NEXTBESTVIEWCALCULATOR_HPP_ */
