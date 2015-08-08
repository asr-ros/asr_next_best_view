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
		ObjectNameSetPtr mObjectNameSetPtr;
		boost::shared_ptr<std::set<ObjectNameSetPtr> > mSingleObjectNameSubPowerSetPtr;
		boost::shared_ptr<std::set<ObjectNameSetPtr> > mRemainderObjectNameSubPowerSetPtr;
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
			ROS_DEBUG("Starting calculation of next best view");

			RobotStatePtr currentState = mRobotModelPtr->calculateRobotState(currentCameraViewport.getSimpleVector3(), currentCameraViewport.getSimpleQuaternion());
			mRobotModelPtr->setCurrentRobotState(currentState);

			// Get the orientations.
			ROS_DEBUG("Create and filter unit sphere");
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
				ROS_DEBUG("Prepare iteration step");
				if (!this->doIterationStep(currentCameraViewport, currentBestViewport, sampledOrientationsPtr, contractor, intermediateResultViewport)) {
					return false;
				}

				DefaultScoreContainerPtr drPtr = boost::static_pointer_cast<DefaultScoreContainer>(intermediateResultViewport.score);
				ROS_DEBUG("x: %f, y: %f, z: %f, ElementCount: %f, Normality: %f, Utility: %f, Costs: %f, Contractor: %f", intermediateResultViewport.x, intermediateResultViewport.y, intermediateResultViewport.z, drPtr->getElementDensity(), drPtr->getNormality(), drPtr->getUtility(), drPtr->getCosts(), contractor);

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
			std::map<std::string, ViewportPoint> objectNameViewportMapping;

			// do the frustum culling by camera model filter
			BOOST_FOREACH(int activeIndex, *feasibleIndicesPtr) {
				SamplePoint &samplePoint = sampledSpacePointCloudPtr->at(activeIndex);
				SimpleVector3 samplePointCoords = samplePoint.getSimpleVector3();
				IndicesPtr samplePointChildIndices = samplePoint.child_indices;

				BOOST_FOREACH(SimpleQuaternion orientation, *sampledOrientationsPtr) {
					objectNameViewportMapping.clear();

					ViewportPoint fullViewportPoint;
					// set the input cloud
					if (!this->doFrustumCulling(samplePointCoords, orientation, samplePointChildIndices, fullViewportPoint)) {
						continue;
					}

					RobotStatePtr targetRobotState = mRobotModelPtr->calculateRobotState(fullViewportPoint.getSimpleVector3(), fullViewportPoint.getSimpleQuaternion());
					float movementCosts = mRobotModelPtr->getMovementCosts(targetRobotState);

					// do the filtering for the the single object types
					for (std::set<ObjectNameSetPtr>::iterator subSetIter = mSingleObjectNameSubPowerSetPtr->begin(); subSetIter != mSingleObjectNameSubPowerSetPtr->end(); ++subSetIter) {
						ViewportPoint candidateViewportPoint;
						if (!this->doObjectNameFiltering(*subSetIter, fullViewportPoint, candidateViewportPoint)) {
							continue;
						}

						// if the rating is not feasible - skip adding to point cloud
						if (!mRatingModulePtr->getScoreContainer(currentCameraViewport, candidateViewportPoint, candidateViewportPoint.score)) {
							continue;
						}

						// assign movement costs for the given viewport.
						candidateViewportPoint.score->setCosts(movementCosts);

						std::string objectName = *((*subSetIter)->begin());
						objectNameViewportMapping [objectName] = candidateViewportPoint;
						nextBestViewports->push_back(candidateViewportPoint);
					}

					// now aggregating all possible combinations.
					for (std::set<ObjectNameSetPtr>::iterator subSetIter = mRemainderObjectNameSubPowerSetPtr->begin(); subSetIter != mRemainderObjectNameSubPowerSetPtr->end(); ++subSetIter) {
						ObjectNameSetPtr subSet = *subSetIter;

						bool valid = true;
						Precision utility = 0;
						IndicesPtr aggregatedIndicesPtr(new Indices());
						for (ObjectNameSet::iterator itemIter = subSet->begin(); itemIter != subSet->end(); ++itemIter) {
							std::string objectName = *itemIter;
							if (objectNameViewportMapping.find(objectName) == objectNameViewportMapping.end()) {
								valid = false;
								break;
							}

							ViewportPoint singleObjectViewportPoint = objectNameViewportMapping [objectName];
							utility += singleObjectViewportPoint.score->getUtility();
							std::size_t oldSize = aggregatedIndicesPtr->size();
							aggregatedIndicesPtr->resize(oldSize + singleObjectViewportPoint.child_indices->size(), 0);
							std::copy(singleObjectViewportPoint.child_indices->begin(), singleObjectViewportPoint.child_indices->end(), aggregatedIndicesPtr->begin() + oldSize);
						}

						if (!valid) {
							continue;
						}

						ViewportPoint aggregatedViewportPoint = ViewportPoint(fullViewportPoint.getSimpleVector3(), fullViewportPoint.getSimpleQuaternion());
						aggregatedViewportPoint.child_point_cloud = fullViewportPoint.child_point_cloud;
						aggregatedViewportPoint.child_indices = aggregatedIndicesPtr;
						aggregatedViewportPoint.object_name_set = subSet;
						aggregatedViewportPoint.score = mRatingModulePtr->getScoreContainerInstance();
						aggregatedViewportPoint.score->setUtility(utility);
						aggregatedViewportPoint.score->setCosts(movementCosts);

						nextBestViewports->push_back(aggregatedViewportPoint);
					}
				}
			}

			// if there aren't any viewports, the search failed.
			if (nextBestViewports->size() == 0) {
				return false;
			}

			ViewportPointCloud::iterator maxElement = std::max_element(nextBestViewports->begin(), nextBestViewports->end(), boost::bind(&NextBestViewCalculator::compareViewports, *this, _1, _2));

			if (maxElement == nextBestViewports->end()) {
				return false;
			}

			resultViewport = *maxElement;

			return true;
		}
	public:
		bool doFrustumCulling(const SimpleVector3 &point, const SimpleQuaternion &orientation, const IndicesPtr &indices, ViewportPoint &viewportPoint) {
			mCameraModelFilterPtr->setIndices(indices);
			mCameraModelFilterPtr->setPivotPointPose(point, orientation);

			// do the frustum culling
			IndicesPtr frustumIndicesPtr;
			mCameraModelFilterPtr->filter(frustumIndicesPtr);

			if (frustumIndicesPtr->size() == 0) {
				return false;
			}

			viewportPoint = ViewportPoint(point, orientation);
			viewportPoint.child_indices = frustumIndicesPtr;
			viewportPoint.child_point_cloud = mCameraModelFilterPtr->getInputCloud();
			viewportPoint.object_name_set = mObjectNameSetPtr;

			return true;
		}

		bool doObjectNameFiltering(const ObjectNameSetPtr &objectNameSetPtr, const ViewportPoint &fullViewportPoint, ViewportPoint &viewportPoint) {
			IndicesPtr objectNameIndicesPtr(new Indices());
			BOOST_FOREACH(std::size_t index, *fullViewportPoint.child_indices) {
				ObjectPoint &point = mPointCloudPtr->at(index);

				// check if object type is in ObjectNameSet
				ObjectNameSet::iterator iter = std::find(objectNameSetPtr->begin(), objectNameSetPtr->end(), point.object_type_name);
				if (iter != objectNameSetPtr->end()) {
					objectNameIndicesPtr->push_back(index);
				}
			}

			if (objectNameIndicesPtr->size() == 0) {
				return false;
			}

			viewportPoint = ViewportPoint(fullViewportPoint.getSimpleVector3(), fullViewportPoint.getSimpleQuaternion());
			viewportPoint.child_indices = objectNameIndicesPtr;
			viewportPoint.child_point_cloud = fullViewportPoint.child_point_cloud;
			viewportPoint.object_name_set = objectNameSetPtr;

			return true;
		}

		void updateFromExternalObjectPointList(const std::vector<ViewportPoint> &viewport_point_list) {

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

			// empty object name set
			mObjectNameSetPtr = ObjectNameSetPtr(new ObjectNameSet);

			// put each element into the point cloud
			BOOST_FOREACH(AttributedPoint element, msg.elements) {
				// Create a new point with pose and set object type
				ObjectPoint pointCloudPoint(element.pose);
				pointCloudPoint.r = 0;
				pointCloudPoint.g = 255;
				pointCloudPoint.b = 0;
				pointCloudPoint.object_type_name = element.object_type;
				mObjectNameSetPtr->insert(element.object_type);

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

			// get the power set of the object name set.
			ObjectNamePowerSetPtr powerSetPtr = MathHelper::powerSet<ObjectNameSet> (mObjectNameSetPtr);
			mSingleObjectNameSubPowerSetPtr = MathHelper::filterCardinalityPowerSet<ObjectNamePowerSet> (powerSetPtr, 1, 1);
			mRemainderObjectNameSubPowerSetPtr = MathHelper::filterCardinalityPowerSet<ObjectNamePowerSet> (powerSetPtr, 2);

			// set the point cloud
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
