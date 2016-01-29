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
#include <map>

#include <boost/foreach.hpp>
#include <boost/range/algorithm_ext/iota.hpp>
#include <boost/lexical_cast.hpp>

#include "next_best_view/hypothesis_updater/HypothesisUpdater.hpp"
#include "next_best_view/robot_model/RobotModel.hpp"
#include "next_best_view/camera_model_filter/CameraModelFilter.hpp"
#include "next_best_view/unit_sphere_sampler/UnitSphereSampler.hpp"
#include "next_best_view/space_sampler/SpaceSampler.hpp"
#include "next_best_view/rating/RatingModule.hpp"
#include "next_best_view/rating/impl/DefaultScoreContainer.hpp"
#include "pbd_msgs/PbdAttributedPointCloud.h"
#include "pbd_msgs/PbdAttributedPoint.h"
#include "next_best_view/helper/MapHelper.hpp"
#include "helper/VisualizationsHelper.hpp"

namespace next_best_view {
    class NextBestViewCalculator {
	private:
		ObjectPointCloudPtr mPointCloudPtr;
		IndicesPtr mActiveIndicesPtr;
		KdTreePtr mKdTreePtr;
        std::map<std::string, std::string> objectsResources;
		UnitSphereSamplerPtr mUnitSphereSamplerPtr;
        SpaceSamplerPtr mSpaceSamplerPtr;
		RobotModelPtr mRobotModelPtr;
		CameraModelFilterPtr mCameraModelFilterPtr;
		RatingModulePtr mRatingModulePtr;
		HypothesisUpdaterPtr mHypothesisUpdaterPtr;
		float mEpsilon;
		ObjectNameSetPtr mObjectNameSetPtr;
        VisualizationHelper mVisHelper;

	public:
		NextBestViewCalculator(const UnitSphereSamplerPtr & unitSphereSamplerPtr = UnitSphereSamplerPtr(),
				const SpaceSamplerPtr &spaceSamplerPtr = SpaceSamplerPtr(),
				const RobotModelPtr &robotModelPtr = RobotModelPtr(),
				const CameraModelFilterPtr &cameraModelFilterPtr = CameraModelFilterPtr(),
                const RatingModulePtr &ratingModulePtr = RatingModulePtr())
            : objectsResources(),
              mUnitSphereSamplerPtr(unitSphereSamplerPtr),
			  mSpaceSamplerPtr(spaceSamplerPtr),
			  mRobotModelPtr(robotModelPtr),
			  mCameraModelFilterPtr(cameraModelFilterPtr),
              mRatingModulePtr(ratingModulePtr),
              mEpsilon(10E-3),
              mVisHelper(){}
	public:

        /**
		 * Calculates the next best view.
		 */
		bool calculateNextBestView(const ViewportPoint &currentCameraViewport, ViewportPoint &resultViewport) {
			ROS_DEBUG("Starting calculation of next best view");

			RobotStatePtr currentState = mRobotModelPtr->calculateRobotState(currentCameraViewport.getPosition(), currentCameraViewport.getSimpleQuaternion());
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

            int iterationStep = 0;

			ViewportPoint currentBestViewport = currentCameraViewport;
			while (ros::ok()) {
				ViewportPoint intermediateResultViewport;
				ROS_DEBUG("Prepare iteration step");

                if (!this->doIterationStep(currentCameraViewport, currentBestViewport,
                                           sampledOrientationsPtr, 1.0 / pow(2.0, iterationStep),
                                           intermediateResultViewport)) {
					return false;
				}

                SimpleVector3 intermediateResultPosition =intermediateResultViewport.getPosition();

                SpaceSamplerPtr spaceSamplerPtr = this->getSpaceSampler();
                SamplePointCloudPtr pointcloud =
                        spaceSamplerPtr->getSampledSpacePointCloud(intermediateResultPosition, 1.0/pow(2.0,iterationStep));
                IndicesPtr feasibleIndices(new Indices());
                this->getFeasibleSamplePoints(pointcloud, feasibleIndices);

                mVisHelper.triggerIterationVisualizations(iterationStep, intermediateResultPosition, sampledOrientationsPtr,
                                      intermediateResultViewport, feasibleIndices, pointcloud, spaceSamplerPtr);

                DefaultScoreContainerPtr drPtr = intermediateResultViewport.score;
                ROS_DEBUG("x: %f, y: %f, z: %f, Utility: %f, Costs: %f, Translation costs: %f, Rotation costs: %f, PTU movement costs: %f, Recognition costs: %f, IterationStep: %i",
                            intermediateResultViewport.x, intermediateResultViewport.y, intermediateResultViewport.z,
                            drPtr->getUtility(), drPtr->getInverseCosts(),
                            drPtr->getInverseMovementCostsBaseTranslation(), drPtr->getInverseMovementCostsBaseRotation(),
                            drPtr->getInverseMovementCostsPTU(), drPtr->getInverseRecognitionCosts(),
                            iterationStep);

                if (currentCameraViewport.getPosition() == intermediateResultViewport.getPosition() ||
                        (intermediateResultViewport.getPosition() - currentBestViewport.getPosition()).lpNorm<2>() <= this->getEpsilon()) {
					resultViewport = intermediateResultViewport;
                    ROS_INFO_STREAM ("Suceeded. Took " << iterationStep << " iterations");
					return true;
				}

				currentBestViewport = intermediateResultViewport;

                iterationStep ++;
			}

			return false;
		}


        bool doIterationStep(const ViewportPoint &currentCameraViewport, const ViewportPoint &currentBestViewport,
                                const SimpleQuaternionCollectionPtr &sampledOrientationsPtr, float contractor,
                                ViewportPoint &resultViewport) {
			// current camera position
			SimpleVector3 currentBestPosition = currentBestViewport.getPosition();

			//Calculate hex grid for resolution given in this iteration step.
			SamplePointCloudPtr sampledSpacePointCloudPtr = mSpaceSamplerPtr->getSampledSpacePointCloud(currentBestPosition, contractor);

			// do a prefiltering for interesting space sample points
			IndicesPtr feasibleIndicesPtr;
			this->getFeasibleSamplePoints(sampledSpacePointCloudPtr, feasibleIndicesPtr);

			if (feasibleIndicesPtr->size() == 1 && this->getEpsilon() < contractor) {
				return doIterationStep(currentCameraViewport, currentBestViewport, sampledOrientationsPtr, contractor * .5, resultViewport);
			}

			//Create list of all view ports that are checked during this iteration step.
			ViewportPointCloudPtr nextBestViewports = ViewportPointCloudPtr(new ViewportPointCloud());

            //Go through all interesting space sample points for one iteration step to create candidate viewports.
			BOOST_FOREACH(int activeIndex, *feasibleIndicesPtr) {
				SamplePoint &samplePoint = sampledSpacePointCloudPtr->at(activeIndex);
				SimpleVector3 samplePointCoords = samplePoint.getSimpleVector3();
				IndicesPtr samplePointChildIndices = samplePoint.child_indices;

				//For each space sample point: Go through all interesting orientations.
                BOOST_FOREACH(SimpleQuaternion orientation, *sampledOrientationsPtr) {
                    // get the corresponding viewport
					ViewportPoint fullViewportPoint;
                    if (!this->doFrustumCulling(samplePointCoords, orientation, samplePointChildIndices, fullViewportPoint)) {
						continue;
					}

                    // get the best combination of objects to search for and the corresponding score
                    if (!mRatingModulePtr->setBestScoreContainer(currentCameraViewport, fullViewportPoint)) {
                        continue;
                    }

                    nextBestViewports->push_back(fullViewportPoint);
				}
			}

            if (!mRatingModulePtr->getBestViewport(nextBestViewports, resultViewport)) {
                return false;
            }

			return true;
		}
	public:
        /*!
         * \brief creates a new camera viewport with the given data
         * \param position [in] the position of the camera
         * \param orientation [in] the orientation of the camera
         * \param indices [in] the object point indices to be used
         * \param viewportPoint [out] the resulting camera viewport
         * \return
         */
        bool doFrustumCulling(const SimpleVector3 &position, const SimpleQuaternion &orientation, const IndicesPtr &indices, ViewportPoint &viewportPoint) {
			mCameraModelFilterPtr->setIndices(indices);
            mCameraModelFilterPtr->setPivotPointPose(position, orientation);

			// do the frustum culling
			IndicesPtr frustumIndicesPtr;
			mCameraModelFilterPtr->filter(frustumIndicesPtr);

			if (frustumIndicesPtr->size() == 0) {
				return false;
			}

            viewportPoint = ViewportPoint(position, orientation);
			viewportPoint.child_indices = frustumIndicesPtr;
			viewportPoint.child_point_cloud = mCameraModelFilterPtr->getInputCloud();
            viewportPoint.point_cloud = mPointCloudPtr;
			viewportPoint.object_type_name_set = mObjectNameSetPtr;

			return true;
		}

		void updateFromExternalObjectPointList(const std::vector<ViewportPoint> &viewportPointList) {
			BOOST_FOREACH(ViewportPoint viewportPoint, viewportPointList) {
				ViewportPoint culledViewportPoint;
                if (!this->doFrustumCulling(viewportPoint.getPosition(), viewportPoint.getSimpleQuaternion(), this->getActiveIndices(), culledViewportPoint)) {
                    ROS_DEBUG_STREAM("Viewpoint SKIPPED by Culling: " << viewportPoint.getPosition());
					continue;
				}

				ViewportPoint resultingViewportPoint;
                if (!culledViewportPoint.filterObjectNames(viewportPoint.object_type_name_set, resultingViewportPoint)) {
                    ROS_DEBUG_STREAM("Viewpoint SKIPPED by NameFiltering: " << viewportPoint.getPosition());
					continue;
				}

                ROS_DEBUG_STREAM("Viewpoint TAKEN: " << resultingViewportPoint.getPosition());
                for (std::set<std::string>::iterator it=resultingViewportPoint.object_type_name_set->begin(); it!=resultingViewportPoint.object_type_name_set->end(); ++it)
                {
                    ROS_DEBUG_STREAM("Object: " << *it);
                }
				this->updateObjectPointCloud(resultingViewportPoint);
                break;
			}
		}

        /*!
         * \brief Updates the point cloud under the assumption that the given viewport was chosen.
         * \param viewportPoint the viewport that was chosen
         * \return the number of deactivated normals
         */
        unsigned int updateObjectPointCloud(const ViewportPoint &viewportPoint) {
            return mHypothesisUpdaterPtr->update(viewportPoint);
        }

        /////
		///
		// GETTER AND SETTER
		///
		//////

		/**
		 * Sets the point cloud points from point cloud message
		 * @param message - message containing the point cloud
		 */
      bool setPointCloudFromMessage(const pbd_msgs::PbdAttributedPointCloud &msg) {
			// create a new point cloud
			ObjectPointCloudPtr pointCloudPtr = ObjectPointCloudPtr(new ObjectPointCloud());

			object_database::ObjectManager manager;

			// empty object name set
			mObjectNameSetPtr = ObjectNameSetPtr(new ObjectNameSet);

			// put each element into the point cloud
			BOOST_FOREACH(pbd_msgs::PbdAttributedPoint element, msg.elements) {
				// Create a new point with pose and set object type
				ObjectPoint pointCloudPoint(element.pose);
				pointCloudPoint.r = 0;
				pointCloudPoint.g = 255;
				pointCloudPoint.b = 0;
                pointCloudPoint.type = element.type;

                // add type name to list if not already inserted
                if (mObjectNameSetPtr->find(element.type) == mObjectNameSetPtr->end())
                    mObjectNameSetPtr->insert(element.type);

				// Get the rotation matrix to translate the normal vectors of the object.
				SimpleMatrix3 rotationMatrix = pointCloudPoint.getSimpleQuaternion().toRotationMatrix();

				// get object type information
				object_database::ObjectTypeResponsePtr responsePtr = manager.get(pointCloudPoint.type);

				if (responsePtr) {
					// translating from std::vector<geometry_msgs::Point> to std::vector<SimpleVector3>
					int normalVectorCount = 0;
					BOOST_FOREACH(geometry_msgs::Point point, responsePtr->normal_vectors) {
						SimpleVector3 normal(point.x, point.y, point.z);
                        normal = rotationMatrix * normal;
						pointCloudPoint.normal_vectors->push_back(normal);
						pointCloudPoint.active_normal_vectors->push_back(normalVectorCount);
						++normalVectorCount;
					}
				} else {
					ROS_ERROR("Invalid object name '%s' in point cloud or object_database node not started. Point Cloud not set!", pointCloudPoint.type.c_str());
					return false;
				}

                //Insert the meshpath
                objectsResources[pointCloudPoint.type] = responsePtr->object_mesh_resource;

                //Insert color
                std_msgs::ColorRGBA colorByID = VisualizationHelper::getMeshColor(element.identifier);
                pointCloudPoint.color = colorByID;
                ROS_DEBUG_STREAM("Got color (" << colorByID.r << ", " << colorByID.g << ", " << colorByID.b << ", " << colorByID.a << ") for id " << element.identifier);


				// add point to array
				pointCloudPtr->push_back(pointCloudPoint);
			}

			// the active indices.
			IndicesPtr activeIndicesPtr = IndicesPtr(new Indices(msg.elements.size()));
			boost::range::iota(boost::iterator_range<Indices::iterator>(activeIndicesPtr->begin(), activeIndicesPtr->end()), 0);


			// set the point cloud
			this->setActiveIndices(activeIndicesPtr);
			this->setPointCloudPtr(pointCloudPtr);
			return true;
		}

        /**
         * Returns the path to a meshs resource file
         */
        std::string getMeshPathByName(std::string objectName)
        {
            if(this->objectsResources.find(objectName) != this->objectsResources.end())
            {
                return this->objectsResources[objectName];
            }
            else
            {
                return "-2";
            }
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

		/*!
		 * \brief sets the active indices.
		 */
		void setActiveIndices(const IndicesPtr &activeIndicesPtr) {
            mActiveIndicesPtr = activeIndicesPtr;
		}

		/*!
		 * \return the active Indices.
		 */
		IndicesPtr getActiveIndices() {
			return mActiveIndicesPtr;
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
