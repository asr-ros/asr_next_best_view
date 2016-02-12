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
#include "next_best_view/helper/ObjectHelper.h"
#include "next_best_view/helper/VisualizationsHelper.hpp"

#include "pcl-1.7/pcl/filters/impl/crop_box.hpp"

#include <rapid_xml/rapidxml.hpp>
#include <rapid_xml/rapidxml_utils.hpp>

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
        std::vector<CropBoxPtr> mCropBoxPtrList;
        bool mEnableCropBoxFiltering;
        bool mEnableIntermediateObjectWeighting;

        int mMaxIterationSteps;

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
		 * Calculates the next best view. Starting point of iterative calculations for getNextBestView() service call.
		 */
		bool calculateNextBestView(const ViewportPoint &currentCameraViewport, ViewportPoint &resultViewport) {
            std::clock_t begin = std::clock();
			ROS_DEBUG("Starting calculation of next best view");

			//Calculate robot configuration corresponding to current camera viewport of robot.
			RobotStatePtr currentState = mRobotModelPtr->calculateRobotState(currentCameraViewport.getPosition(), currentCameraViewport.getSimpleQuaternion());
			//Save it.
			mRobotModelPtr->setCurrentRobotState(currentState);

			ROS_DEBUG("Calculate discrete set of view orientations on unit sphere");
			//Get discretized set of camera orientations (pan, tilt) that are to be considered at each robot position considered during iterative optimization.
			SimpleQuaternionCollectionPtr sampledOrientationsPtr = mUnitSphereSamplerPtr->getSampledUnitSphere();
			SimpleQuaternionCollectionPtr feasibleOrientationsCollectionPtr(new SimpleQuaternionCollection());

			BOOST_FOREACH(SimpleQuaternion q, *sampledOrientationsPtr) {
				if (mRobotModelPtr->isPoseReachable(SimpleVector3(0, 0, 0), q)) {
					feasibleOrientationsCollectionPtr->push_back(q);
				}
			}
			ROS_DEBUG("ENDING CALCULATE-NEXT-BEST-VIEW METHOD");
			// create the next best view point cloud
            bool success = this->doIteration(currentCameraViewport, feasibleOrientationsCollectionPtr, resultViewport);



            std::clock_t end = std::clock();
            double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
            ROS_INFO_STREAM("Iteration took " << elapsed_secs << " seconds.");
            return success;
		}

		void getFeasibleSamplePoints(const SamplePointCloudPtr &sampledSpacePointCloudPtr, IndicesPtr &resultIndicesPtr) {
			resultIndicesPtr = IndicesPtr(new Indices());
			// get max radius by far clipping plane of camera. this marks the limit for the visible object distance.
			double radius = mCameraModelFilterPtr->getFarClippingPlane();
			//Go through all 
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
		  ROS_DEBUG("STARTING DOITERATION METHOD");
		  
		  int iterationStep = 0;
		  //Best viewport at the end of each iteration step and starting point for optimization (grid alignment) for each following step.
		  ViewportPoint currentBestViewport = currentCameraViewport;

		  //Enables to interrupt iterating if it takes too long.
		  while (ros::ok()) {
		    ViewportPoint intermediateResultViewport;
		    
		    //Contractor is always divided by two.
		    if (!this->doIterationStep(currentCameraViewport, currentBestViewport,
					       sampledOrientationsPtr, 1.0 / pow(2.0, iterationStep),
					       intermediateResultViewport)) {
		      //Happens, when no valid viewport is found in that iteration step (including current viewport). E.g. when all normals are invalidated.
		      return false;
		    }
			//Iteration step must be increased before the following check.
		    iterationStep ++;
		    DefaultScoreContainerPtr drPtr = intermediateResultViewport.score;
		    ROS_DEBUG("THIS IS THE BEST VIEWPORT IN THE GIVEN ITERATION STEP.");
		    ROS_DEBUG("x: %f, y: %f, z: %f",intermediateResultViewport.x, intermediateResultViewport.y, intermediateResultViewport.z);
		    ROS_DEBUG("Utility: %f, Costs: %f, Rating: %f",drPtr->getUtility(), drPtr->getInverseCosts(), mRatingModulePtr->getRating(drPtr));
		    ROS_DEBUG("Translation costs: %f, Rotation costs: %f, PTU movement costs: %f, Recognition costs: %f",
			      drPtr->getInverseMovementCostsBaseTranslation(), drPtr->getInverseMovementCostsBaseRotation(),
			      drPtr->getInverseMovementCostsPTU(), drPtr->getInverseRecognitionCosts());
		    ROS_DEBUG("IterationStep: %i",iterationStep);
		    
			//First condition is runtime optimization to not iterate around current pose. Second is general abort criterion.
		    if (currentCameraViewport.getPosition() == intermediateResultViewport.getPosition() ||
                        (intermediateResultViewport.getPosition() - currentBestViewport.getPosition()).lpNorm<2>() <= this->getEpsilon() || iterationStep >= mMaxIterationSteps) {
		      //Stop once position displacement (resp. differing view at sufficient space sampling resolution) is small enough.
			  resultViewport = intermediateResultViewport;
		      ROS_INFO_STREAM ("Next-best-view estimation SUCCEEDED. Took " << iterationStep << " iterations");
		      ROS_DEBUG("THIS IS THE BEST VIEWPORT FOR ALL ITERATION STEPS.");
		      ROS_DEBUG("x: %f, y: %f, z: %f",intermediateResultViewport.x, intermediateResultViewport.y, intermediateResultViewport.z);
		      ROS_DEBUG("Utility: %f, Costs: %f, Rating: %f",drPtr->getUtility(), drPtr->getInverseCosts(), mRatingModulePtr->getRating(drPtr));
		      ROS_DEBUG("Translation costs: %f, Rotation costs: %f, PTU movement costs: %f, Recognition costs: %f",
				drPtr->getInverseMovementCostsBaseTranslation(), drPtr->getInverseMovementCostsBaseRotation(),
				drPtr->getInverseMovementCostsPTU(), drPtr->getInverseRecognitionCosts());
		      ROS_DEBUG("IterationStep: %i",iterationStep);
		      return true;
		    }

		    currentBestViewport = intermediateResultViewport;

		  }
		  ROS_DEBUG("ENDING DOITERATION METHOD");
		  //Only reached when iteration fails or is interrupted.
		  return false;
		}


      bool doIterationStep(const ViewportPoint &currentCameraViewport, const ViewportPoint &currentBestViewport,
			   const SimpleQuaternionCollectionPtr &sampledOrientationsPtr, float contractor,
			   ViewportPoint &resultViewport) {
		    ROS_DEBUG("STARTING DOITERATIONSTEP METHOD");

			// current camera position
			SimpleVector3 currentBestPosition = currentBestViewport.getPosition();

			//Calculate hex grid for resolution given in this iteration step.
			SamplePointCloudPtr sampledSpacePointCloudPtr = mSpaceSamplerPtr->getSampledSpacePointCloud(currentBestPosition, contractor);

			IndicesPtr feasibleIndicesPtr;
			//Prune space sample points in that iteration step by checking whether there are any surrounding object points (within constant far-clipping plane).
			this->getFeasibleSamplePoints(sampledSpacePointCloudPtr, feasibleIndicesPtr);

			//Skip rating all orientations (further code here) if we can only consider our current best robot position and increase sampling resolution			
			if (feasibleIndicesPtr->size() == 1 && this->getEpsilon() < contractor) {
				ROS_DEBUG("No RViz visualization for this iteration step, since no new next-best-view found for that resolution.");				
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
				ROS_DEBUG("Iterating over all orientations for a given robot position.");
				BOOST_FOREACH(SimpleQuaternion orientation, *sampledOrientationsPtr) {
				  // get the corresponding viewport
				  ViewportPoint fullViewportPoint;
				  //Calculate which object points lie within frustum for given viewport.
				  if (!this->doFrustumCulling(samplePointCoords, orientation, samplePointChildIndices, fullViewportPoint)) {
				    //Skip viewport if no object point is within frustum.
				    continue;
				  }
				  //For given viewport(combination of robot position and camera viewing direction)
				  // get combination of objects (all present in frustum) to search for and the corresponding score for viewport, given that combination.
				 ROS_DEBUG("Getting viewport with optimal object constellation for given position & orientation combination."); 
				 if (!mRatingModulePtr->setBestScoreContainer(currentCameraViewport, fullViewportPoint)) {
				    continue;
				  }
				  //Keep viewport with optimal subset of objects within frustum to search for.
				  nextBestViewports->push_back(fullViewportPoint);
				}
			}
			ROS_DEBUG("Sorted list of all viewports (each best for pos & orient combi) in this iteration step.");
			if (!mRatingModulePtr->getBestViewport(nextBestViewports, resultViewport)) {
			  ROS_DEBUG("ENDING DOITERATIONSTEP METHOD");               
			  return false;
			}
			ROS_DEBUG("ENDING DOITERATIONSTEP METHOD");
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
			//Call wrapper (with next-best-view data structures) for PCL frustum culling call.
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
	ObjectPointCloudPtr originalPointCloudPtr = ObjectPointCloudPtr(new ObjectPointCloud());
	ObjectPointCloudPtr croppedPointCloudPtr = ObjectPointCloudPtr(new ObjectPointCloud());

	ObjectHelper objectHelper;

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
	  ObjectMetaDataResponsePtr responsePtr_ObjectData = objectHelper.getObjectMetaData(pointCloudPoint.type);

	  if (responsePtr_ObjectData) {
	    // translating from std::vector<geometry_msgs::Point> to std::vector<SimpleVector3>
	    int normalVectorCount = 0;
	    BOOST_FOREACH(geometry_msgs::Point point, responsePtr_ObjectData->normal_vectors) {
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
	  objectsResources[pointCloudPoint.type] = responsePtr_ObjectData->object_mesh_resource;

	  //Insert color
	  std_msgs::ColorRGBA colorByID = VisualizationHelper::getMeshColor(element.identifier);
	  pointCloudPoint.color = colorByID;

	  if(mEnableIntermediateObjectWeighting)
	    {
	      //get the weight for the object from world model
	      IntermediateObjectWeightResponsePtr responsePtr_Intermediate = objectHelper.getIntermediateObjectValue(pointCloudPoint.type);

	      if(responsePtr_Intermediate)
		{
		  pointCloudPoint.intermediate_object_weight = responsePtr_Intermediate->value;
		  //ROS_ERROR_STREAM("Set input cloud " << responsePtr_Intermediate->value);
		}
	      else
		{
		  ROS_ERROR("Invalid object name %s or world model service call failed. Point Cloud not set!", pointCloudPoint.type.c_str());
		  return false;
		}
	    }
	  else
	    {
	      pointCloudPoint.intermediate_object_weight = 1;
	    }

	  // add point to array
	  originalPointCloudPtr->push_back(pointCloudPoint);
	}

	ObjectPointCloudPtr outputPointCloudPtr;
	if(mEnableCropBoxFiltering)
	  {
	    //Filter the point cloud
	    for(std::vector<CropBoxPtr>::const_iterator it=mCropBoxPtrList.begin(); it!=mCropBoxPtrList.end(); ++it)
	      {
		ObjectPointCloudPtr outputTempPointCloudPtr = ObjectPointCloudPtr(new ObjectPointCloud);
		(*it)->setInputCloud(originalPointCloudPtr);
		(*it)->filter(*outputTempPointCloudPtr);
		(*croppedPointCloudPtr) += (*outputTempPointCloudPtr);
	      }

	    ROS_DEBUG_STREAM("setPointCloudFromMessage::Filtering point cloud finished.");

	    mVisHelper.triggerCropBoxVisualization(mCropBoxPtrList);

	    outputPointCloudPtr = croppedPointCloudPtr;
	  }
	else
	  {
	    outputPointCloudPtr = originalPointCloudPtr;
	  }

	//If point cloud is empty, getting the indices lead to an Error.
	if(outputPointCloudPtr->size() > 0)
	  {
	    // the active indices.
	    IndicesPtr activeIndicesPtr = IndicesPtr(new Indices(outputPointCloudPtr->size()));
	    boost::range::iota(boost::iterator_range<Indices::iterator>(activeIndicesPtr->begin(), activeIndicesPtr->end()), 0);


	    // set the point cloud
	    this->setActiveIndices(activeIndicesPtr);
	  }
	else
	  {
	    ROS_DEBUG_STREAM("setPointCloudFromMessage::output point cloud is empty.");
	  }

	this->setPointCloudPtr(outputPointCloudPtr);
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

        void loadCropBoxListFromFile(const std::string mCropBoxListFilePath)
        {
            readCropBoxDataFromXMLFile(mCropBoxListFilePath);
        }

        void setEnableCropBoxFiltering(const bool mEnableCropBoxFiltering)
        {
            this->mEnableCropBoxFiltering = mEnableCropBoxFiltering;
        }

        void setEnableIntermediateObjectWeighting(const bool mEnableIntermediateObjectWeighting)
        {
            this->mEnableIntermediateObjectWeighting = mEnableIntermediateObjectWeighting;
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

       /**
	 * Max number of hierarchical iterations when calculating a next-best-view.
         * @param maxIterationSteps  - max number of iteration steps
         */
        void setMaxIterationSteps(int maxIterationSteps)
        {
            mMaxIterationSteps  = maxIterationSteps;
        }

        //TODO : REFACTOR
        void readCropBoxDataFromXMLFile(std::string mCropBoxListFilePath)
        {
            std::string xml_path = mCropBoxListFilePath;
            ROS_DEBUG_STREAM("Path to CropBoxList xml file: " << xml_path);
            try {
                rapidxml::file<> xmlFile(xml_path.c_str());
                rapidxml::xml_document<> doc;
                doc.parse<0>(xmlFile.data());

                rapidxml::xml_node<> *root_node = doc.first_node();
                if (root_node) {
                    rapidxml::xml_node<> *child_node = root_node->first_node();
                    while (child_node)
                    {
                        CropBoxPtr bufferCropBoxPtr = CropBoxPtr(new CropBox);
                        rapidxml::xml_node<> * min_pt = child_node->first_node("min_pt");
                        rapidxml::xml_attribute<> *x = min_pt->first_attribute("x");
                        rapidxml::xml_attribute<> *y = min_pt->first_attribute("y");
                        rapidxml::xml_attribute<> *z = min_pt->first_attribute("z");
                        if (x && y && z)
                        {
                            double x_ = boost::lexical_cast<double>(x->value());
                            double y_ = boost::lexical_cast<double>(y->value());
                            double z_ = boost::lexical_cast<double>(z->value());
                            Eigen::Vector4f pt_min(x_,y_,z_,1);
                            bufferCropBoxPtr->setMin(pt_min);
                        }

                        rapidxml::xml_node<> * max_pt = child_node->first_node("max_pt");
                        x = max_pt->first_attribute("x");
                        y = max_pt->first_attribute("y");
                        z = max_pt->first_attribute("z");
                        if (x && y && z)
                        {
                            double x_ = boost::lexical_cast<double>(x->value());
                            double y_ = boost::lexical_cast<double>(y->value());
                            double z_ = boost::lexical_cast<double>(z->value());
                            Eigen::Vector4f pt_max(x_,y_,z_,1);
                            bufferCropBoxPtr->setMax(pt_max);
                        }

                        rapidxml::xml_node<> * rotation = child_node->first_node("rotation");
                        x = rotation->first_attribute("x");
                        y = rotation->first_attribute("y");
                        z = rotation->first_attribute("z");
                        if (x && y && z)
                        {
                            double x_ = boost::lexical_cast<double>(x->value());
                            double y_ = boost::lexical_cast<double>(y->value());
                            double z_ = boost::lexical_cast<double>(z->value());
                            Eigen::Vector3f rotation(x_,y_,z_);
                            bufferCropBoxPtr->setRotation(rotation);
                        }

                        rapidxml::xml_node<> * translation = child_node->first_node("translation");
                        x = translation->first_attribute("x");
                        y = translation->first_attribute("y");
                        z = translation->first_attribute("z");
                        if (x && y && z)
                        {
                            double x_ = boost::lexical_cast<double>(x->value());
                            double y_ = boost::lexical_cast<double>(y->value());
                            double z_ = boost::lexical_cast<double>(z->value());
                            Eigen::Vector3f translation(x_,y_,z_);
                            bufferCropBoxPtr->setTranslation(translation);
                        }
                        mCropBoxPtrList.push_back(bufferCropBoxPtr);
                        child_node = child_node->next_sibling();
                    }
                }
            } catch(std::runtime_error err) {
                ROS_ERROR_STREAM("Can't parse xml-file. Runtime error: " << err.what());
            } catch (rapidxml::parse_error err) {
                ROS_ERROR_STREAM("Can't parse xml-file Parse error: " << err.what());
            }
        }

	};
}


#endif /* NEXTBESTVIEWCALCULATOR_HPP_ */
